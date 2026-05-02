"""
Shared-memory radar tracker -> TCP bridge for Unity.
When TEST_IN_WEB = True, also starts:
  - WebSocket server (port 8765) that broadcasts track JSON to browsers
  - HTTP file server (port 8080) that serves radar_simulator.html

Packet format, little-endian:
    uint32 payload_length
    uint32 track_count
    repeated track_count times:
        float32 id
        float32 x
        float32 y
        float32 z

payload_length is the byte size after the length field:
    4 + track_count * 16

Usage:
    python radar_tcp_sender.py                           # Unity only
    python radar_tcp_sender.py --test-in-web              # Unity + browser simulator
    python radar_tcp_sender.py --test-in-web --ws-port 8765 --http-port 8080
"""

import argparse
import asyncio
import http.server
import json
import os
import socket
import struct
import threading
import time
import webbrowser
from typing import Iterable, List, Optional, Sequence, Set, Tuple

import cluster_radar_shared_memory as tracking


# ═══════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════

TEST_IN_WEB = True          # ← 여기를 True로 바꾸거나 --test-in-web 플래그 사용

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9000
DEFAULT_FPS = 30.0
DEFAULT_WS_PORT = 8765
DEFAULT_HTTP_PORT = 8080

TrackTuple = Tuple[float, float, float, float]


# ═══════════════════════════════════════════════════════════
#  TCP packet helpers (기존 그대로)
# ═══════════════════════════════════════════════════════════

def row_to_tracks(row: Sequence[float]) -> List[TrackTuple]:
    if len(row) % 4 != 0:
        raise ValueError(f"tracking row length must be a multiple of 4, got {len(row)}")

    tracks: List[TrackTuple] = []
    for i in range(0, len(row), 4):
        tracks.append(
            (
                float(row[i]),
                float(row[i + 1]),
                float(row[i + 2]),
                float(row[i + 3]),
            )
        )
    return tracks


def build_packet(tracks: Iterable[TrackTuple]) -> bytes:
    track_list = list(tracks)
    payload = bytearray()
    payload += struct.pack("<I", len(track_list))

    for track_id, x, y, z in track_list:
        payload += struct.pack("<ffff", float(track_id), float(x), float(y), float(z))

    return struct.pack("<I", len(payload)) + payload


def load_tracker_config(config_path: Optional[str]) -> tracking.TrackerConfig:
    return tracking.TrackerConfig(**tracking.load_config_defaults(config_path))


def accept_client(server: socket.socket) -> socket.socket:
    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[tcp] Unity connected: {addr}", flush=True)
    return conn


# ═══════════════════════════════════════════════════════════
#  WebSocket broadcast (TEST_IN_WEB 전용)
# ═══════════════════════════════════════════════════════════

_ws_clients: Set = set()
_ws_latest_json: Optional[str] = None
_ws_loop: Optional[asyncio.AbstractEventLoop] = None


def _tracks_to_json(tracks: List[TrackTuple], frame_count: int) -> str:
    """트랙 리스트를 브라우저용 JSON으로 변환."""
    return json.dumps({
        "frame": frame_count,
        "track_count": len(tracks),
        "tracks": [
            {"id": int(round(tid)), "x": round(x, 4), "y": round(y, 4), "z": round(z, 4)}
            for tid, x, y, z in tracks
        ],
    })


def broadcast_to_web(tracks: List[TrackTuple], frame_count: int) -> None:
    """메인 스레드에서 호출: 트랙 데이터를 WebSocket 클라이언트들에게 전송 예약."""
    global _ws_latest_json
    _ws_latest_json = _tracks_to_json(tracks, frame_count)


async def _ws_handler(websocket):
    _ws_clients.add(websocket)
    addr = websocket.remote_address
    print(f"[ws] 브라우저 연결: {addr}", flush=True)
    try:
        async for _ in websocket:
            pass
    finally:
        _ws_clients.discard(websocket)
        print(f"[ws] 브라우저 종료: {addr}", flush=True)


async def _ws_broadcast_loop():
    """_ws_latest_json이 갱신될 때마다 모든 WebSocket 클라이언트에게 전송."""
    global _ws_latest_json
    last_sent = None
    while True:
        if _ws_latest_json is not None and _ws_latest_json != last_sent:
            last_sent = _ws_latest_json
            if _ws_clients:
                await asyncio.gather(
                    *[c.send(last_sent) for c in _ws_clients],
                    return_exceptions=True,
                )
        await asyncio.sleep(0.008)


async def _run_ws_server(ws_port: int):
    from websockets import serve as ws_serve
    print(f"[ws] WebSocket 서버 시작: ws://0.0.0.0:{ws_port}", flush=True)
    async with ws_serve(_ws_handler, "0.0.0.0", ws_port):
        await _ws_broadcast_loop()


def start_ws_server(ws_port: int) -> None:
    """WebSocket 서버를 별도 스레드에서 실행."""
    try:
        import websockets  # noqa: F401
    except ImportError:
        print("[ws] websockets 패키지 필요: pip install websockets", flush=True)
        print("[ws] WebSocket 서버를 시작하지 않습니다.", flush=True)
        return

    def _thread():
        global _ws_loop
        _ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(_ws_loop)
        _ws_loop.run_until_complete(_run_ws_server(ws_port))

    t = threading.Thread(target=_thread, daemon=True)
    t.start()


# ═══════════════════════════════════════════════════════════
#  HTTP file server (TEST_IN_WEB 전용)
# ═══════════════════════════════════════════════════════════

def start_http_server(http_port: int, ws_port: int) -> None:
    """radar_simulator.html 을 제공하는 간단한 HTTP 서버."""
    html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "radar_simulator.html")

    if not os.path.exists(html_path):
        print(f"[http] radar_simulator.html 을 찾을 수 없습니다: {html_path}", flush=True)
        print(f"[http] radar_tcp_sender.py 와 같은 폴더에 radar_simulator.html 을 두세요.", flush=True)
        return

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/" or self.path == "/radar_simulator.html":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                with open(html_path, "rb") as f:
                    content = f.read()
                # HTML 안의 기본 WS URL을 실제 포트에 맞게 치환
                content = content.replace(
                    b"ws://127.0.0.1:8765",
                    f"ws://127.0.0.1:{ws_port}".encode(),
                )
                self.wfile.write(content)
            else:
                self.send_error(404)

        def log_message(self, format, *args):
            pass  # 조용히

    server = http.server.HTTPServer(("0.0.0.0", http_port), Handler)
    print(f"[http] 시뮬레이터 URL: http://127.0.0.1:{http_port}", flush=True)

    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()


# ═══════════════════════════════════════════════════════════
#  Main send loop (TCP + optional WebSocket)
# ═══════════════════════════════════════════════════════════

def send_loop(
    host: str,
    port: int,
    fps: float,
    config_path: Optional[str],
    print_tracks: bool,
    test_in_web: bool,
    ws_port: int,
    http_port: int,
    open_browser: bool,
) -> None:
    config = load_tracker_config(config_path)
    tracker = tracking.ClusterTracker(config)
    mm = tracking.open_shared_memory()

    frame_interval = 1.0 / fps if fps > 0.0 else 0.0
    last_frame_count: Optional[int] = None

    # ── WebSocket + HTTP (TEST_IN_WEB) ──
    if test_in_web:
        start_ws_server(ws_port)
        start_http_server(http_port, ws_port)
        time.sleep(0.3)
        url = f"http://127.0.0.1:{http_port}"
        print(f"[web] 브라우저에서 시뮬레이터 확인: {url}", flush=True)
        if open_browser:
            try:
                webbrowser.open(url)
            except Exception:
                pass

    # ── TCP server (Unity) ──
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)

    print("[tcp] Radar tracking TCP bridge started", flush=True)
    print(f"[tcp] Listening for Unity on {host}:{port}", flush=True)
    print("[tcp] Packet: <uint32 payload_length><uint32 count><float id,x,y,z>*", flush=True)

    if test_in_web:
        print("[tcp] TEST_IN_WEB=True → Unity 연결 없이도 WebSocket으로 데이터 전송 중", flush=True)

    conn: Optional[socket.socket] = None

    # TEST_IN_WEB일 때 Unity 연결을 기다리지 않고 non-blocking accept
    if test_in_web:
        server.settimeout(0.0)  # non-blocking

    try:
        if not test_in_web:
            conn = accept_client(server)

        while True:
            start = time.perf_counter()
            frame = tracking.read_frame(mm)

            if frame is None:
                print("[shm] Shared memory is open, but producer is not initialized", flush=True)
                time.sleep(1.0)
                continue

            if frame.frame_count == last_frame_count:
                time.sleep(0.001)
                continue

            last_frame_count = frame.frame_count
            row = tracker.update(frame)
            tracks = row_to_tracks(row)
            packet = build_packet(tracks)

            # ── WebSocket broadcast ──
            if test_in_web:
                broadcast_to_web(tracks, frame.frame_count)

            # ── TCP send (Unity) ──
            if conn is not None:
                try:
                    conn.sendall(packet)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    print("[tcp] Unity disconnected", flush=True)
                    try:
                        conn.close()
                    except OSError:
                        pass
                    conn = None

            # TEST_IN_WEB: 비동기로 Unity 연결 시도
            if conn is None and test_in_web:
                try:
                    conn = server.accept()[0]
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print(f"[tcp] Unity connected (late join)", flush=True)
                except BlockingIOError:
                    pass  # 아직 연결 없음 → 괜찮음
            elif conn is None and not test_in_web:
                conn = accept_client(server)

            if print_tracks:
                print(
                    f"[frame {frame.frame_count}] sent {len(tracks)} track(s): "
                    + ", ".join(
                        f"({track_id}, {x:.3f}, {y:.3f}, {z:.3f})"
                        for track_id, x, y, z in tracks
                    ),
                    flush=True,
                )

            elapsed = time.perf_counter() - start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[tcp] Stopped", flush=True)
    finally:
        if conn is not None:
            try:
                conn.close()
            except OSError:
                pass
        server.close()
        mm.close()


# ═══════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send radar tracking output to Unity over TCP (+ optional web simulator).",
    )
    parser.add_argument("--config", default=None, help="Path to cluster_config.json")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", default=DEFAULT_PORT, type=int)
    parser.add_argument("--fps", default=DEFAULT_FPS, type=float)
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Do not print per-frame track payloads.",
    )

    # ── TEST_IN_WEB 관련 ──
    parser.add_argument(
        "--test-in-web",
        action="store_true",
        default=TEST_IN_WEB,
        help="WebSocket + HTTP 서버를 함께 시작하여 브라우저 시뮬레이터 사용",
    )
    parser.add_argument("--ws-port", type=int, default=DEFAULT_WS_PORT,
                        help=f"WebSocket 포트 (기본: {DEFAULT_WS_PORT})")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT,
                        help=f"HTTP 시뮬레이터 포트 (기본: {DEFAULT_HTTP_PORT})")
    parser.add_argument("--no-browser", action="store_true",
                        help="자동 브라우저 열기 비활성화")

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    send_loop(
        host=args.host,
        port=args.port,
        fps=args.fps,
        config_path=args.config,
        print_tracks=not args.quiet,
        test_in_web=args.test_in_web,
        ws_port=args.ws_port,
        http_port=args.http_port,
        open_browser=not args.no_browser,
    )


if __name__ == "__main__":
    main()