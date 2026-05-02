"""
Shared-memory radar tracker -> TCP bridge for Unity.

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
"""

import argparse
import socket
import struct
import time
from typing import Iterable, List, Optional, Sequence, Tuple

import cluster_radar_shared_memory as tracking


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9000
DEFAULT_FPS = 30.0

TrackTuple = Tuple[float, float, float, float]


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


def send_loop(
    host: str,
    port: int,
    fps: float,
    config_path: Optional[str],
    print_tracks: bool,
) -> None:
    config = load_tracker_config(config_path)
    tracker = tracking.ClusterTracker(config)
    mm = tracking.open_shared_memory()

    frame_interval = 1.0 / fps if fps > 0.0 else 0.0
    last_frame_count: Optional[int] = None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)

    print("[tcp] Radar tracking TCP bridge started", flush=True)
    print(f"[tcp] Listening for Unity on {host}:{port}", flush=True)
    print("[tcp] Packet: <uint32 payload_length><uint32 count><float id,x,y,z>*", flush=True)

    conn: Optional[socket.socket] = None

    try:
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

            while True:
                try:
                    conn.sendall(packet)
                    break
                except (BrokenPipeError, ConnectionResetError, OSError):
                    print("[tcp] Unity disconnected; waiting for reconnect", flush=True)
                    try:
                        conn.close()
                    except OSError:
                        pass
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send radar tracking output to Unity over TCP.",
    )
    parser.add_argument("--config", default=None, help="Path to cluster_config.example.json")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", default=DEFAULT_PORT, type=int)
    parser.add_argument("--fps", default=DEFAULT_FPS, type=float)
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Do not print per-frame track payloads.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    send_loop(
        host=args.host,
        port=args.port,
        fps=args.fps,
        config_path=args.config,
        print_tracks=not args.quiet,
    )


if __name__ == "__main__":
    main()
