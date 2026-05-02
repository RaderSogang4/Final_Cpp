using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

/// <summary>
/// Python radar_tcp_sender.py 로부터 트래킹 데이터를 수신합니다.
///
/// 패킷 구조:
///   [4B: packet_length] [4B: track_count] [ ID(4B) + x(4B) + y(4B) + z(4B) ] × N
///
/// 사용법:
///   1. 빈 GameObject 에 이 컴포넌트를 추가합니다.
///   2. Python radar_tcp_sender.py 를 먼저 실행합니다.
///   3. Unity Play 버튼을 누르면 자동 연결됩니다.
///   4. OnTracksUpdated 이벤트를 구독하거나
///      GetLatestTracks() 로 매 프레임 데이터를 가져옵니다.
/// </summary>
public class RadarReceiver : MonoBehaviour
{
    // ── Inspector 설정 ──────────────────────────
    [Header("Connection")]
    public string host = "127.0.0.1";
    public int    port = 9000;

    [Header("Debug")]
    public bool showGizmos     = true;
    public bool logEveryFrame  = false;

    // ── 이벤트 (외부 구독용) ───────────────────
    /// <summary>새 트래킹 데이터가 도착할 때마다 호출됩니다 (백그라운드 스레드).</summary>
    public event Action<List<RadarTrack>> OnTracksUpdated;

    // ── 공개 구조체 ────────────────────────────
    public struct RadarTrack
    {
        public int   id;
        public float x, y, z;
        public Vector3 Position => new Vector3(x, y, z);
        public override string ToString() =>
            $"ID={id:D2}  ({x:F3}, {y:F3}, {z:F3})";
    }

    // ── 내부 상태 ──────────────────────────────
    private List<RadarTrack> _latestTracks = new List<RadarTrack>();
    private readonly object  _lock         = new object();

    private TcpClient  _client;
    private Thread     _thread;
    private bool       _running;

    // ── Unity 생명주기 ─────────────────────────
    void Start()
    {
        _running = true;
        _thread  = new Thread(ReceiveLoop) { IsBackground = true };
        _thread.Start();
        Debug.Log($"[Radar] Python 서버 연결 시도 중 {host}:{port}");
    }

    void Update()
    {
        if (!logEveryFrame) return;

        List<RadarTrack> tracks = GetLatestTracks();
        if (tracks.Count == 0) return;

        foreach (var t in tracks)
            Debug.Log($"[Radar] {t}");
    }

    void OnDestroy()
    {
        _running = false;
        _thread?.Join(500);
        _client?.Close();
    }

    // ── 공개 API ───────────────────────────────
    /// <summary>가장 최근 트래킹 결과를 반환합니다 (Thread-safe).</summary>
    public List<RadarTrack> GetLatestTracks()
    {
        lock (_lock)
            return new List<RadarTrack>(_latestTracks);
    }

    // ── 수신 루프 (백그라운드 스레드) ──────────
    void ReceiveLoop()
    {
        while (_running)
        {
            try
            {
                _client = new TcpClient(host, port);
                Debug.Log($"[Radar] Python 서버 연결 성공 ({host}:{port})");

                NetworkStream stream = _client.GetStream();

                while (_running)
                {
                    // 1. packet_length 읽기 (4 bytes)
                    byte[] lenBuf = ReadExact(stream, 4);
                    if (lenBuf == null) break;
                    int packetLen = (int)BitConverter.ToUInt32(lenBuf, 0);

                    // 2. 본문 읽기
                    byte[] body = ReadExact(stream, packetLen);
                    if (body == null) break;

                    // 3. 파싱
                    List<RadarTrack> tracks = ParseTracks(body);

                    // 4. 스레드 안전 업데이트
                    lock (_lock) { _latestTracks = tracks; }

                    // 5. 이벤트 발행
                    OnTracksUpdated?.Invoke(tracks);
                }
            }
            catch (Exception e)
            {
                if (_running)
                {
                    Debug.LogWarning($"[Radar] 연결 오류: {e.Message} — 1초 후 재시도");
                    Thread.Sleep(1000);
                }
            }
            finally
            {
                _client?.Close();
                _client = null;
            }
        }
    }

    // ── 파싱 ───────────────────────────────────
    static List<RadarTrack> ParseTracks(byte[] body)
    {
        int count  = (int)BitConverter.ToUInt32(body, 0);
        var result = new List<RadarTrack>(count);

        for (int i = 0; i < count; i++)
        {
            int offset = 4 + i * 16;  // 헤더 4B + 객체당 16B
            result.Add(new RadarTrack
            {
                id = (int)BitConverter.ToUInt32(body, offset),
                x  = BitConverter.ToSingle(body, offset + 4),
                y  = BitConverter.ToSingle(body, offset + 8),
                z  = BitConverter.ToSingle(body, offset + 12),
            });
        }
        return result;
    }

    // ── 헬퍼: 정확히 size 바이트 읽기 ─────────
    static byte[] ReadExact(NetworkStream stream, int size)
    {
        byte[] buf      = new byte[size];
        int    received = 0;
        while (received < size)
        {
            int n = stream.Read(buf, received, size - received);
            if (n == 0) return null;  // 연결 종료
            received += n;
        }
        return buf;
    }

    // ── Gizmo 시각화 ───────────────────────────
    void OnDrawGizmos()
    {
        if (!showGizmos || !Application.isPlaying) return;

        List<RadarTrack> tracks = GetLatestTracks();
        foreach (var t in tracks)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(t.Position, 0.15f);

#if UNITY_EDITOR
            UnityEditor.Handles.Label(
                t.Position + Vector3.up * 0.3f,
                $"ID:{t.id}"
            );
#endif
        }
    }
}
