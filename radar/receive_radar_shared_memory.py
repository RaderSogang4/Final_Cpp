import mmap
import struct
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

SHM_NAME = r"Local\RadarPointCloudSharedMemory"

MAGIC = 0x52444350
VERSION = 1

MAX_POINTS = 65536
MAX_TARGETS = 256

HEADER_STRUCT = struct.Struct("<IIIIIIIIIIIIQII16I")
POINT_STRUCT = struct.Struct("<fffffi")
TARGET_STRUCT = struct.Struct("<ffIIffffff")

HEADER_SIZE = HEADER_STRUCT.size
POINT_SIZE = POINT_STRUCT.size
TARGET_SIZE = TARGET_STRUCT.size

POINTS_OFFSET = HEADER_SIZE
TARGETS_OFFSET = POINTS_OFFSET + MAX_POINTS * POINT_SIZE
TOTAL_SIZE = TARGETS_OFFSET + MAX_TARGETS * TARGET_SIZE

STATE_STOPPED = 0
STATE_RECEIVING = 1
STATE_PAUSED = 2
STATE_ERROR = 3


@dataclass(frozen=True)
class RadarSnapshot:
    state: int
    frame_count: int
    packet_size: int
    timestamp_ns: int
    points: List[Tuple[float, float, float, float, float, int]]
    targets: List[Tuple[float, float, int, int, float, float, float, float, float, float]]


def state_to_string(state: int) -> str:
    return {
        STATE_STOPPED: "Stopped",
        STATE_RECEIVING: "Receiving",
        STATE_PAUSED: "Paused",
        STATE_ERROR: "Error",
    }.get(state, f"Unknown({state})")


def open_shared_memory() -> mmap.mmap:
    while True:
        try:
            mm = mmap.mmap(
                -1,
                TOTAL_SIZE,
                tagname=SHM_NAME,
                access=mmap.ACCESS_READ,
            )
            return mm
        except OSError as e:
            print(f"[python] waiting for shared memory: {e}")
            time.sleep(1.0)


def unpack_header(mm: mmap.mmap):
    return HEADER_STRUCT.unpack_from(mm, 0)


def read_snapshot(mm: mmap.mmap) -> Optional[RadarSnapshot]:
    while True:
        header1 = unpack_header(mm)

        magic = header1[0]
        version = header1[1]
        header_size = header1[2]
        total_size = header1[3]
        sequence1 = header1[4]

        if magic != MAGIC or version != VERSION:
            return None

        if header_size != HEADER_SIZE or total_size != TOTAL_SIZE:
            raise RuntimeError(
                f"layout mismatch: header_size={header_size}, total_size={total_size}, "
                f"expected header_size={HEADER_SIZE}, total_size={TOTAL_SIZE}"
            )

        if sequence1 & 1:
            time.sleep(0)
            continue

        state = header1[5]
        frame_count = header1[6]
        packet_size = header1[7]
        point_count = min(header1[8], MAX_POINTS)
        target_count = min(header1[9], MAX_TARGETS)
        timestamp_ns = header1[12]
        points_offset = header1[13]
        targets_offset = header1[14]

        points = [
            POINT_STRUCT.unpack_from(mm, points_offset + i * POINT_SIZE)
            for i in range(point_count)
        ]

        targets = [
            TARGET_STRUCT.unpack_from(mm, targets_offset + i * TARGET_SIZE)
            for i in range(target_count)
        ]

        sequence2 = unpack_header(mm)[4]

        if sequence1 == sequence2 and not (sequence2 & 1):
            return RadarSnapshot(
                state=state,
                frame_count=frame_count,
                packet_size=packet_size,
                timestamp_ns=timestamp_ns,
                points=points,
                targets=targets,
            )


def main() -> None:
    print(f"[python] opening shared memory: {SHM_NAME}")
    print(f"[python] expected size: {TOTAL_SIZE} bytes")

    mm = open_shared_memory()

    last_frame = None

    try:
        while True:
            snapshot = read_snapshot(mm)

            if snapshot is None:
                print("[python] shared memory exists, but C++ producer has not initialized it yet")
                time.sleep(1.0)
                continue

            if snapshot.frame_count != last_frame:
                last_frame = snapshot.frame_count

                if snapshot.points:
                    x, y, z, doppler, power, target_id = snapshot.points[0]
                    first_point = (
                        f"first=({x:.3f}, {y:.3f}, {z:.3f}), "
                        f"doppler={doppler:.3f}, power={power:.1f}, target={target_id}"
                    )
                else:
                    first_point = "first=None"

                print(
                    f"[python] state={state_to_string(snapshot.state):>9} "
                    f"frame={snapshot.frame_count:<8} "
                    f"points={len(snapshot.points):<6} "
                    f"targets={len(snapshot.targets):<4} "
                    f"{first_point}"
                )

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[python] stopped")

    finally:
        mm.close()


if __name__ == "__main__":
    main()
