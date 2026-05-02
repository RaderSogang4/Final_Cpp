import math
import mmap
import struct
import time
from typing import Dict, List, Tuple

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


Point = Tuple[float, float, float, float, float, int]
Target = Tuple[float, float, int, int, float, float, float, float, float, float]


def pack_header(
    mm: mmap.mmap,
    sequence: int,
    state: int,
    frame_count: int,
    packet_size: int,
    point_count: int,
    target_count: int,
    timestamp_ns: int,
) -> None:
    HEADER_STRUCT.pack_into(
        mm,
        0,
        MAGIC,
        VERSION,
        HEADER_SIZE,
        TOTAL_SIZE,
        sequence,
        state,
        frame_count,
        packet_size,
        point_count,
        target_count,
        MAX_POINTS,
        MAX_TARGETS,
        timestamp_ns,
        POINTS_OFFSET,
        TARGETS_OFFSET,
        *([0] * 16),
    )


def make_dummy_points(frame_count: int, count: int = 1600) -> List[Point]:
    t = frame_count * 0.05
    points: List[Point] = []

    for i in range(count):
        ring = i % 80
        layer = i // 80

        angle = ring / 80.0 * math.tau + t
        radius = 0.2 + 1.6 * (layer / max(1, count // 80))

        x = math.cos(angle) * radius
        y = 1.0 + 7.0 * (layer / max(1, count // 80))
        z = math.sin(angle) * radius * 0.7

        doppler = math.sin(angle + t) * 0.8
        power = 2500.0 + 2500.0 * abs(math.sin(angle * 0.5 + t))

        target_id = layer % 5

        points.append((x, y, z, doppler, power, target_id))

    return points


def make_targets(points: List[Point]) -> List[Target]:
    boxes: Dict[int, List[float]] = {}

    for x, y, z, doppler, power, target_id in points:
        if target_id not in boxes:
            boxes[target_id] = [x, x, y, y, z, z]
        else:
            b = boxes[target_id]
            b[0] = min(b[0], x)
            b[1] = max(b[1], x)
            b[2] = min(b[2], y)
            b[3] = max(b[3], y)
            b[4] = min(b[4], z)
            b[5] = max(b[5], z)

    targets: List[Target] = []

    for target_id, b in sorted(boxes.items()):
        minx, maxx, miny, maxy, minz, maxz = b
        cx = (minx + maxx) * 0.5
        cy = (miny + maxy) * 0.5
        status = 4  # Walking

        targets.append(
            (
                cx,
                cy,
                status,
                target_id,
                minx,
                maxx,
                miny,
                maxy,
                minz,
                maxz,
            )
        )

    return targets[:MAX_TARGETS]


def write_frame(
    mm: mmap.mmap,
    frame_count: int,
    points: List[Point],
    targets: List[Target],
    state: int = STATE_RECEIVING,
) -> None:
    old_header = HEADER_STRUCT.unpack_from(mm, 0)
    sequence = old_header[4]

    if sequence & 1:
        sequence += 1

    point_count = min(len(points), MAX_POINTS)
    target_count = min(len(targets), MAX_TARGETS)
    timestamp_ns = time.time_ns()

    pack_header(
        mm,
        sequence + 1,
        state,
        frame_count,
        0,
        point_count,
        target_count,
        timestamp_ns,
    )

    for i in range(point_count):
        POINT_STRUCT.pack_into(
            mm,
            POINTS_OFFSET + i * POINT_SIZE,
            *points[i],
        )

    for i in range(target_count):
        TARGET_STRUCT.pack_into(
            mm,
            TARGETS_OFFSET + i * TARGET_SIZE,
            *targets[i],
        )

    pack_header(
        mm,
        sequence + 2,
        state,
        frame_count,
        0,
        point_count,
        target_count,
        timestamp_ns,
    )

    mm.flush()


def main() -> None:
    print(f"[dummy] creating shared memory: {SHM_NAME}")
    print(f"[dummy] size: {TOTAL_SIZE} bytes")

    mm = mmap.mmap(
        -1,
        TOTAL_SIZE,
        tagname=SHM_NAME,
        access=mmap.ACCESS_WRITE,
    )

    pack_header(
        mm,
        sequence=0,
        state=STATE_STOPPED,
        frame_count=0,
        packet_size=0,
        point_count=0,
        target_count=0,
        timestamp_ns=time.time_ns(),
    )

    frame_count = 0

    try:
        while True:
            points = make_dummy_points(frame_count)
            targets = make_targets(points)

            write_frame(
                mm,
                frame_count=frame_count,
                points=points,
                targets=targets,
                state=STATE_RECEIVING,
            )

            print(
                f"[dummy] frame={frame_count:<8} "
                f"points={len(points):<6} "
                f"targets={len(targets):<4}",
                end="\r",
                flush=True,
            )

            frame_count += 1
            time.sleep(1.0 / 30.0)

    except KeyboardInterrupt:
        print("\n[dummy] stopping")
        pack_header(
            mm,
            sequence=HEADER_STRUCT.unpack_from(mm, 0)[4] + 2,
            state=STATE_STOPPED,
            frame_count=frame_count,
            packet_size=0,
            point_count=0,
            target_count=0,
            timestamp_ns=time.time_ns(),
        )

    finally:
        mm.close()


if __name__ == "__main__":
    main()
