import argparse
import json
import math
import mmap
import struct
import time
from dataclasses import dataclass, fields
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


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
POINTS_OFFSET = HEADER_SIZE
TARGETS_OFFSET = POINTS_OFFSET + MAX_POINTS * POINT_SIZE
TARGET_SIZE = TARGET_STRUCT.size
TOTAL_SIZE = TARGETS_OFFSET + MAX_TARGETS * TARGET_SIZE

STATE_STOPPED = 0
STATE_RECEIVING = 1
STATE_PAUSED = 2
STATE_ERROR = 3

Point = Tuple[float, float, float, float, float, int]
Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class RadarFrame:
    state: int
    frame_count: int
    timestamp_ns: int
    points: List[Point]
    targets: List["RadarTarget"]


@dataclass(frozen=True)
class RadarTarget:
    x: float
    y: float
    status: int
    target_id: int
    minx: float
    maxx: float
    miny: float
    maxy: float
    minz: float
    maxz: float

    @property
    def box_center(self) -> Vec3:
        return (
            (self.minx + self.maxx) * 0.5,
            (self.miny + self.maxy) * 0.5,
            (self.minz + self.maxz) * 0.5,
        )


@dataclass(frozen=True)
class Detection:
    center: Vec3
    box_min: Vec3
    box_max: Vec3
    doppler: Optional[float]
    radar_target_id: Optional[int]
    point_count: int
    source: str

    @property
    def box_size(self) -> Vec3:
        return (
            self.box_max[0] - self.box_min[0],
            self.box_max[1] - self.box_min[1],
            self.box_max[2] - self.box_min[2],
        )


@dataclass
class Track:
    track_id: int
    radar_target_id: Optional[int]
    state: List[float]
    covariance: List[List[float]]
    missed: int = 0
    hits: int = 1
    last_frame: int = 0
    last_observed_center: Vec3 = (0.0, 0.0, 0.0)
    exit_candidate: bool = False
    last_exit_outward_speed: float = 0.0
    radar_id_confidence: float = 0.0
    radar_id_candidate: Optional[int] = None
    radar_id_candidate_hits: int = 0

    @property
    def center(self) -> Vec3:
        return (self.state[0], self.state[1], self.state[2])

    @property
    def output_id(self) -> int:
        return self.radar_target_id if self.radar_target_id is not None else self.track_id


@dataclass
class TrackerConfig:
    dbscan_eps: float = 0.35
    dbscan_min_samples: int = 8
    max_assignment_distance: float = 0.75
    max_mahalanobis_distance: float = 4.0
    exit_missed_frames: int = 8
    process_noise: float = 0.8
    measurement_noise: float = 0.05
    doppler_noise: float = 0.35
    doppler_sign: float = 1.0
    power_threshold: float = 0.0
    max_points: int = 12000
    center_method: str = "weighted"
    min_detection_points: int = 0
    human_min_width: float = 0.0
    human_max_width: float = 1.6
    human_min_depth: float = 0.0
    human_max_depth: float = 1.8
    human_min_height: float = 0.0
    human_max_height: float = 2.4
    prefer_radar_targets: bool = True
    radar_id_match_scale: float = 0.25
    radar_id_mismatch_penalty: float = 2.0
    radar_id_switch_hits: int = 3
    radar_id_confidence_up: float = 0.2
    radar_id_confidence_down: float = 0.1
    output_radar_id: bool = False
    output_lost_tracks: bool = True
    exit_door_enabled: bool = False
    exit_door_x_min: float = 0.0
    exit_door_x_max: float = 0.0
    exit_door_y_min: float = 0.0
    exit_door_y_max: float = 0.0
    exit_door_z_min: float = -10.0
    exit_door_z_max: float = 10.0
    exit_door_outward_x: float = -1.0
    exit_door_outward_y: float = 0.0
    exit_door_outward_z: float = 0.0
    exit_door_boundary_margin: float = 0.25
    exit_door_min_outward_speed: float = 0.15
    exit_door_require_outward_motion: bool = True
    exit_door_require_boundary_proximity: bool = True


class ClusterTracker:
    def __init__(self, config: TrackerConfig):
        self.config = config
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1
        self.last_timestamp_ns: Optional[int] = None

    def update(self, frame: RadarFrame) -> List[float]:
        dt = self._compute_dt(frame.timestamp_ns)
        detections = build_detections(frame, self.config)

        for track in self.tracks.values():
            kalman_predict(track, dt, self.config.process_noise)

        assignments, unmatched_tracks, unmatched_detections = assign_tracks(
            list(self.tracks.values()),
            detections,
            self.config.max_assignment_distance,
            self.config.max_mahalanobis_distance,
            self.config.measurement_noise,
            self.config.radar_id_match_scale,
            self.config.radar_id_mismatch_penalty,
        )

        track_list = list(self.tracks.values())
        for track_index, detection_index in assignments:
            track = track_list[track_index]
            detection = detections[detection_index]
            kalman_update_position(track, detection.center, self.config.measurement_noise)
            if detection.doppler is not None:
                kalman_update_doppler(
                    track,
                    detection.center,
                    detection.doppler * self.config.doppler_sign,
                    self.config.doppler_noise,
                )
            update_track_radar_id(track, detection, self.config)
            track.missed = 0
            track.hits += 1
            track.last_frame = frame.frame_count
            track.last_observed_center = detection.center
            update_exit_candidate(track, self.config)

        for track_index in unmatched_tracks:
            track_list[track_index].missed += 1
            track_list[track_index].radar_id_confidence = max(
                0.0,
                track_list[track_index].radar_id_confidence - self.config.radar_id_confidence_down,
            )

        for detection_index in unmatched_detections:
            self._create_track(detections[detection_index], frame.frame_count)

        self._drop_stale_tracks()
        return self.row_vector()

    def row_vector(self) -> List[float]:
        live_tracks = sorted(
            self.tracks.values(),
            key=lambda track: track.track_id,
        )

        row: List[float] = []
        visible_tracks = [
            track
            for track in live_tracks
            if self.config.output_lost_tracks or track.missed == 0
        ]
        radar_id_counts: Dict[int, int] = {}
        for track in visible_tracks:
            if track.radar_target_id is None:
                continue
            radar_id_counts[track.radar_target_id] = radar_id_counts.get(track.radar_target_id, 0) + 1

        for track in visible_tracks:
            x, y, z = track.center
            row.extend([float(output_id_for_track(track, self.config, radar_id_counts)), x, y, z])

        return row

    def _compute_dt(self, timestamp_ns: int) -> float:
        if self.last_timestamp_ns is None or timestamp_ns <= self.last_timestamp_ns:
            self.last_timestamp_ns = timestamp_ns
            return 1.0 / 30.0

        dt = (timestamp_ns - self.last_timestamp_ns) / 1_000_000_000.0
        self.last_timestamp_ns = timestamp_ns
        return max(1.0e-3, min(dt, 0.25))

    def _create_track(self, detection: Detection, frame_count: int) -> None:
        track_id = self.next_track_id
        self.next_track_id += 1

        x, y, z = detection.center
        self.tracks[track_id] = Track(
            track_id=track_id,
            radar_target_id=detection.radar_target_id,
            state=[x, y, z, 0.0, 0.0, 0.0],
            covariance=identity_matrix(6, 1.0),
            last_frame=frame_count,
            last_observed_center=detection.center,
            radar_id_confidence=1.0 if detection.radar_target_id is not None else 0.0,
        )

        if detection.doppler is not None:
            kalman_update_doppler(
                self.tracks[track_id],
                detection.center,
                detection.doppler * self.config.doppler_sign,
                self.config.doppler_noise,
            )
        update_exit_candidate(self.tracks[track_id], self.config)

    def _drop_stale_tracks(self) -> None:
        stale_ids = [
            track_id
            for track_id, track in self.tracks.items()
            if should_delete_track(track, self.config)
        ]

        for track_id in stale_ids:
            del self.tracks[track_id]


def open_shared_memory() -> mmap.mmap:
    while True:
        try:
            return mmap.mmap(
                -1,
                TOTAL_SIZE,
                tagname=SHM_NAME,
                access=mmap.ACCESS_READ,
            )
        except OSError as exc:
            print(f"[cluster] waiting for shared memory: {exc}", flush=True)
            time.sleep(1.0)


def read_frame(mm: mmap.mmap) -> Optional[RadarFrame]:
    while True:
        header1 = HEADER_STRUCT.unpack_from(mm, 0)

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
            RadarTarget(*TARGET_STRUCT.unpack_from(mm, targets_offset + i * TARGET_SIZE))
            for i in range(target_count)
        ]

        sequence2 = HEADER_STRUCT.unpack_from(mm, 0)[4]
        if sequence1 == sequence2 and not (sequence2 & 1):
            return RadarFrame(
                state=state,
                frame_count=frame_count,
                timestamp_ns=timestamp_ns,
                points=points,
                targets=targets,
            )


def build_detections(frame: RadarFrame, config: TrackerConfig) -> List[Detection]:
    if config.prefer_radar_targets and frame.targets:
        target_detections = detections_from_radar_targets(
            frame.targets,
            frame.points,
            config.power_threshold,
            config.center_method,
        )

        if target_detections:
            filtered_targets = filter_human_detections(target_detections, config)
            if filtered_targets:
                return filtered_targets

    dbscan_detections = detections_from_dbscan(
        frame.points,
        eps=config.dbscan_eps,
        min_samples=config.dbscan_min_samples,
        power_threshold=config.power_threshold,
        max_points=config.max_points,
        center_method=config.center_method,
    )
    return filter_human_detections(dbscan_detections, config)


def detections_from_radar_targets(
    targets: Sequence[RadarTarget],
    points: Sequence[Point],
    power_threshold: float,
    center_method: str,
) -> List[Detection]:
    points_by_target: Dict[int, List[Point]] = {}

    for point in points:
        target_id = point[5]
        power = point[4]
        if target_id < 0 or power < power_threshold:
            continue
        points_by_target.setdefault(target_id, []).append(point)

    detections: List[Detection] = []
    seen_target_ids = set()

    for target in targets:
        if target.target_id in seen_target_ids:
            continue
        seen_target_ids.add(target.target_id)

        target_points = points_by_target.get(target.target_id, [])
        fallback_center = target.box_center
        box_min_value = (target.minx, target.miny, target.minz)
        box_max_value = (target.maxx, target.maxy, target.maxz)

        detections.append(
            Detection(
                center=choose_center(
                    target_points,
                    center_method,
                    fallback_center,
                    box_min_value,
                    box_max_value,
                ),
                box_min=box_min_value,
                box_max=box_max_value,
                doppler=weighted_doppler(target_points),
                radar_target_id=target.target_id,
                point_count=len(target_points),
                source="radar_target",
            )
        )

    return detections


def detections_from_dbscan(
    points: Sequence[Point],
    eps: float,
    min_samples: int,
    power_threshold: float,
    max_points: int,
    center_method: str,
) -> List[Detection]:
    filtered = [
        point
        for point in points
        if point[4] >= power_threshold
    ]
    coords = [
        (point[0], point[1], point[2])
        for point in filtered
    ]

    if not coords:
        return []

    if len(filtered) > max_points:
        stride = math.ceil(len(filtered) / max_points)
        filtered = filtered[::stride]
        coords = coords[::stride]

    labels = dbscan(coords, eps, min_samples)
    clusters: Dict[int, List[Point]] = {}

    for point, label in zip(filtered, labels):
        if label < 0:
            continue
        clusters.setdefault(label, []).append(point)

    detections: List[Detection] = []
    for cluster_points in clusters.values():
        box_min_value, box_max_value = point_box(cluster_points)
        fallback_center = center_from_box(box_min_value, box_max_value)
        detections.append(
            Detection(
                center=choose_center(
                    cluster_points,
                    center_method,
                    fallback_center,
                    box_min_value,
                    box_max_value,
                ),
                box_min=box_min_value,
                box_max=box_max_value,
                doppler=weighted_doppler(cluster_points),
                radar_target_id=dominant_target_id(cluster_points),
                point_count=len(cluster_points),
                source="dbscan",
            )
        )

    return detections


def point_box(points: Sequence[Point]) -> Tuple[Vec3, Vec3]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    zs = [point[2] for point in points]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def center_from_box(box_min_value: Vec3, box_max_value: Vec3) -> Vec3:
    return (
        (box_min_value[0] + box_max_value[0]) * 0.5,
        (box_min_value[1] + box_max_value[1]) * 0.5,
        (box_min_value[2] + box_max_value[2]) * 0.5,
    )


def choose_center(
    points: Sequence[Point],
    center_method: str,
    fallback_center: Vec3,
    box_min_value: Vec3,
    box_max_value: Vec3,
) -> Vec3:
    if center_method == "box" or not points:
        return fallback_center

    if center_method == "mean":
        return mean_center(points)

    if center_method == "median":
        return median_center(points)

    if center_method == "weighted":
        return weighted_center(points)

    if center_method == "weighted_box_clamped":
        center = weighted_center(points)
        return (
            clamp(center[0], box_min_value[0], box_max_value[0]),
            clamp(center[1], box_min_value[1], box_max_value[1]),
            clamp(center[2], box_min_value[2], box_max_value[2]),
        )

    return fallback_center


def mean_center(points: Sequence[Point]) -> Vec3:
    count = float(len(points))
    return (
        sum(point[0] for point in points) / count,
        sum(point[1] for point in points) / count,
        sum(point[2] for point in points) / count,
    )


def median_center(points: Sequence[Point]) -> Vec3:
    return (
        median([point[0] for point in points]),
        median([point[1] for point in points]),
        median([point[2] for point in points]),
    )


def weighted_center(points: Sequence[Point]) -> Vec3:
    weighted_x = 0.0
    weighted_y = 0.0
    weighted_z = 0.0
    weight_sum = 0.0

    for point in points:
        weight = max(point[4], 1.0)
        weighted_x += point[0] * weight
        weighted_y += point[1] * weight
        weighted_z += point[2] * weight
        weight_sum += weight

    if weight_sum <= 0.0:
        return mean_center(points)

    return weighted_x / weight_sum, weighted_y / weight_sum, weighted_z / weight_sum


def median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) * 0.5


def filter_human_detections(
    detections: Sequence[Detection],
    config: TrackerConfig,
) -> List[Detection]:
    return [
        detection
        for detection in detections
        if detection_passes_human_filter(detection, config)
    ]


def detection_passes_human_filter(detection: Detection, config: TrackerConfig) -> bool:
    width, depth, height = detection.box_size

    if detection.point_count < config.min_detection_points:
        return False

    return (
        config.human_min_width <= width <= config.human_max_width
        and config.human_min_depth <= depth <= config.human_max_depth
        and config.human_min_height <= height <= config.human_max_height
    )


def weighted_doppler(points: Sequence[Point]) -> Optional[float]:
    if not points:
        return None

    weighted_sum = 0.0
    weight_sum = 0.0

    for point in points:
        doppler = point[3]
        power = max(point[4], 1.0)
        weighted_sum += doppler * power
        weight_sum += power

    if weight_sum <= 0.0:
        return None

    return weighted_sum / weight_sum


def dominant_target_id(points: Sequence[Point]) -> Optional[int]:
    counts: Dict[int, int] = {}

    for point in points:
        target_id = point[5]
        if target_id < 0:
            continue
        counts[target_id] = counts.get(target_id, 0) + 1

    if not counts:
        return None

    target_id, count = max(counts.items(), key=lambda item: item[1])
    if count < max(1, len(points) // 2):
        return None

    return target_id


def dbscan(points: Sequence[Vec3], eps: float, min_samples: int) -> List[int]:
    labels = [-99] * len(points)
    cluster_id = 0
    grid = build_grid(points, eps)

    for index in range(len(points)):
        if labels[index] != -99:
            continue

        neighbors = region_query(points, grid, index, eps)
        if len(neighbors) < min_samples:
            labels[index] = -1
            continue

        labels[index] = cluster_id
        seeds = list(neighbors)
        cursor = 0

        while cursor < len(seeds):
            neighbor_index = seeds[cursor]
            cursor += 1

            if labels[neighbor_index] == -1:
                labels[neighbor_index] = cluster_id

            if labels[neighbor_index] != -99:
                continue

            labels[neighbor_index] = cluster_id
            neighbor_neighbors = region_query(points, grid, neighbor_index, eps)

            if len(neighbor_neighbors) >= min_samples:
                seeds.extend(neighbor_neighbors)

        cluster_id += 1

    return labels


def build_grid(points: Sequence[Vec3], cell_size: float) -> Dict[Tuple[int, int, int], List[int]]:
    grid: Dict[Tuple[int, int, int], List[int]] = {}

    for index, point in enumerate(points):
        grid.setdefault(grid_key(point, cell_size), []).append(index)

    return grid


def grid_key(point: Vec3, cell_size: float) -> Tuple[int, int, int]:
    return (
        math.floor(point[0] / cell_size),
        math.floor(point[1] / cell_size),
        math.floor(point[2] / cell_size),
    )


def region_query(
    points: Sequence[Vec3],
    grid: Dict[Tuple[int, int, int], List[int]],
    index: int,
    eps: float,
) -> List[int]:
    cx, cy, cz = grid_key(points[index], eps)
    eps2 = eps * eps
    neighbors: List[int] = []

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                for candidate in grid.get((cx + dx, cy + dy, cz + dz), []):
                    if squared_distance(points[index], points[candidate]) <= eps2:
                        neighbors.append(candidate)

    return neighbors


def assign_tracks(
    tracks: Sequence[Track],
    detections: Sequence[Detection],
    max_distance: float,
    max_mahalanobis_distance: float,
    measurement_noise: float,
    radar_id_match_scale: float,
    radar_id_mismatch_penalty: float,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    if not tracks:
        return [], [], list(range(len(detections)))

    if not detections:
        return [], list(range(len(tracks))), []

    cost = [
        [
            assignment_cost(
                track,
                detection,
                measurement_noise,
                radar_id_match_scale,
                radar_id_mismatch_penalty,
            )
            for detection in detections
        ]
        for track in tracks
    ]
    assignments = hungarian(cost)

    accepted: List[Tuple[int, int]] = []
    assigned_tracks = set()
    assigned_detections = set()

    for track_index, detection_index in assignments:
        euclidean_distance = distance(tracks[track_index].center, detections[detection_index].center)
        mahalanobis = mahalanobis_distance(
            tracks[track_index],
            detections[detection_index].center,
            measurement_noise,
        )

        if euclidean_distance > max_distance or mahalanobis > max_mahalanobis_distance:
            continue

        accepted.append((track_index, detection_index))
        assigned_tracks.add(track_index)
        assigned_detections.add(detection_index)

    unmatched_tracks = [
        index for index in range(len(tracks)) if index not in assigned_tracks
    ]
    unmatched_detections = [
        index for index in range(len(detections)) if index not in assigned_detections
    ]

    return accepted, unmatched_tracks, unmatched_detections


def assignment_cost(
    track: Track,
    detection: Detection,
    measurement_noise: float,
    radar_id_match_scale: float,
    radar_id_mismatch_penalty: float,
) -> float:
    cost = mahalanobis_distance(track, detection.center, measurement_noise)

    if track.radar_target_id is None or detection.radar_target_id is None:
        return cost

    if track.radar_target_id == detection.radar_target_id:
        return cost * radar_id_match_scale

    return cost + radar_id_mismatch_penalty


def mahalanobis_distance(track: Track, measurement: Vec3, measurement_noise: float) -> float:
    residual = [
        measurement[0] - track.state[0],
        measurement[1] - track.state[1],
        measurement[2] - track.state[2],
    ]
    innovation = [
        [track.covariance[row][col] + (measurement_noise if row == col else 0.0)
         for col in range(3)]
        for row in range(3)
    ]
    innovation_inv = invert_3x3(innovation)
    weighted = mat_vec_mul(innovation_inv, residual)
    value = sum(residual[i] * weighted[i] for i in range(3))
    return math.sqrt(max(value, 0.0))


def update_track_radar_id(
    track: Track,
    detection: Detection,
    config: TrackerConfig,
) -> None:
    radar_target_id = detection.radar_target_id

    if radar_target_id is None:
        track.radar_id_confidence = max(
            0.0,
            track.radar_id_confidence - config.radar_id_confidence_down,
        )
        return

    if track.radar_target_id is None:
        track.radar_target_id = radar_target_id
        track.radar_id_confidence = min(1.0, config.radar_id_confidence_up)
        track.radar_id_candidate = None
        track.radar_id_candidate_hits = 0
        return

    if track.radar_target_id == radar_target_id:
        track.radar_id_confidence = min(
            1.0,
            track.radar_id_confidence + config.radar_id_confidence_up,
        )
        track.radar_id_candidate = None
        track.radar_id_candidate_hits = 0
        return

    track.radar_id_confidence = max(
        0.0,
        track.radar_id_confidence - config.radar_id_confidence_down,
    )

    if track.radar_id_candidate == radar_target_id:
        track.radar_id_candidate_hits += 1
    else:
        track.radar_id_candidate = radar_target_id
        track.radar_id_candidate_hits = 1

    if (
        track.radar_id_candidate_hits >= config.radar_id_switch_hits
        and track.radar_id_confidence <= 0.5
    ):
        track.radar_target_id = radar_target_id
        track.radar_id_confidence = min(1.0, config.radar_id_confidence_up)
        track.radar_id_candidate = None
        track.radar_id_candidate_hits = 0


def output_id_for_track(
    track: Track,
    config: TrackerConfig,
    radar_id_counts: Dict[int, int],
) -> int:
    if (
        config.output_radar_id
        and track.radar_target_id is not None
        and track.radar_id_confidence > 0.5
        and radar_id_counts.get(track.radar_target_id, 0) == 1
    ):
        return track.radar_target_id

    return track.track_id


def should_delete_track(track: Track, config: TrackerConfig) -> bool:
    if not config.exit_door_enabled:
        return False

    if track.missed < config.exit_missed_frames:
        return False

    return track.exit_candidate


def update_exit_candidate(track: Track, config: TrackerConfig) -> None:
    if not config.exit_door_enabled:
        track.exit_candidate = False
        track.last_exit_outward_speed = 0.0
        return

    outward_speed = exit_outward_speed(track, config)
    track.last_exit_outward_speed = outward_speed

    inside_exit_door = point_in_exit_door(track.last_observed_center, config)
    near_boundary = (
        not config.exit_door_require_boundary_proximity
        or near_exit_outward_boundary(track.last_observed_center, config)
    )
    moving_outward = (
        not config.exit_door_require_outward_motion
        or outward_speed >= config.exit_door_min_outward_speed
    )

    track.exit_candidate = inside_exit_door and near_boundary and moving_outward


def point_in_exit_door(point: Vec3, config: TrackerConfig) -> bool:
    return (
        config.exit_door_x_min <= point[0] <= config.exit_door_x_max
        and config.exit_door_y_min <= point[1] <= config.exit_door_y_max
        and config.exit_door_z_min <= point[2] <= config.exit_door_z_max
    )


def near_exit_outward_boundary(point: Vec3, config: TrackerConfig) -> bool:
    outward = (
        config.exit_door_outward_x,
        config.exit_door_outward_y,
        config.exit_door_outward_z,
    )
    axis = max(range(3), key=lambda index: abs(outward[index]))
    direction = outward[axis]

    if abs(direction) < 1.0e-6:
        return True

    if axis == 0 and direction < 0.0:
        return point[0] <= config.exit_door_x_min + config.exit_door_boundary_margin
    if axis == 0 and direction > 0.0:
        return point[0] >= config.exit_door_x_max - config.exit_door_boundary_margin
    if axis == 1 and direction < 0.0:
        return point[1] <= config.exit_door_y_min + config.exit_door_boundary_margin
    if axis == 1 and direction > 0.0:
        return point[1] >= config.exit_door_y_max - config.exit_door_boundary_margin
    if axis == 2 and direction < 0.0:
        return point[2] <= config.exit_door_z_min + config.exit_door_boundary_margin

    return point[2] >= config.exit_door_z_max - config.exit_door_boundary_margin


def exit_outward_speed(track: Track, config: TrackerConfig) -> float:
    outward = (
        config.exit_door_outward_x,
        config.exit_door_outward_y,
        config.exit_door_outward_z,
    )
    norm = math.sqrt(
        outward[0] * outward[0]
        + outward[1] * outward[1]
        + outward[2] * outward[2]
    )

    if norm < 1.0e-6:
        return 0.0

    return (
        track.state[3] * outward[0]
        + track.state[4] * outward[1]
        + track.state[5] * outward[2]
    ) / norm


def hungarian(cost: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    rows = len(cost)
    cols = len(cost[0]) if rows else 0

    if rows <= cols:
        return hungarian_rectangular(cost)

    transposed = [[cost[row][col] for row in range(rows)] for col in range(cols)]
    return [(col, row) for row, col in hungarian_rectangular(transposed)]


def hungarian_rectangular(cost: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    rows = len(cost)
    cols = len(cost[0]) if rows else 0
    u = [0.0] * (rows + 1)
    v = [0.0] * (cols + 1)
    p = [0] * (cols + 1)
    way = [0] * (cols + 1)

    for i in range(1, rows + 1):
        p[0] = i
        j0 = 0
        minv = [float("inf")] * (cols + 1)
        used = [False] * (cols + 1)

        while True:
            used[j0] = True
            i0 = p[j0]
            delta = float("inf")
            j1 = 0

            for j in range(1, cols + 1):
                if used[j]:
                    continue

                current = cost[i0 - 1][j - 1] - u[i0] - v[j]
                if current < minv[j]:
                    minv[j] = current
                    way[j] = j0

                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j

            for j in range(cols + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta

            j0 = j1
            if p[j0] == 0:
                break

        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break

    result: List[Tuple[int, int]] = []
    for j in range(1, cols + 1):
        if p[j] != 0:
            result.append((p[j] - 1, j - 1))

    return result


def kalman_predict(track: Track, dt: float, process_noise: float) -> None:
    transition = [
        [1.0, 0.0, 0.0, dt, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, dt, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, dt],
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]

    track.state = mat_vec_mul(transition, track.state)
    predicted_covariance = mat_mul(
        mat_mul(transition, track.covariance),
        transpose(transition),
    )

    q = process_noise * dt * dt
    for i in range(6):
        predicted_covariance[i][i] += q

    track.covariance = predicted_covariance


def kalman_update_position(track: Track, measurement: Vec3, measurement_noise: float) -> None:
    residual = [
        measurement[0] - track.state[0],
        measurement[1] - track.state[1],
        measurement[2] - track.state[2],
    ]

    s = [
        [track.covariance[row][col] + (measurement_noise if row == col else 0.0)
         for col in range(3)]
        for row in range(3)
    ]
    s_inv = invert_3x3(s)

    covariance_ht = [[track.covariance[row][col] for col in range(3)] for row in range(6)]
    kalman_gain = mat_mul(covariance_ht, s_inv)
    correction = mat_vec_mul(kalman_gain, residual)

    track.state = [
        track.state[i] + correction[i]
        for i in range(6)
    ]

    kh = [
        [kalman_gain[row][col] if col < 3 else 0.0 for col in range(6)]
        for row in range(6)
    ]
    identity = identity_matrix(6, 1.0)
    track.covariance = mat_mul(mat_sub(identity, kh), track.covariance)


def kalman_update_doppler(
    track: Track,
    center: Vec3,
    doppler: float,
    doppler_noise: float,
) -> None:
    norm = math.sqrt(center[0] * center[0] + center[1] * center[1] + center[2] * center[2])
    if norm < 1.0e-6:
        return

    radial_unit = (center[0] / norm, center[1] / norm, center[2] / norm)
    h = [0.0, 0.0, 0.0, radial_unit[0], radial_unit[1], radial_unit[2]]
    predicted_doppler = sum(h[i] * track.state[i] for i in range(6))
    residual = doppler - predicted_doppler

    pht = [
        sum(track.covariance[row][col] * h[col] for col in range(6))
        for row in range(6)
    ]
    innovation = sum(h[i] * pht[i] for i in range(6)) + doppler_noise
    if innovation <= 1.0e-9:
        return

    kalman_gain = [value / innovation for value in pht]

    for i in range(6):
        track.state[i] += kalman_gain[i] * residual

    kh = [
        [kalman_gain[row] * h[col] for col in range(6)]
        for row in range(6)
    ]
    identity = identity_matrix(6, 1.0)
    track.covariance = mat_mul(mat_sub(identity, kh), track.covariance)


def identity_matrix(size: int, value: float) -> List[List[float]]:
    return [
        [value if row == col else 0.0 for col in range(size)]
        for row in range(size)
    ]


def mat_vec_mul(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> List[float]:
    return [
        sum(row[col] * vector[col] for col in range(len(vector)))
        for row in matrix
    ]


def mat_mul(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> List[List[float]]:
    right_t = transpose(right)
    return [
        [
            sum(left_row[k] * right_col[k] for k in range(len(right_col)))
            for right_col in right_t
        ]
        for left_row in left
    ]


def mat_sub(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> List[List[float]]:
    return [
        [left[row][col] - right[row][col] for col in range(len(left[row]))]
        for row in range(len(left))
    ]


def transpose(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    return [list(col) for col in zip(*matrix)]


def invert_3x3(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]

    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )

    if abs(det) < 1.0e-9:
        det = 1.0e-9 if det >= 0.0 else -1.0e-9

    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def distance(left: Vec3, right: Vec3) -> float:
    return math.sqrt(squared_distance(left, right))


def squared_distance(left: Vec3, right: Vec3) -> float:
    dx = left[0] - right[0]
    dy = left[1] - right[1]
    dz = left[2] - right[2]
    return dx * dx + dy * dy + dz * dz


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def format_row(row: Iterable[float]) -> str:
    values = list(row)
    chunks = []

    for i in range(0, len(values), 4):
        track_id = int(values[i])
        x = values[i + 1]
        y = values[i + 2]
        z = values[i + 3]
        chunks.extend([str(track_id), f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"])

    return "(" + ", ".join(chunks) + ")"


def tracker_config_defaults() -> Dict[str, Any]:
    return {
        field.name: getattr(TrackerConfig(), field.name)
        for field in fields(TrackerConfig)
    }


def load_config_defaults(config_path: Optional[str]) -> Dict[str, Any]:
    defaults = tracker_config_defaults()

    if not config_path:
        return defaults

    with open(config_path, "r", encoding="utf-8") as config_file:
        raw_config = json.load(config_file)

    apply_config_mapping(defaults, raw_config)
    return defaults


def apply_config_mapping(defaults: Dict[str, Any], raw_config: Dict[str, Any]) -> None:
    direct_keys = set(defaults.keys())

    for key, value in raw_config.items():
        if key in direct_keys:
            defaults[key] = value

    sections = {
        "dbscan": {
            "eps": "dbscan_eps",
            "min_samples": "dbscan_min_samples",
            "max_points": "max_points",
        },
        "tracking": {
            "max_assignment_distance": "max_assignment_distance",
            "max_mahalanobis_distance": "max_mahalanobis_distance",
            "exit_missed_frames": "exit_missed_frames",
            "process_noise": "process_noise",
            "measurement_noise": "measurement_noise",
            "doppler_noise": "doppler_noise",
            "doppler_sign": "doppler_sign",
        },
        "radar_id": {
            "prefer_targets": "prefer_radar_targets",
            "match_scale": "radar_id_match_scale",
            "mismatch_penalty": "radar_id_mismatch_penalty",
            "switch_hits": "radar_id_switch_hits",
            "confidence_up": "radar_id_confidence_up",
            "confidence_down": "radar_id_confidence_down",
            "output_radar_id": "output_radar_id",
            "output_lost_tracks": "output_lost_tracks",
        },
        "human_filter": {
            "min_detection_points": "min_detection_points",
            "min_width": "human_min_width",
            "max_width": "human_max_width",
            "min_depth": "human_min_depth",
            "max_depth": "human_max_depth",
            "min_height": "human_min_height",
            "max_height": "human_max_height",
        },
        "exit_door": {
            "enabled": "exit_door_enabled",
            "x_min": "exit_door_x_min",
            "x_max": "exit_door_x_max",
            "y_min": "exit_door_y_min",
            "y_max": "exit_door_y_max",
            "z_min": "exit_door_z_min",
            "z_max": "exit_door_z_max",
            "outward_x": "exit_door_outward_x",
            "outward_y": "exit_door_outward_y",
            "outward_z": "exit_door_outward_z",
            "boundary_margin": "exit_door_boundary_margin",
            "min_outward_speed": "exit_door_min_outward_speed",
            "require_outward_motion": "exit_door_require_outward_motion",
            "require_boundary_proximity": "exit_door_require_boundary_proximity",
        },
    }

    for section_name, section_mapping in sections.items():
        section = raw_config.get(section_name)
        if not isinstance(section, dict):
            continue

        for key, config_field in section_mapping.items():
            if key in section:
                defaults[config_field] = section[key]


def make_tracker_config(args: argparse.Namespace) -> TrackerConfig:
    values = {
        field.name: getattr(args, field.name)
        for field in fields(TrackerConfig)
    }
    return TrackerConfig(**values)


def parse_args() -> argparse.Namespace:
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--config")
    config_args, remaining_args = config_parser.parse_known_args()
    defaults = load_config_defaults(config_args.config)

    parser = argparse.ArgumentParser(
        description="Cluster radar shared-memory points and print a 1 x 4N row vector.",
        parents=[config_parser],
    )
    parser.set_defaults(**defaults)

    parser.add_argument("--eps", dest="dbscan_eps", type=float)
    parser.add_argument("--min-samples", dest="dbscan_min_samples", type=int)
    parser.add_argument("--max-assignment-distance", type=float)
    parser.add_argument("--max-mahalanobis-distance", type=float)
    parser.add_argument("--exit-missed-frames", type=int)
    parser.add_argument("--power-threshold", type=float)
    parser.add_argument("--max-points", type=int)
    parser.add_argument(
        "--center-method",
        choices=("box", "mean", "weighted", "weighted_box_clamped", "median"),
    )
    parser.add_argument("--min-detection-points", type=int)
    parser.add_argument("--human-min-width", type=float)
    parser.add_argument("--human-max-width", type=float)
    parser.add_argument("--human-min-depth", type=float)
    parser.add_argument("--human-max-depth", type=float)
    parser.add_argument("--human-min-height", type=float)
    parser.add_argument("--human-max-height", type=float)
    parser.add_argument("--doppler-noise", type=float)
    parser.add_argument("--doppler-sign", type=float)
    parser.add_argument("--ignore-radar-targets", dest="prefer_radar_targets", action="store_false")
    parser.add_argument("--use-radar-targets", dest="prefer_radar_targets", action="store_true")
    parser.add_argument("--radar-id-match-scale", type=float)
    parser.add_argument("--radar-id-mismatch-penalty", type=float)
    parser.add_argument("--radar-id-switch-hits", type=int)
    parser.add_argument("--output-radar-id", action="store_true")
    parser.add_argument("--no-output-radar-id", dest="output_radar_id", action="store_false")
    parser.add_argument("--output-lost-tracks", dest="output_lost_tracks", action="store_true")
    parser.add_argument("--hide-lost-tracks", dest="output_lost_tracks", action="store_false")
    parser.add_argument("--exit-door-enabled", dest="exit_door_enabled", action="store_true")
    parser.add_argument("--exit-door-disabled", dest="exit_door_enabled", action="store_false")
    parser.add_argument("--exit-door-x-min", type=float)
    parser.add_argument("--exit-door-x-max", type=float)
    parser.add_argument("--exit-door-y-min", type=float)
    parser.add_argument("--exit-door-y-max", type=float)
    parser.add_argument("--exit-door-z-min", type=float)
    parser.add_argument("--exit-door-z-max", type=float)
    parser.add_argument("--exit-door-outward-x", type=float)
    parser.add_argument("--exit-door-outward-y", type=float)
    parser.add_argument("--exit-door-outward-z", type=float)
    parser.add_argument("--exit-door-boundary-margin", type=float)
    parser.add_argument("--exit-door-min-outward-speed", type=float)
    parser.add_argument("--exit-door-require-outward-motion", dest="exit_door_require_outward_motion", action="store_true")
    parser.add_argument("--exit-door-no-outward-motion", dest="exit_door_require_outward_motion", action="store_false")
    parser.add_argument("--exit-door-require-boundary-proximity", dest="exit_door_require_boundary_proximity", action="store_true")
    parser.add_argument("--exit-door-no-boundary-proximity", dest="exit_door_require_boundary_proximity", action="store_false")
    parser.add_argument("--once", action="store_true", default=False)
    return parser.parse_args(remaining_args, namespace=config_args)


def main() -> None:
    args = parse_args()
    tracker = ClusterTracker(make_tracker_config(args))

    print(f"[cluster] opening shared memory: {SHM_NAME}", flush=True)
    mm = open_shared_memory()
    last_frame_count: Optional[int] = None

    try:
        while True:
            frame = read_frame(mm)

            if frame is None:
                print("[cluster] shared memory exists, but producer is not initialized", flush=True)
                time.sleep(1.0)
                continue

            if frame.frame_count == last_frame_count:
                time.sleep(0.005)
                continue

            last_frame_count = frame.frame_count
            row = tracker.update(frame)
            print(format_row(row), flush=True)

            if args.once:
                break

    except KeyboardInterrupt:
        print("\n[cluster] stopped", flush=True)
    finally:
        mm.close()


if __name__ == "__main__":
    main()
