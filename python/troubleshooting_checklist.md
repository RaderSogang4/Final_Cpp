# Radar Tracking Troubleshooting Checklist

실제 버스 또는 실험 환경에서 레이더 기반 사람 추적을 확인할 때 사용할 문제 상황별 점검표입니다.

## 1. 기본 연결 문제

### 문제 상황
- C++ viewer에 point cloud가 보이지 않는다.
- `receive_radar_shared_memory.py`에서 `frame_count`가 증가하지 않는다.
- `points=0` 또는 `targets=0`만 반복된다.
- `cluster_radar_shared_memory.py` 출력이 `()`만 반복된다.

### 확인 방법
- C++ viewer에서 수신 상태가 `Receiving`인지 확인한다.
- `receive_radar_shared_memory.py`를 실행해 `frame`, `points`, `targets`가 갱신되는지 확인한다.
- `first=(x, y, z), doppler, power, target` 값이 시간에 따라 바뀌는지 확인한다.
- `targets=0`이면 radar firmware target을 못 받는 상황일 수 있으므로 DBSCAN fallback 동작을 확인한다.
- `points`는 있는데 출력이 `()`이면 `power_threshold`, 사람 크기 필터, ROI 필터가 너무 강한지 확인한다.

## 2. 좌표계 불일치

### 문제 상황
- 왼쪽 문 ROI를 설정했는데 실제 왼쪽 문에서 출구 판정이 되지 않는다.
- 사람이 뒤로 이동하는데 `y`가 줄어든다.
- 출구 바깥 방향으로 움직였는데 outward motion 조건을 만족하지 않는다.

### 확인 방법
- 한 사람이 레이더 바로 앞, 통로 중앙, 왼쪽 문, 오른쪽 벽, 뒤쪽 위치에 차례로 서서 `(x, y, z)`를 기록한다.
- 레이더 기준 왼쪽이 `x<0`인지 `x>0`인지 확인한다.
- 버스 뒤쪽으로 갈수록 `y`가 커지는지 확인한다.
- 왼쪽 출구 바깥 방향이 `outward_x=-1`인지 `outward_x=1`인지 확인한다.
- 실제 문 영역이 `exit_door.x_min/x_max/y_min/y_max/z_min/z_max` 안에 들어오는지 확인한다.

## 3. 빈 공간 False Positive

### 문제 상황
- 사람이 없는데 cluster가 출력된다.
- 항상 같은 위치에서 cluster가 생긴다.
- 문 열림/닫힘에 따라 고정 cluster가 생긴다.

### 확인 방법
- 빈 버스에서 문 닫힘, 문 열림 상태를 각각 기록한다.
- false cluster 위치가 고정 구조물, 손잡이, 좌석, 문, 금속 프레임과 일치하는지 확인한다.
- 해당 cluster의 point count, power, box size를 확인한다.
- `power_threshold`를 올렸을 때 사라지는지 확인한다.
- 사람이 있을 수 없는 영역이면 ROI 마스킹 후보로 기록한다.

## 4. 사람 1명 정지

### 문제 상황
- 사람 1명이 있는데 출력 cluster가 없다.
- 사람 1명이 여러 cluster로 쪼개진다.
- ID가 계속 바뀐다.
- 위치가 크게 흔들린다.

### 확인 방법
- 사람 1명을 통로 중앙, 좌석 근처, 출입문 근처에 세워 각각 출력 cluster 개수를 확인한다.
- `receive_radar_shared_memory.py`에서 `targets`가 1개로 나오는지 확인한다.
- radar target 기반 detection을 쓰는지, DBSCAN fallback으로 가는지 확인한다.
- 출력이 없으면 사람 크기 필터의 `human_max_width/depth/height`, `min_detection_points`, `power_threshold`를 확인한다.
- 여러 cluster로 쪼개지면 `dbscan_eps`, `dbscan_min_samples`를 확인한다.

## 5. 사람 1명 이동

### 문제 상황
- 걷는 동안 ID가 바뀐다.
- 위치가 이동 방향과 반대로 움직인다.
- Doppler 적용 후 tracking이 더 불안정해진다.
- 출구 ROI 안에 들어갔다는 이유만으로 삭제된다.

### 확인 방법
- 레이더에서 멀어지는 방향, 레이더 쪽으로 다가오는 방향, 좌우 방향 이동을 각각 기록한다.
- 이동 방향과 `(x, y, z)` 변화가 직관적으로 맞는지 확인한다.
- Doppler 부호가 맞는지 `doppler_sign=1`과 `doppler_sign=-1`을 비교한다.
- `doppler_noise` 값을 올렸을 때 속도 보정 영향이 줄어드는지 확인한다.
- 출구 ROI 안에 들어가도 `missed` 상태가 아니면 삭제되지 않는지 확인한다.

## 6. 사람 2명 분리

### 문제 상황
- 사람 2명이 1개 cluster로 합쳐진다.
- 사람 1명이 2개 이상 cluster로 쪼개진다.
- 두 사람의 ID가 간헐적으로 바뀐다.

### 확인 방법
- 사람 2명을 충분히 떨어뜨려 정지시킨 뒤 cluster 개수가 2개인지 확인한다.
- 두 사람이 나란히 이동할 때 각 ID가 유지되는지 확인한다.
- radar target ID가 각 사람마다 안정적으로 부여되는지 확인한다.
- 합쳐지면 `dbscan_eps`를 줄이거나 radar target 우선 사용 여부를 확인한다.
- 쪼개지면 `dbscan_eps` 증가, `dbscan_min_samples` 감소를 검토한다.

## 7. 사람 2명 교차

### 문제 상황
- 두 사람이 교차한 뒤 ID가 서로 바뀐다.
- 가까워지는 순간 하나의 cluster로 합쳐졌다가 다시 분리된다.
- radar target ID가 순간적으로 바뀌거나 재할당된다.

### 확인 방법
- 통로에서 두 사람이 서로 반대 방향으로 지나가게 한다.
- 교차 전후의 output ID가 같은 사람에게 유지되는지 확인한다.
- `radar_target_id`, `radar_id_confidence`, `radar_id_candidate_hits`를 로그로 확인한다.
- ID swap이 있으면 `radar_id_mismatch_penalty`, `radar_id_switch_hits`, `max_mahalanobis_distance`를 조정한다.

## 8. 오클루전

### 문제 상황
- 사람이 잠깐 가려진 뒤 새 ID로 다시 생긴다.
- 출구가 아닌 곳에서 사라졌는데 track이 삭제된다.
- 오클루전 후 예측 위치가 너무 멀리 튄다.

### 확인 방법
- 한 사람이 다른 사람 뒤에 가려졌다가 다시 나오게 한다.
- 좌석, 기둥, 손잡이 근처에서 잠깐 사라지는 상황을 만든다.
- 출구 ROI 밖에서 `missed`가 증가해도 삭제되지 않는지 확인한다.
- 다시 관측됐을 때 기존 ID로 매칭되는지 확인한다.
- 새 ID가 생기면 `max_assignment_distance`, `max_mahalanobis_distance`, `process_noise`를 확인한다.

## 9. 하차

### 문제 상황
- 실제 하차했는데 track이 유지된다.
- 문 앞에 서 있기만 했는데 track이 삭제된다.
- 출구 ROI에 들어갔지만 outward motion 조건을 만족하지 않는다.

### 확인 방법
- 사람이 왼쪽 출구 ROI로 이동한 뒤 바깥 방향으로 사라지게 한다.
- 마지막 관측 위치가 `exit_door` ROI 내부인지 확인한다.
- 마지막 관측 위치가 outward 경계 근처인지 확인한다.
- `outward_speed >= min_outward_speed`인지 확인한다.
- `missed >= exit_missed_frames` 이후에만 삭제되는지 확인한다.
- 하차했는데 유지되면 ROI 위치, `outward_x/y/z`, `boundary_margin`, `min_outward_speed`를 확인한다.
- 문 앞 정지 중 삭제되면 `min_outward_speed`를 올리거나 `boundary_margin`과 ROI를 줄인다.

## 10. 승차

### 문제 상황
- 승차하는 사람이 새 track으로 생성되지 않는다.
- 승차 중인데 하차로 잘못 판단되어 삭제된다.
- 문 밖 또는 문 근처에서 위치가 크게 튄다.

### 확인 방법
- 출입문 근처에서 사람이 버스 내부 방향으로 들어오게 한다.
- 내부 방향 이동이 outward motion과 반대 방향인지 확인한다.
- 새 ID가 생성되고 계속 유지되는지 확인한다.
- `min_detection_points`, 사람 크기 필터, `power_threshold` 때문에 초기 detection이 사라지는지 확인한다.

## 11. 문 앞 혼잡

### 문제 상황
- 문 앞에 여러 사람이 몰리면 cluster가 합쳐진다.
- 한 명만 하차했는데 남은 사람 ID가 삭제된다.
- 승차/하차가 동시에 일어날 때 ID swap이 생긴다.

### 확인 방법
- 2~4명이 문 앞에 모이는 상황을 만든다.
- 한 명은 하차하고 다른 한 명은 문 앞에 남는 상황을 기록한다.
- 삭제된 track의 마지막 위치, outward speed, boundary proximity를 확인한다.
- 남아 있는 사람의 ID가 유지되는지 확인한다.
- 오삭제가 있으면 exit ROI를 줄이고, `boundary_margin`을 줄이고, `min_outward_speed`를 올린다.

## 12. 로그로 반드시 확인할 값

### 문제 상황
- 왜 ID가 바뀌었는지 설명할 수 없다.
- 왜 track이 삭제됐는지 설명할 수 없다.
- 왜 cluster가 생성되지 않았는지 설명할 수 없다.

### 확인 방법
- 매 프레임 아래 값을 기록한다.

```text
frame_count
points count
targets count
raw radar targetId list
detection count
filtered detection count
output row
track_id
radar_target_id
radar_id_confidence
center x,y,z
velocity vx,vy,vz
missed count
inside_exit_roi
near_exit_boundary
outward_speed
delete decision
```

## 13. 권장 실험 순서

### 문제 상황
- 복잡한 승하차 상황에서 원인을 분리하기 어렵다.

### 확인 방법
- 아래 순서대로 단순한 상황부터 검증한다.

```text
1. 빈 공간
2. 사람 1명 정지
3. 사람 1명 이동
4. 사람 1명 출구 접근 후 정지
5. 사람 1명 하차
6. 사람 2명 분리
7. 사람 2명 교차
8. 사람 2명 오클루전
9. 문 앞 혼잡
10. 승하차 동시 상황
```
