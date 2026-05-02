_아래 README는 클로드로 작성된 정보입니다. 읽어보고 수정하셔도 됩니다._
# Radar Viewer — 데이터 전송 기능 추가 작업 가이드

## 작업 배경

현재 뷰어는 레이다 장치로부터 패킷을 수신해 3D 점군(Point Cloud)과 트래킹 대상(Target)을 실시간으로 시각화합니다.  
Unity 팀에 포인트/타겟 데이터를 API 명세에 맞춰 전달하고, 일정 시간 동안 정지한 트래킹 대상을 BE 서버로 보고하는 기능이 필요합니다.

---

## 전체 요구사항 요약

| # | 기능 | 관련 파일 |
|---|------|-----------|
| 1 | 노이즈 임계값(threshold) 필터링 후 Unity API 명세에 맞는 JSON/CSV 직렬화 | `main.cpp`, 신규 `export.h/.cpp` |
| 2 | 직렬화된 데이터를 로컬 BE 서버로 HTTP POST | 신규 `http_sender.h/.cpp` |
| 3 | Track ID별 정지 감지 → 확정 후 BE 서버로 별도 POST | `main.cpp`, 신규 `dwell_tracker.h/.cpp` |

---

## 1. 노이즈 필터링 및 데이터 직렬화

### 1-1. 필터링 대상 필드

`retina::Point` 구조체(`retina.h`)의 필드 중 아래 두 값으로 노이즈를 걸러냅니다.

```cpp
struct Point {
    float x, y, z;
    float doppler;   // 도플러 속도
    float power;     // 수신 전력 (노이즈 판별 핵심)
    int32_t targetId;
};
```

**power 값이 임계값 미만인 포인트는 전송 대상에서 제외합니다.**  
임계값은 런타임에 UI에서 조정할 수 있어야 합니다(기존 `m_power_min` 활용 또는 별도 슬라이더 추가).

### 1-2. 직렬화 포맷

Unity 팀 API 명세에 따라 **JSON** 또는 **CSV** 중 하나로 직렬화합니다.  
아래는 현재 데이터 구조 기준 예시입니다 — **Unity 팀 제공 명세가 우선**합니다.

**JSON 예시 (권장)**
```json
{
  "frameCount": 1024,
  "timestamp": 1713180000.123,
  "points": [
    { "x": 1.2, "y": 3.4, "z": 0.1, "doppler": 0.5, "power": 3200.0, "targetId": 2 }
  ],
  "targets": [
    {
      "targetId": 2,
      "x": 1.1, "y": 3.3,
      "status": "Walking",
      "minx": 0.9, "maxx": 1.3,
      "miny": 3.1, "maxy": 3.5,
      "minz": -0.2, "maxz": 0.4
    }
  ]
}
```

**CSV 예시 (포인트만 전송할 경우)**
```
frameCount,x,y,z,doppler,power,targetId
1024,1.2,3.4,0.1,0.5,3200.0,2
```

### 1-3. 구현 위치

신규 파일 `export.h` / `export.cpp`를 추가하고 아래 인터페이스를 구현하세요.

```cpp
// export.h
#pragma once
#include "retina.h"
#include <string>

namespace export_util
{
    struct ExportConfig
    {
        float power_threshold = 0.0f;   // 이 값 미만 포인트 제외
        bool use_json = true;           // false면 CSV
    };

    // 필터링된 포인트/타겟을 직렬화해 문자열 반환
    std::string serialize(const retina::Frame& frame, const ExportConfig& config);
}
```

---

## 2. 로컬 BE 서버로 HTTP POST

### 2-1. 전송 시점

매 프레임(`readFrames` 콜백 내부) 또는 설정 가능한 주기(예: 100ms)마다 직렬화된 데이터를 POST합니다.  
전송 주기를 너무 짧게 설정하면 BE 서버 부하가 증가하므로, UI에 **전송 주기 슬라이더**(ms 단위)를 제공하면 좋습니다.

### 2-2. 구현 위치

신규 파일 `http_sender.h` / `http_sender.cpp`를 추가하세요.  
이미 프로젝트에 **ASIO**가 포함되어 있으므로(`retina.h`의 `#include <asio.hpp>`) 같은 라이브러리로 구현합니다.

```cpp
// http_sender.h
#pragma once
#include <asio.hpp>
#include <string>

class HttpSender
{
public:
    // base_url 예: "http://127.0.0.1:5000"
    HttpSender(asio::io_context& io, const std::string& base_url);

    // 비동기 POST, 응답은 무시해도 무방
    void postAsync(const std::string& path, const std::string& body, const std::string& content_type);
};
```

**엔드포인트 예시 (BE 팀과 확정 필요)**

| 경로 | 본문 | 설명 |
|------|------|------|
| `POST /frames` | JSON/CSV | 매 프레임 포인트+타겟 데이터 |
| `POST /dwell` | JSON | 정지 확정 이벤트 |

### 2-3. `main.cpp` 수정 포인트

`MyApp` 클래스에 멤버를 추가하고 `renderUI` → `readFrames` 콜백 내에서 호출합니다.

```cpp
// MyApp 멤버 추가
std::unique_ptr<HttpSender> m_sender;
export_util::ExportConfig m_export_config;
std::chrono::steady_clock::time_point m_last_send_time {};
float m_send_interval_ms = 100.0f;  // UI 슬라이더로 조정

// beginAsyncIO()에서 m_client 생성 직후
m_sender = std::make_unique<HttpSender>(m_io, "http://127.0.0.1:5000");
```

```cpp
// readFrames 콜백 내부 (run() 루프)
const auto now = std::chrono::steady_clock::now();
const float elapsed_ms = std::chrono::duration<float, std::milli>(now - m_last_send_time).count();

if (elapsed_ms >= m_send_interval_ms)
{
    m_export_config.power_threshold = m_power_min;  // UI 값 연동
    const std::string body = export_util::serialize(frame, m_export_config);
    m_sender->postAsync("/frames", body, "application/json");
    m_last_send_time = now;
}
```

---

## 3. 정지 감지(Dwell Detection) 및 확정 전송

### 3-1. 동작 정의

- 트래킹된 Target(`retina::Target`)의 `(x, y)` 위치가 **감지 범위 [n, m]** 내에 있으면서
- **s초 동안 일정 거리 이내에서 머문 경우** → "정지 확정" 이벤트 발생
- 확정 이벤트는 `POST /dwell`로 한 번만 전송하고, 이후 해당 Target이 다시 움직이면 카운터를 리셋합니다

> **n, m, s 값과 "정지 판정 거리(movement_threshold)"는 모두 UI에서 조정 가능하게** 구현하세요.

### 3-2. 구현 위치

신규 파일 `dwell_tracker.h` / `dwell_tracker.cpp`를 추가하세요.

```cpp
// dwell_tracker.h
#pragma once
#include "retina.h"
#include <functional>
#include <unordered_map>
#include <chrono>

struct DwellConfig
{
    float range_min = 0.0f;         // 감지 범위 시작 (Y축 기준)
    float range_max = 9.5f;         // 감지 범위 끝   (Y축 기준)
    float dwell_seconds = 3.0f;     // 정지 판정 시간 (s)
    float movement_threshold = 0.1f; // 이 거리 이내면 "정지"로 간주 (m)
};

struct DwellEvent
{
    uint32_t targetId;
    float x, y;
    double dwell_seconds;
};

class DwellTracker
{
public:
    using Callback = std::function<void(const DwellEvent&)>;

    DwellTracker(Callback on_confirmed);

    // 매 프레임 호출. 확정 이벤트 발생 시 콜백 호출
    void update(const std::vector<retina::Target>& targets, const DwellConfig& config);

private:
    struct TargetState
    {
        float last_x, last_y;
        std::chrono::steady_clock::time_point still_since;
        bool confirmed = false;
    };

    std::unordered_map<uint32_t, TargetState> m_states;
    Callback m_callback;
};
```

### 3-3. `main.cpp` 수정 포인트

```cpp
// MyApp 멤버 추가
std::unique_ptr<DwellTracker> m_dwell_tracker;
DwellConfig m_dwell_config;

// MyApp 생성자 또는 initWindow()에서 초기화
m_dwell_tracker = std::make_unique<DwellTracker>([this](const DwellEvent& ev)
{
    // 정지 확정 이벤트 → BE 전송
    const std::string body = /* ev를 JSON으로 직렬화 */;
    m_sender->postAsync("/dwell", body, "application/json");
    m_log << "[dwell] target " << ev.targetId << " confirmed stationary at ("
          << ev.x << ", " << ev.y << ")\n";
});
```

```cpp
// readFrames 콜백 내부 (매 프레임 갱신)
m_dwell_tracker->update(frame.targets, m_dwell_config);
```

### 3-4. UI에 추가할 항목 (`drawSidebar` 내부)

```cpp
ImGui::Separator();
ImGui::TextUnformatted("Dwell Detection");
ImGui::DragFloat("Range Min (Y)", &m_dwell_config.range_min, 0.1f, 0.0f, 10.0f, "%.1f m");
ImGui::DragFloat("Range Max (Y)", &m_dwell_config.range_max, 0.1f, 0.0f, 10.0f, "%.1f m");
ImGui::DragFloat("Dwell Time",    &m_dwell_config.dwell_seconds, 0.5f, 1.0f, 60.0f, "%.1f s");
ImGui::DragFloat("Move Thresh",   &m_dwell_config.movement_threshold, 0.01f, 0.01f, 1.0f, "%.2f m");

ImGui::Separator();
ImGui::TextUnformatted("Export");
ImGui::DragFloat("Send Interval", &m_send_interval_ms, 10.0f, 16.0f, 2000.0f, "%.0f ms");
ImGui::Checkbox("Use JSON", &m_export_config.use_json);
```

---

## 파일 구조 (변경 후)

```
project/
├── main.cpp              ← MyApp에 멤버/호출 추가
├── retina.h / retina.cpp ← 수정 없음
├── network.h / network.cpp ← 수정 없음
├── debug_console.h / .cpp  ← 수정 없음
├── implot3d_custom.h / .cpp ← 수정 없음
│
├── export.h              ← 신규: 직렬화
├── export.cpp
├── http_sender.h         ← 신규: HTTP POST
├── http_sender.cpp
├── dwell_tracker.h       ← 신규: 정지 감지
└── dwell_tracker.cpp
```

---

## 체크리스트

- [ ] Unity 팀 API 명세 수령 후 `export.cpp` 직렬화 포맷 확정
- [ ] BE 팀에 엔드포인트 URL 및 포트 확인 (`/frames`, `/dwell`)
- [ ] `power_threshold` 기본값 BE/Unity 팀과 협의
- [ ] 정지 판정 거리(`movement_threshold`) 및 `dwell_seconds` 기본값 현장 테스트로 보정
- [ ] `HttpSender` 전송 실패 시 재시도 또는 로그 출력 방식 결정
- [ ] 빌드 시 ASIO standalone 설정 유지 확인 (`#define ASIO_STANDALONE` — `retina.h` 참고)
