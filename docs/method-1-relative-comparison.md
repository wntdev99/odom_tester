# 방법 ① — swerve vs EKF 오도메트리 드리프트 비교 (`odom_compare`)

> 로봇을 **자동으로** 정해진 경로로 주행시키면서, `swerve_controller/odom`과 `fused_odom`
> (EKF)이 **`odom` 프레임에서 서로 얼마나 벌어지는지(드리프트)** 만 본다.
> 가장 단순한 버전이다 — 외부 장비·ground truth·emcl·teleop 없음.

---

## 0. 범위와 성격 — 먼저 못 박고 간다

- **비교 대상은 둘뿐**: `/swerve_controller/odom`(원본) vs `/fused_odom`(EKF 융합).
- **진실(ground truth)은 없다.** 따라서 이 방법은 **절대 정확도(accuracy)를 재지 않는다.**
  두 추정기가 **서로 얼마나 어긋나는지(mutual divergence)** 만 본다.
- `fused_odom`은 `swerve_odom`에 IMU를 더해 만든 값이라, 둘의 차이는 사실상 **"IMU 융합이
  바퀴 추정치를 얼마나 바꿨는가"** 를 분리해서 보여준다. EKF는 보통 위치(x,y)는 바퀴를,
  heading(yaw)은 IMU를 더 신뢰하므로 **이 비교의 주인공은 heading 차이**다.
- **규율**: 이 방법이 내는 숫자를 절대적 의미의 "오차(error)"라 부르지 않는다. **"드리프트
  (drift)", "불일치(divergence)", "융합 기여(fusion contribution)"** 로 부른다. 절대 정확도는
  방법 ②(UMBmark)·③(AprilTag GT)가 담당한다.

---

## 1. 확정된 인터페이스 (실제 로봇에서 실측)

| 대상 | 토픽 | 타입 | 프레임 |
|---|---|---|---|
| swerve 원본 오도메트리 | `/swerve_controller/odom` | `nav_msgs/Odometry` | `odom → base_footprint` |
| EKF 융합 오도메트리 | `/fused_odom` | `nav_msgs/Odometry` | `odom → base_footprint` |
| 명령 | `/swerve_controller/cmd_vel` | `geometry_msgs/Twist` (unstamped) | `vx=linear.x`, `vy=linear.y`, `wz=angular.z` |

- **두 오도메트리가 같은 `odom` 프레임**을 쓴다는 게 이 방법을 단순하게 만드는 핵심이다.
  프레임 정렬·좌표 변환이 필요 없다.
- 로봇 기준 프레임은 `base_footprint`.
- **cmd_vel은 "속도" 명령이다** — `Twist`에는 거리 필드가 없다. "몇 m 가라"는 직접 못 시키고,
  속도를 주는 동안 움직이다가 0을 주면 멈춘다. 또한 컨트롤러의 cmd_vel timeout 때문에 발행을
  멈추면 안전상 스스로 정지하므로, **주행 중에는 일정 주기(20~50 Hz)로 계속 발행**해야 한다.
  (이 로봇의 timeout 실제값은 구현 전 확인.)

---

## 2. 준비 (Preparation)

- 경로를 밟을 **평탄한 빈 공간**(1 m × 1 m + 여유).
- 두 오도메트리 토픽이 발행 중일 것(확인 완료).
- `/swerve_controller/cmd_vel`에 **다른 명령 소스가 경합하지 않도록** 실행 환경 정리
  (테스트 중 정지 0 발행 등 다른 발행자 비활성).
- 외부 장비·바닥 표시·맵·초기위치 설정 **불필요**.

---

## 3. 실험 조건 — 회전량을 단계적으로 바꾼다

세 조건이 하나의 변수(**총 회전량**)만 바꾸도록 설계한다. 그래야 드리프트 중 **회전에서 오는
부분과 병진에서 오는 부분을 분리**할 수 있다.

| 조건(preset) | 경로 | 5바퀴 후 누적 회전 | 무엇을 드러내나 |
|---|---|---|---|
| `square_cw` | 코너마다 90° 회전(헤딩이 경로 따라감) | **+1800°** | 회전에서의 heading 드리프트 |
| `square_ccw` | 반대 방향 | **−1800°** | 방향 의존성(부호 뒤집힘 여부) |
| `strafe_square` | **헤딩 고정**, `+x→+y→−x→−y`로 변 이동 | **0°** | 순수 병진 드리프트(대조군) |

- **CW + CCW**: 부호가 방향 따라 뒤집히면 **스케일성 비대칭**(회전 과/과소 판독), 방향 무관하게
  같은 부호면 **방향 독립 바이어스**. 원인을 좁혀 준다. (방법 ②는 양방향이 필수이므로 그대로 재사용)
- **`strafe_square`(병진 전용)**: 회전이 0이라 나오는 드리프트는 순수 병진 성분. 회전 square의
  위치 드리프트와 대조하면 "위치 문제가 heading에서 왔는지 병진에서 왔는지"가 갈린다.
  - ⚠️ **방법 ① 한정 주의**: EKF는 보통 x,y는 바퀴, yaw만 IMU로 융합한다(가속도계는 위치에 안
    씀). 그래서 병진 전용에선 **fused ≈ swerve라 drift가 거의 0으로 나올 가능성이 크다.** 이건
    고장이 아니라 **의미 있는 결과**다 — "순수 병진에선 IMU 기여 없음 → 병진 정확도는 전적으로
    바퀴에 달림 → 방법 ②·③에서 병진 스케일을 직접 봐야 함"을 확인해 준다. 병진 전용의 진짜
    수확은 GT가 있는 ②·③(횡방향 스케일·strafe 슬립)에서 나온다.

---

## 4. 실행 방식 — 자동 주행, 연속 5바퀴

cmd_vel은 속도 명령이므로(§1), "L m 이동"은 노드가 **odom 피드백으로 직접 구현**한다:
시작 위치를 기록하고, 이동 거리가 목표에 닿을 때까지 속도를 **주기적으로 발행**하다가 도달하면
0을 발행해 멈춘다. 회전도 같은 방식(시작 yaw 기록 → 목표 각도까지).

```
# 거리 L 주행 (회전도 yaw로 동일하게)
x0, y0 = cur_x, cur_y
while hypot(cur_x - x0, cur_y - y0) < L:
    publish(vx=v)          # 20~50Hz로 계속 (timeout 대비), 목표 근처 감속(P)
publish(vx=0.0); settle()  # 정지 + 잠깐 대기(관성 제거)
```

- **피드백 소스**: 한쪽 오도메트리(기본 `fused_odom`)를 거리·각도 판정에 사용. 어느 쪽을 쓰든
  **두 추정기의 상호 드리프트 측정에는 영향이 없다**(진실이 없어 둘 다 같은 물리 움직임을 각자
  추정할 뿐). 선택은 임의이며 기록만 남긴다.
- **저속·완만한 가감속**, 각 구간 끝 **완전 정지·대기(settle)**.
- **안전장치**: 구간별 최대 시간·거리·각도 가드, 취소/종료 시 cmd_vel 0 발행, 속도 상한.

### 4.1 왜 "연속 5바퀴"인가 (1바퀴×5회가 아니라)

두 방식은 **다른 것을 측정**한다.
- **연속 5바퀴** → **누적 추세(trend)**: 드리프트가 회전·거리에 따라 선형으로 자라는지(계통적)
  유계로 진동하는지(노이즈)가 한 곡선으로 드러난다. 오래 연속 주행 시 바퀴 heading이 IMU에서
  얼마나 빨리 멀어지는지 = so-what에 직결.
- **1바퀴×5회** → **재현성/분산**: 평균±표준편차로 계통 vs 비계통 분리. 단 **방법 ①은 GT·바닥
  마크가 없어 매 회차 시작점에 정확히 되돌릴 수 없다** → 5회가 진짜 독립 표본이 아니라 표준편차가
  깨끗한 비계통 추정이 못 된다. **깨끗한 반복 통계는 마크 재배치가 되는 방법 ②의 몫**이다.

**결정**: 방법 ①은 **연속 5바퀴 + 매 바퀴 경계 체크포인트 기록**. 하나의 실행에서 누적 곡선과
바퀴별 증분(선형성·워밍업 이상치 판단용)을 함께 얻는다. → `loops=5, repeats=1`.
`loops`/`repeats`는 config로 분리해, 방법 ②에서 `loops=1, repeats=5`(마크 재배치 + 테이프
실측)로 전환한다.

---

## 5. 인터페이스 — info 서비스 + run 액션 (네임스페이스: 방법)

테스트를 한 번에 다 돌리지 않고, **하나씩 골라 호출**한다. 네임스페이스는 방법별(`/method1`,
이후 `/method2`·`/method3`)로 두고, **모든 방법이 같은 인터페이스 타입을 공유**한다
(→ 하나의 CLI로 어느 방법이든 조회·실행 가능). 정의는 공유 패키지 `odom_test_interfaces`에 둔다.

**info = 서비스** (짧게 끝남): 실행 가능한 테스트 목록·설명을 반환.
```
# odom_test_interfaces/srv/ListTests.srv    →  /method1/list_tests
---
string[] tests          # 예: square_cw, square_ccw, strafe_square, full
string[] descriptions   # 각 테스트 설명
string[] default_params # 기본 파라미터 요약
```

**run = 액션** (장시간·취소 가능): 테스트 이름을 넣어 호출하면 해당 테스트를 실행.
서비스가 아니라 액션인 이유 — 몇 분간 로봇이 움직이므로 ① 진행상황 **피드백**, ② 움직이는
로봇을 중간에 세우는 **취소(cancel, 안전상 필수)** 가 필요하기 때문.
```
# odom_test_interfaces/action/RunTest.action   →  /method1/run_test
# --- Goal ---
string test             # 실행할 테스트 (list_tests가 알려준 이름)
float64 side_length     # 비우면 config 기본값 사용
int32   loops
int32   repeats
bool    dry_run         # 실제 구동 없이 검증만
---
# --- Result ---
bool    success
string  message
string  output_path     # 기록 파일/bag 경로
float64 final_drift_pos
float64 final_drift_yaw
---
# --- Feedback ---
string  phase           # 현재 조건(square_cw 등)
int32   current_loop
int32   total_loops
string  current_segment # drive/rotate/strafe/settle
float64 drift_pos_so_far
float64 drift_yaw_so_far
```

> 순수 서비스만 원하면 대안(run 시작 서비스 + 별도 cancel 서비스 + status 토픽)도 가능하나,
> 액션이 이 셋을 한 번에 제공하므로 권장.

---

## 6. Config 관리 (YAML)

실험 설정은 전부 config로 뺀다. `odom_tester_bringup`에 두고 세 방법이 공유·오버라이드한다.

```yaml
/method1/odom_compare:
  ros__parameters:
    # 인터페이스
    cmd_vel_topic: /swerve_controller/cmd_vel
    odom_a_topic:  /swerve_controller/odom     # 비교 대상 A (swerve 원본)
    odom_b_topic:  /fused_odom                 # 비교 대상 B (EKF)
    feedback_source: b                          # 거리/각도 판정에 쓸 odom (a|b) — 측정엔 무영향
    odom_frame: odom
    base_frame:  base_footprint

    # 루틴
    preset: full            # square_cw | square_ccw | strafe_square | full
    side_length: 1.0        # [m]  1x1
    loops: 5                # 한 실행의 바퀴 수  (방법①=5)
    repeats: 1              # 독립 실행 횟수     (방법①=1, 방법②=5)

    # 속도·타이밍
    v_lin: 0.15             # [m/s]
    v_ang: 0.3              # [rad/s]
    accel_lin: 0.3          # 완만한 가감속
    publish_rate: 30.0      # [Hz]  cmd_vel timeout 대비 주기 발행
    settle_time: 1.0        # [s]   구간 끝 정지 대기
    reach_tol_lin: 0.02     # [m]
    reach_tol_ang: 0.02     # [rad]

    # 안전 가드
    max_seg_time: 30.0
    max_seg_dist: 1.5

    # 출력
    record_bag: true
    checkpoint_each_loop: true
    output_dir: ~/odom_tests
```

`run_test` 액션 goal의 필드(side_length/loops/repeats 등)로 config 기본값을 **호출 시
오버라이드**할 수 있다.

---

## 7. 프레임·재영점 규약 (단순형)

두 오도메트리가 같은 `odom` 프레임을 쓰므로 규약이 단순하다.

- **테스트 시작 시점에 각자 재영점**한다(과거 누적분 제거, 이번 주행분만 남김). 물리 리셋
  불필요 — 소프트웨어로 시작값 빼기로 충분·동등.
- **회전시키지 않는다.** 둘 다 `odom` 축을 그대로 쓰므로 시작 위치·heading만 뺀다. (몸체
  프레임으로 회전시키면 heading 원점 차이가 병진 비교에 새어들어 해롭다.)

```
S̄(t) = ( xS(t)−xS0,  yS(t)−yS0,  wrap(θS(t)−θS0) )     # swerve
F̄(t) = ( xF(t)−xF0,  yF(t)−yF0,  wrap(θF(t)−θF0) )     # fused(EKF)
drift_pos(t) = ‖ (x,y)_S̄ − (x,y)_F̄ ‖
drift_yaw(t) = wrap( θS̄ − θF̄ )
```

- **raw는 손실 없이 로깅**하고 위 재영점은 분석 시 적용 → "절대 위치 뷰"와 "변화량 뷰"가 분석 시
  토글(재실험 불필요). "절대 위치 vs 변화량"은 시작에 재영점하면 같은 것이다.

---

## 8. 결과값 (Outputs)

**실시간 토픽 / 액션 피드백** (PlotJuggler·RViz용) — 재영점된 두 궤적, `drift_pos(t)`, `drift_yaw(t)`.
**저장 파일**(CSV/bag) — 타임스탬프별 두 raw pose + 드리프트 + **바퀴별 체크포인트**.
**요약 통계** — 경로 종료 시점 드리프트, 미터당 드리프트율(m/m)·회전당 heading 드리프트(deg/rev),
구간별(직진/회전/strafe) 증가 패턴, 조건별(cw/ccw/strafe) 비교.

---

## 9. 결과 해석 (Interpretation) — 패턴 → 의미

- **회전 구간에서 `drift_yaw`가 튄다** → 바퀴 회전각과 IMU 회전각이 다름. 스워브 바퀴 heading은
  모듈 조향·트랙 기하 오차에 취약 → **바퀴 단독 heading을 믿으면 안 된다**는 신호이자 조향
  캘리브레이션 의심 근거. (②가 절대값으로 확정)
- **직진인데 `drift_pos`가 커진다** → 대개 heading 차이가 누적된 2차 효과. 위치 문제가 아니라
  heading 문제로 읽어야 한다. (`strafe_square`가 0에 가깝게 나오면 이 해석이 확인됨)
- **CW/CCW 부호 비교** → 뒤집히면 스케일성 비대칭, 같으면 방향 독립 바이어스.
- **거리·회전 대비 선형 증가 vs 유계 진동** → 계통적 불일치 vs 노이즈 수준.
- **drift ≈ 0 (모든 구간)** → 이 로봇에선 **정상 관측**이며 IMU 무시 신호가 아니다.
  실측된 EKF 구조(2026-07-08 확인): `/ekf_filter_node`가 **odom0(`/swerve_controller/odom`)에서
  절대 yaw + vx + vy** 를, **imu0(`/imu`)에서 yaw rate(vyaw)만** 융합한다(거부 임계값 ∞).
  즉 **바퀴가 "절대 yaw" 앵커**를 공급하므로 저속·직진에선 fused가 wheel에 수렴한다. IMU 기여는
  급회전·슬립 등 과도 구간에서 커진다. → **함의: 바퀴 yaw의 계통오차(조향 캘리브레이션 등)는
  fused에도 그대로 전파되고 방법 ①으론 안 보인다. 반드시 방법 ②·③(GT)로 확인.**
  (위험 신호: 정지 부근 `gyro z ≈ 0.07 rad/s` 바이어스 관측 → vyaw 융합 탓 yaw 드리프트 주입 가능,
  별도 점검 권장.)

**내릴 수 있는 결론**: 융합의 heading 기여도(정량), 추정기 발산·이상 감지, **어느 동작이 취약한지
→ 방법 ②·③의 조준점**, health 베이스라인(회귀 감지).
**내릴 수 없는 결론(경계)**: 절대 정확도, 어느 쪽이 "맞는지". → 방법 ②·③.

---

## 10. 시각화 (Visualization)

- **PlotJuggler** (`ros-jazzy-plotjuggler-ros`): 드리프트를 시간축, 두 궤적을 X-Y로. 1차 도구.
- **RViz2**: 두 오도메트리를 `odom` 프레임에 겹쳐 Path로 표시 → 벌어짐을 공간적으로.
- 저장 CSV → matplotlib로 드리프트 곡선·XY 오버레이(오프라인 리포트).
- RViz 설정·PlotJuggler 레이아웃은 `odom_tester_bringup`에.

---

## 11. 구현 전 확인

- 컨트롤러 **cmd_vel timeout** 실제값 → `publish_rate` 결정.
- 안전 **속도 상한**(`v_lin`, `v_ang`)·`settle_time`을 로봇 실제값에 맞춰 파라미터화.
- `/swerve_controller/cmd_vel` 발행 경합 정리(테스트 중 다른 명령 소스 비활성).

## 12. 관련 패키지 (메타패키지 구조와의 관계)

- `odom_test_interfaces` — `ListTests.srv`, `RunTest.action` (세 방법 공유).
- `odom_test_core` — `drive/strafe/rotate` 프리미티브 + 테스트 서버 베이스(재사용).
- `odom_compare` — 방법 ① 구현. `/method1` 네임스페이스로 info 서비스 + run 액션 노출.
- `odom_tester_bringup` — config·launch·RViz/PlotJuggler 레이아웃.
