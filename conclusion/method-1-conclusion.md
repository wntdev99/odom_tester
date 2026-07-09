# 방법 ① 결론 — swerve vs EKF 오도메트리 상대 비교 (1차 테스트)

> 첫 번째 테스트(`odom_compare`, 방법 ①)에서 **실주행으로 확실하게 확인된 것만** 정리한다.
> 추측·미검증 항목은 「아직 모르는 것(경계)」에 분리해 둔다.
> 상세 설계는 [`docs/method-1-relative-comparison.md`](../docs/method-1-relative-comparison.md) 참조.

- **실측일**: 2026-07-08
- **대상 로봇**: 자체 구현 스워브 드라이브 (ROS2 Jazzy)
- **성격**: 절대 정확도 아님. 두 추정기의 **상호 불일치(mutual divergence)** 만 측정.

---

## 1. 테스트 환경

| 항목 | 값 |
|---|---|
| 비교 대상 A | `/swerve_controller/odom` (바퀴 원본 오도메트리, `nav_msgs/Odometry`) |
| 비교 대상 B | `/fused_odom` (EKF 융합 오도메트리, 같은 타입) |
| 공통 프레임 | `odom → base_footprint` (두 오도메트리가 **같은 `odom` 프레임** → 좌표 변환 불필요) |
| 명령 | `/swerve_controller/cmd_vel` — `geometry_msgs/Twist`(unstamped), `vx/vy/wz` |
| cmd_vel timeout | 0.5 s → 주행 중 20~50 Hz 주기 발행 필수 |
| 속도 상한 | linear 0.8 m/s, angular 1.2 rad/s (컨트롤러 update_rate 30 Hz) |
| 테스트 실행 위치 | jeongmin 개발 PC(`/home/jeongmin/ros2_ws`) → 네트워크로 로봇에 cmd_vel 발행 |
| 주행 공간 | 평탄한 빈 공간 (1 m × 1 m + 여유). 외부 장비·바닥 마크·맵·초기위치 설정 없음 |

**EKF 구성 (실측, `/ekf_filter_node`, robot_localization, 50 Hz, two_d_mode):**
- `odom0 = /swerve_controller/odom` → **절대 yaw + vx + vy** 융합 (vyaw 미사용)
- `imu0 = /imu` → **yaw rate(vyaw)만** 융합 (절대 yaw·가속도 미사용), 100 Hz
- pose/twist 거부 임계값 = ∞ (모든 측정 채택)
- **핵심 함의**: 바퀴가 "절대 yaw 앵커"를 공급 → 저속·직진에선 `fused ≈ swerve`.

---

## 2. 테스트 방법

로봇을 자동으로 정해진 경로에 태워, 두 오도메트리가 `odom` 프레임에서 서로 얼마나 벌어지는지 기록.

- **주행 구현**: `cmd_vel`은 속도 명령이라 "몇 m"를 직접 못 줌 → **odom 피드백 루프**로 목표
  거리·각도까지 주기 발행(30 Hz) 후 정지. 구간 끝마다 완전 정지·대기(settle).
- **재영점**: 같은 `odom` 프레임이므로 회전 없이 시작 위치·heading만 빼서 이번 주행분만 남김.
- **실행 조건(preset)** — 총 회전량만 바꿔 회전/병진 성분 분리:

  | 조건 | 경로 | 5바퀴 누적 회전 |
  |---|---|---|
  | `square_cw` | 코너 90° 시계방향 | +1800° |
  | `square_ccw` | 코너 90° 반시계방향 | −1800° |
  | `strafe_square` | 헤딩 고정 병진(+x→+y→−x→−y) | 0° |

- **반복**: 연속 5바퀴 + 바퀴별 체크포인트 기록(`loops=5, repeats=1`) → 누적 추세 관찰.
- **튜닝**: `reach_tol_lin=1 mm`, `reach_tol_ang≈0.11°`, 최소속도 `v_lin_min=0.01 / v_ang_min=0.02`.
- **저장**: `results/*.csv`, `results/plots/*.png` (gitignore).

### 1차 실주행 진행 범위
- dry_run → 0.5 m × 1 → **1 m × 5바퀴 (`square_cw`) 완주**. 파이프라인 검증됨.
- `square_ccw`, `strafe_square` 5바퀴는 **아직 미실행** (TODO).

---

## 3. 테스트 결과

1. **파이프라인 동작 확인**: dry_run·단거리·5바퀴 연속 자동 주행, 피드백·정지·CSV 기록까지 정상.
2. **swerve ≈ fused (반복 확인)**: 5바퀴(≈20 m)에서
   - 위치 드리프트 `drift_pos < 0.2 mm`
   - 헤딩 드리프트 `drift_yaw < 0.05°`
   - RViz 오버레이(빨강=swerve, 파랑=fused, `odom` 프레임)에서도 두 궤적 화살표가 거의 포개짐.
3. **정사각형 비폐합(핀휠) 발견**: 로봇 **자기 odom** 기준으로 정사각형이 안 닫힘
   — 바퀴당 ≈ **−4 cm, −4°** 씩 어긋남.

---

## 4. 결론

### ✅ 확실하게 알게 된 것
1. **방법 ① 측정 파이프라인은 신뢰할 수 있다** — 자동 주행·피드백·취소·기록이 실주행에서 작동.
2. **저속·직진 조건에서 EKF는 바퀴 오도메트리에 사실상 수렴한다** (`fused ≈ swerve`).
   이는 실측된 EKF 구조로 설명됨: **바퀴가 절대 yaw 앵커**이고 IMU는 yaw rate만 기여하므로,
   과도(급회전·슬립) 구간이 아니면 IMU 기여가 작다. **정상 관측이며 IMU 무시 신호가 아니다.**
3. 따라서 **방법 ①은 절대 정확도를 측정할 수 없음이 구조적으로 확정됐다.**
   바퀴 yaw의 계통오차(조향 캘리브레이션 등)는 `fused`에도 그대로 전파되어 이 비교로는 상쇄돼
   보이지 않는다. → 절대 정확도는 반드시 **방법 ②(UMBmark)·③(AprilTag GT)** 로 확인해야 한다.
4. **핀휠(정사각형 비폐합)의 원인 규명 — 회전 무죄, "직진 중 heading 드리프트"**
   (2차 테스트, 타이트 tol — 상세 [archive/20260709-m1-square-cw-5loops-tight-tol](../archive/20260709-m1-square-cw-5loops-tight-tol/analysis.md)):
   - 허용오차를 1 mm/0.11°로 조였더니 핀휠이 **−4→−7.3°/loop로 오히려 증가** → "명령기 허용오차 아티팩트" 가설 **반증**.
   - full-rate TUM 분석: **코너 회전 평균 −89.82°(정확)**, 비회전(직진) 구간 yaw 드리프트 −40.7°
     → **직진 1 m당 heading ≈ −2° 드리프트**가 핀휠의 원인.
   - → **회전 제어·square 로직은 정상.** 문제는 "직진이 직진이 아님" = 스워브 **모듈 조향 영점** 유력.

### ⚠️ 아직 모르는 것 (경계 — GT 필요)
- **절대 오차 크기(cm·°).** 진실(GT)이 없어 판정 불가.
- **직진 heading 드리프트가 "실제 곡선 주행(모듈 영점·물리)"인지 "odom heading 오차"인지.**
  방법 ①으론 구분 불가 — GT(방법 ③, 부분적으로 ④)가 판별.
- **`square_ccw`·`strafe_square` 결과** — 미실행(방향 의존성·순수 병진 데이터 없음).
- **자이로 z 바이어스(~4.2°/s)** 실제 영향 미확인.

### 📌 전략 — 미보정 상태로 baseline 측정 (조향 영점 안 고치고 진행)
조향 영점 문제를 **지금 고치지 않고** 다른 테스트를 진행한다. 근거:
- 측정 방법들은 로봇이 완벽히 주행하기를 요구하지 않음 → 미보정이어도 "현재 상태 오차"를 유효하게 잰다.
- **미보정 baseline → 영점 수정 → 재측정**의 before/after로 보정 효과를 정량화(먼저 고치면 baseline 상실).
- GT 기반(④·③)은 덤으로 **물리 곡선 vs odom 오차를 판별**해 줌 → 진행이 오히려 생산적.
- 주의: 핀휠로 로봇이 드리프트하니 여유 공간 필요; ②의 보정계수 모델은 스워브 각색 필요; 큰 오차가 작은 오차를 가릴 수 있음(정상, 반복 캘리브레이션).

### ➡️ 다음 행동
- **방법 ④(MCL) 실주행** — 미보정 baseline 위치 잔차 + 직진 드리프트의 외부기준 대비 확인.
- 방법 ① 완성: `square_ccw` + `strafe_square` sanity.
- **방법 ③(GT)** 로 물리 vs odom 확정 → 이후 조향 영점 수정 → 재측정(before/after).
