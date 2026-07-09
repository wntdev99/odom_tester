# odom_tester — 개발 핸드오프 & TODO

> 스워브 드라이브 로봇의 **오도메트리 정확도**를 측정하는 ROS2(Jazzy) 도구 모음.
> 이 문서는 **다른 개발 에이전트가 이어서 개발**할 수 있도록 지금까지의 논의·결정·현황·남은
> 작업을 정리한 핸드오프다. 상세 설계는 [`docs/method-1-relative-comparison.md`](docs/method-1-relative-comparison.md) 참조.

## ⚠️ 협업 규칙 (반드시 준수)

- **실주행(로봇을 움직이는 테스트)은 개발 에이전트가 하지 않는다.** 오케스트레이터(주 세션)가
  사용자 허락을 받아 수행한다. 개발 에이전트는 **코드·문서·분석 스크립트·오프라인 작업만**.
- 로봇에 **쓰기 금지**(파라미터 set, 노드 재시작, 파일 수정). 조사는 읽기 전용(`ros2 topic/param`, ssh 조회)만.
- `/swerve_controller/cmd_vel` 에 절대 직접 발행하지 말 것(실주행은 오케스트레이터 담당).
- 장시간 로봇 명령은 Bash 툴 `timeout`을 명시(기본 120s에 잘림 — 과거 실수). 5바퀴 ≈ 6분.

---

## 1. 무엇을·왜

우리가 **직접 구현한** 스워브 오도메트리(ROS2 공식 스워브 컨트롤러 부재)가 얼마나 정확한지
실측한다. 오도메트리는 항법·액션 전 계층의 토대인데 자체 제작품이라 검증이 없다.
핵심 원칙: **오차를 성격별로 분리**(계통 vs 비계통), 그리고 **진실(GT)의 출처를 명확히**.

## 2. 로봇 인터페이스 (실기 실측, 2026-07-08)

| 항목 | 값 |
|---|---|
| 명령 | `/swerve_controller/cmd_vel` — `geometry_msgs/Twist`(unstamped). `vx=linear.x, vy=linear.y, wz=angular.z` |
| cmd_vel timeout | **0.5 s** → 주행 중 20~50 Hz 주기 발행 필수 |
| 속도 상한 | linear 0.8 m/s, angular 1.2 rad/s. 컨트롤러 update_rate 30 Hz |
| 원본 오도메트리 | `/swerve_controller/odom` — `nav_msgs/Odometry`, `odom → base_footprint` |
| 융합 오도메트리 | `/fused_odom` — 같은 타입·프레임 |
| 로봇 기준 프레임 | **`base_footprint`** (base_link 아님) |
| 테스트 노드 실행 위치 | jeongmin 개발 PC(`/home/jeongmin/ros2_ws`), 네트워크로 로봇에 cmd_vel 발행 |

**EKF 구성 (호스트 `ubuntu@192.168.34.201`, `/ekf_filter_node`, robot_localization, 50Hz, two_d_mode):**
- `odom0 = /swerve_controller/odom` → **절대 yaw + vx + vy** 융합 (vyaw 미사용)
- `imu0 = /imu` → **yaw rate(vyaw)만** 융합 (절대 yaw·가속도 미사용), 100Hz
- pose/twist rejection threshold = ∞ (모든 측정 채택)
- yaml: `/home/ubuntu/colcon_ws/install/share/w_type_mw/config/ekf.yaml` (상단 "IMU 제외" 주석은 **stale/틀림**)
- **함의**: 바퀴가 절대 yaw 앵커 → 저속·직진에선 `fused ≈ swerve`. **바퀴 yaw 계통오차는 fused에도 전파**되어 방법 ①으로는 안 보임 → 절대 정확도는 방법 ②·③ 필요.
- **위험 신호**: 정지 부근 `/imu` gyro z ≈ 0.0735 rad/s(~4.2°/s) 바이어스 → yaw 드리프트 주입 가능. 점검 대상.

## 3. 논의한 세 가지 측정 방법

| 방법 | 패키지 | 진실(truth) 출처 | 재는 것 | 상태 |
|---|---|---|---|---|
| **① 상대 비교** | `odom_compare` | 없음 | swerve↔fused **상호 불일치** (절대 정확도 아님) | **구현·실주행 검증됨** |
| **④ MCL 잔차** | `odom_mcl` | `/mcl_pose` (부분·순환) | **운영 위치오차 (준-절대)** | **코드 구현됨, 실주행 미착수** |
| **② UMBmark + 테이프** | `odom_umbmark` (미생성) | 자로 잰 물리 시작/끝 | **계통오차 분리·보정계수** (끝점) | 미착수 |
| **③ AprilTag GT** | `odom_apriltag_gt` (미생성) | 천장 카메라 궤적 | **절대 오차(경로 전체)** | 미착수 |

네 방법은 보완 관계: ①=싼 상시 감시, ④=장비 없이 준-절대 감시(맵+라이다), ②=장비 없이
계통오차+보정, ③=경로 전체 절대 진실. ②→③ 순으로 정교해지고 ③가 ②·④를 검증.
**①은 절대 정확도를 못 잡음이 구조적으로 확정**(EKF 앵커). **④도 MCL이 odom을 모션모델로 쓰는
순환성 탓 원시 odom 오차를 과소평가** — 독립 GT(③)가 최종 판정. (상세: `docs/method-4-mcl-reference.md`)

## 4. 리포지토리 구조 (메타패키지)

```
odom_tester/                 ← 메타패키지 (ament_cmake, exec_depend만)
odom_test_interfaces/        ← ListTests.srv + RunTest.action (세 방법 공유)
odom_test_core/              ← pose_utils.py(재영점·yaw·wrap), primitives.py(drive/strafe/rotate)
odom_compare/                ← 방법 ① 노드 (info 서비스 + run 액션, /method1 네임스페이스)
odom_mcl/                    ← 방법 ④ 노드 (MCL 잔차, /method4 네임스페이스)
odom_tester_bringup/         ← config/method{1,4}.yaml + launch/method{1,4}.launch.py
scripts/analyze_method1.py   ← 방법① CSV → 드리프트 곡선·XY 오버레이 (matplotlib, ROS 비의존)
scripts/analyze_method4.py   ← 방법④ CSV → MCL 잔차 곡선·품질 게이팅·XY 오버레이 (ROS 비의존)
docs/method-1-relative-comparison.md  ← 방법 ① 상세 명세
docs/method-4-mcl-reference.md        ← 방법 ④ 상세 명세
conclusion/                  ← 방법별 실주행 결론 모음 (README 인덱스 + method-N-conclusion.md)
results/                     ← 실험 산출물 (gitignore, 커밋 안 됨)
```
- 원격: `github.com/wntdev99/odom_tester` (branch `main`).
- 빌드: `cd ~/ros2_ws && colcon build --packages-up-to odom_tester`
- 실행: `ros2 launch odom_tester_bringup method1.launch.py` (네임스페이스 `/method1`)

### 인터페이스 (공유, `odom_test_interfaces`)
- **info = 서비스** `/method1/list_tests` (`ListTests.srv`) → 실행 가능한 테스트 목록.
- **run = 액션** `/method1/run_test` (`RunTest.action`) → 테스트 실행(피드백·취소 지원).
  액션인 이유: 장시간·취소(안전)·피드백. Goal: test, side_length, loops, repeats, dry_run.

## 5. 방법 ① 설계 결정 요약 (상세는 docs/)

- **성격**: swerve↔fused **상호 불일치**만. 절대 "오차"라 부르지 않는다.
- **명령**: cmd_vel은 속도 명령이라 "몇 m"를 직접 못 줌 → **odom 피드백 루프**로 목표까지 주행 후 정지. 주기 발행(30Hz).
- **재영점**: 테스트 시작 시점에 각자 재영점, **같은 odom 프레임이므로 회전 없이 시작 위치·heading만 뺌**.
- **세 조건(preset)**: `square_cw`(누적 +회전), `square_ccw`(−회전), `strafe_square`(헤딩 고정 병진, 회전 0). `full`=순차.
  - 병진 전용은 ① 한정 near-zero 가능(EKF가 x,y에 IMU 안 씀) → 정상. 진짜 수확은 ②·③.
- **반복**: ①은 **연속 5바퀴 + 바퀴별 체크포인트**(추세). ②는 `loops=1, repeats=5`(마크 재배치·테이프).
- **저장**: `results/`(상대경로, CWD 기준, gitignore). config `output_dir`.
- **튜닝(완료)**: `reach_tol_lin=0.001(1mm)`, `reach_tol_ang=0.002(~0.11°)`, 최소속도 `v_lin_min=0.01/v_ang_min=0.02` 노출(허용오차만 낮추면 헌팅 → 함께 조정). **주의: 실행 정밀도·폐합 개선용, 오도메트리 정확도 불변.**

## 6. 지금까지 결과

- 실주행: dry_run → 0.5m×1 → 1m×5(CW) 완주. 파이프라인 검증됨.
- **swerve ≈ fused** 반복 확인: 5바퀴(20m)에서 drift_pos <0.2mm, drift_yaw <0.05°. RViz `~/odom_vs_fused.png`(빨강=swerve, 파랑=fused, odom 프레임)에서도 화살표 거의 포개짐.
- **정사각형 비폐합** 발견: 로봇 자기 odom이 바퀴당 ≈ −4cm, −4°로 안 닫힘(핀휠). **대부분 명령기 각도 허용오차 아티팩트**로 추정(5절 튜닝으로 축소 시도) — 진짜 미회전인지 odom 오차인지는 **GT로만** 구분.
- 데이터: `results/*.csv`, `results/plots/*.png`.

## 7. 남은 작업 (TODO)

### 개발 (다른 에이전트 담당 — 오프라인)
- [x] **방법 ① 분석 강화**(`analyze_method1.py`): 자기 odom **폐루프 비폐합(핀휠)을 상호 드리프트와
  분리** 표기(플롯+통계), CW/CCW 부호 비교(상호+비폐합 yaw), 마크다운 요약 리포트(`summary_method1.md`).
  합성 CSV로 end-to-end 검증됨.
- [x] **방법 ④ `odom_mcl` 패키지 생성**: `/method4`, MCL(`/mcl_pose`) 기준 잔차. 코어 재사용,
  SE(2) 시작정렬(`pose_utils.compose/inverse/align_transform`), 공분산 게이팅, gt_topic/gt_type 파라미터화.
  - [x] `scripts/analyze_method4.py`: 잔차 곡선(A/B, pos/yaw)·품질 게이팅 표시·바퀴당 증가율·map
    프레임 XY 오버레이. 합성 CSV로 end-to-end 검증됨.
  - [ ] **evo TUM 내보내기 + 진짜 snap-back**: 현재 CSV는 바퀴당 체크포인트라 evo·snap-back엔
    부족 → 노드에 **full-rate 궤적 로깅(timestamp 포함)** 추가가 선행 과제.
  - [ ] **`/mcl_pose` 실제 토픽·타입 확인**(읽기 전용 조회) 후 `method4.yaml`의 `gt_topic`/`gt_type` 확정.
- [ ] **방법 ② `odom_umbmark` 패키지 생성**: 코어 명령기 재사용. `umbmark_cw/ccw` 러너, **테이프 실측 입력**(CLI/CSV), UMBmark 분석기(무게중심, `E_max,syst`, Type A/B 분해 → 보정계수). `loops=1, repeats=5` 양방향. `/method2` 네임스페이스, 같은 info/run 인터페이스.
- [ ] **방법 ③ `odom_apriltag_gt` 패키지 생성**: `christianrauch/apriltag_ros`(`apt install ros-jazzy-apriltag-ros`), `tag36h11`, 바닥 기준 태그(월드 원점), `image_proc` rectify, intrinsic 캘리브레이션. GT pose 발행 + rosbag + evo(ATE/RPE) 어댑터. `/method3`.
- [ ] **evo 연동**: /odom·/fused_odom·GT를 TUM/bag으로 저장→evo_ape/rpe(2D는 `-s` 원점정렬).
- [ ] **자이로 바이어스 로거**: 정지/직진/급회전 시 `/imu`(gyro z)·wheel yaw·fused yaw 동시 로깅·비교 툴(방법① 확장 or 별도). (로깅 코드는 개발, 실행은 오케스트레이터)
- [ ] **문서**: 방법 ②·③ 명세를 `docs/`에 방법 ①과 같은 형식으로.
- [ ] (선택) RViz 설정·PlotJuggler 레이아웃을 `odom_tester_bringup`에 추가.

### 테스트 = 실주행 (오케스트레이터 담당, 사용자 허락 필요) — **개발 에이전트는 하지 말 것**
- [ ] 타이트 허용오차(1mm/0.11°)로 재주행 → 비폐합(핀휠) 개선 확인. 헌팅하면 `reach_tol`·`v_min` 재조정.
- [ ] 방법 ① 완성: `square_ccw` 5바퀴 + `strafe_square` 실주행·분석.
- [ ] 방법 ④ 실측: `/mcl_pose` 확인·로컬라이제이션 수렴 상태에서 feature-rich 공간 주행·분석.
- [ ] 방법 ②·③ 실측(패키지 완성 후).

### 로봇 측 (사용자/관리자, 별도 승인)
- [ ] `ekf.yaml` 상단 stale 주석 정정("IMU 제외"→실제 imu0 활성).
- [ ] 자이로 z 바이어스 캘리브레이션 검토.

## 8. 알려진 이슈 / 주의 (gotchas)

- 장시간 실주행은 Bash 툴 `timeout` 명시(기본 120s에 잘림).
- `/swerve_controller/cmd_vel` 에 발행자 다수(≈7) — 테스트 중 다른 명령 소스(nav2/docking/teleop) 비활성 확인.
- 타이트 허용오차는 스워브 마찰·데드밴드로 물리적 바닥 존재 → 헌팅 시 완화.
- 방법 ①·RViz 모두 odom 프레임(로봇 믿음)이라 **절대 정확도·물리적 폐합은 판정 불가** — GT 필요.
- `results/`는 gitignore. 커밋되는 건 코드·문서뿐.
