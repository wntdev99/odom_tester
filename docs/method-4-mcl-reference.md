# 방법 ④ — MCL pose 기준 오도메트리 잔차 (`odom_mcl`)

> 로봇을 정해진 경로로 자동 주행시키면서, `odom`(바퀴/EKF)이 **로컬라이제이션(MCL) pose**
> 로부터 얼마나 벌어지는지를 본다. 외부 장비 없이 **로봇이 이미 쓰는 내비게이션 스택**만으로
> 잰다. 방법 ③(AprilTag GT)의 저렴한 준(準)-절대 버전이지만 **재는 대상이 다르다**(§0).

---

## 0. 범위와 성격 — 먼저 못 박고 간다

- **비교**: `odom`(검사 대상, 예 `/swerve_controller/odom`·`/fused_odom`) vs **`/mcl_pose`**(로컬라이제이션 추정).
- **⚠️ MCL은 독립 진실(GT)이 아니다 — 순환성이 있다.**
  MCL(파티클 필터)은 **odometry를 모션 모델 입력으로 쓰고** 라이다-맵 매칭으로 보정한다.
  즉 `MCL pose = odom 전파 + 라이다 보정`이라, odom을 MCL과 비교하면 피검사체가 자기 계산
  루프 안에 들어가 있다. 스캔 업데이트 사이에서 MCL이 odom에 끌려다니므로 **원시 odom 오차를
  과소평가(낙관적)** 한다. (방법 ①에서 "fused는 swerve로 만든 값이라 상호 비교로 절대 정확도
  못 잡음"이라 못 박은 것과 같은 종류의 오염이다.)
- **따라서 방법 ④가 재는 것은 "원시 odom 절대 오차"가 아니라 "운영 중 로봇의 위치 인식 오차
  (localized-pose residual)"** 다 — odom+라이다+맵 풀스택이 돌 때 로봇이 자기 위치를 얼마나
  잘 아는가. 내비게이션 관점에선 오히려 이게 더 직접적인 지표다.
- **규율**: 이 방법의 숫자를 절대적 "정확도(accuracy)"라 부르지 않는다. **"MCL 대비 잔차
  (residual)", "운영 위치오차"** 로 부른다. 진짜 절대 정확도는 독립 GT를 갖는 방법 ③이 담당한다.

### 방법 사다리에서의 위치
| 방법 | GT 출처 | 독립성 | 재는 것 | 비용 |
|---|---|---|---|---|
| ① 상대 비교 | 없음 | — | swerve↔fused 상호 불일치 | 최저 (상시) |
| **④ MCL 잔차** | `/mcl_pose` | **부분(순환)** | **운영 위치오차(준-절대)** | **낮음 (맵+라이다만)** |
| ② UMBmark | 자로 잰 끝점 | 완전 | 계통오차·보정계수 | 중 (테이프) |
| ③ AprilTag | 천장 카메라 | 완전 | 절대 오차(경로 전체) | 높음 (설치·캘리브) |

방법 ④ = **장비 없이 상시 돌릴 수 있는 준-절대 감시.** ③가 나오면 ③가 ④를 검증한다.

---

## 1. 인터페이스 (토픽·타입은 config로 — 로봇 확인 후 확정)

| 대상 | 토픽 (파라미터) | 타입 | 프레임 |
|---|---|---|---|
| 검사 대상 A | `odom_a_topic` = `/swerve_controller/odom` | `nav_msgs/Odometry` | `odom → base_footprint` |
| 검사 대상 B | `odom_b_topic` = `/fused_odom` | `nav_msgs/Odometry` | `odom → base_footprint` |
| **로컬라이제이션** | `gt_topic` = **`/mcl_pose`** | `gt_type`로 지정 | `map → base_footprint` |
| 명령 | `cmd_vel_topic` = `/swerve_controller/cmd_vel` | `geometry_msgs/Twist` | body |

- **`/mcl_pose`의 정확한 메시지 타입은 아직 미확정** → `gt_type` 파라미터로 스위칭한다:
  - `pose_with_cov` — `geometry_msgs/PoseWithCovarianceStamped` (기본, 공분산 게이팅 가능)
  - `odom` — `nav_msgs/Odometry` (공분산 게이팅 가능)
  - `pose` — `geometry_msgs/PoseStamped` (공분산 없음 → 게이팅 비활성)
  - **구현 원칙**: 토픽 이름·타입은 노드 하드코딩이 아니라 **config에서 마지막에 한 줄로 교체**할
    수 있게 둔다. 로봇에서 실제 토픽·타입 확인 후 yaml만 고친다.
- **두 프레임이 다르다**(`map` vs `odom`) → 방법 ①과 달리 **시작 시점 정렬(§7)이 필수**다.

---

## 2. 준비 (Preparation)

- **로컬라이제이션이 살아 있고 수렴한 상태**여야 한다: 맵 로드됨, `/mcl_pose` 발행 중,
  초기 pose 설정·수렴 완료(공분산 낮음).
- **feature-rich 공간**을 고른다(코너·벽·구조물). 긴 복도·대칭·개활지 등 feature-poor에서는
  MCL이 틀어져 GT 자격을 잃는다(§0의 전제 "로컬라이제이션이 틀어지지 않는 이상").
- 경로용 평탄 공간(1 m×1 m + 여유), `cmd_vel` 발행 경합 정리(방법 ①과 동일).
- 맵 품질이 GT 품질의 상한이다 — 최근 맵·정합 상태 확인.

---

## 3. 실험 조건 — 방법 ①과 동일 preset 재사용

동작별 취약점을 방법 ①과 같은 축으로 보기 위해 preset을 공유한다.

| 조건(preset) | 경로 | 5바퀴 누적 회전 | 방법 ④에서 보는 것 |
|---|---|---|---|
| `square_cw` | 코너 90° 시계 | +1800° | 회전 누적 시 위치·heading 잔차 |
| `square_ccw` | 코너 90° 반시계 | −1800° | 방향 의존성 |
| `strafe_square` | 헤딩 고정 병진 | 0° | 순수 병진 잔차 (방법 ①과 달리 MCL이 x,y 절대보정 → 0이 아닐 수 있음 = 유의미) |

- 방법 ①에서 `strafe_square`는 `fused≈swerve`라 0에 가까웠지만, **방법 ④는 라이다가 x,y를
  절대 보정**하므로 병진 잔차가 실제로 잡힐 수 있다 — 이게 방법 ④의 수확 지점 중 하나.

---

## 4. 실행 방식 — 자동 주행, odom 피드백 (방법 ① 코어 재사용)

- 주행 제어는 방법 ①과 동일: **`odom` 피드백 루프**로 목표 거리·각도까지 주기 발행 후 정지.
  `odom_test_core`의 `drive/strafe/rotate` 프리미티브를 그대로 쓴다.
- **`/mcl_pose`는 측정 전용 — 주행 제어에 절대 쓰지 않는다.** (피드백을 MCL로 걸면 제어 루프에
  라이다 지연·점프가 섞여 위험. 제어는 매끄러운 odom으로.)
- **점프 대응**: MCL은 스캔 매칭마다 pose가 튀므로, 매 구간 끝 **settle 후** MCL이 안정된
  시점의 샘플을 체크포인트로 기록. RPE보다 **ATE(누적 절대오차)** 해석에 무게.
- 안전장치(구간 시간·거리·각도 가드, 취소 시 cmd_vel 0)는 방법 ①과 동일.

---

## 5. 인터페이스 — info 서비스 + run 액션 (`/method4`)

**세 방법과 동일한 공유 타입**(`odom_test_interfaces`)을 쓴다. 네임스페이스만 `/method4`.

- **info** = `/method4/list_tests` (`ListTests.srv`) → `square_cw|square_ccw|strafe_square|full`.
- **run** = `/method4/run_test` (`RunTest.action`).
  - Goal: `test, side_length, loops, repeats, dry_run` (방법 ①과 동일).
  - **Result 의미 재정의**(공유 필드 재사용): `final_drift_pos/yaw` → **"MCL 대비 잔차(pos/yaw)"**.
    두 검사대상(A/B) 중 대표값 하나 + 상세는 CSV.
  - Feedback: `drift_pos/yaw_so_far` → 진행 중 MCL 잔차.

> 필드 이름(`drift_*`)은 세 방법 공유라 그대로 두되, 방법 ④ 맥락에선 "MCL 잔차"로 읽는다.

---

## 6. Config 관리 (YAML) — `/method4`

```yaml
/method4/odom_mcl:
  ros__parameters:
    # 인터페이스
    cmd_vel_topic: /swerve_controller/cmd_vel
    odom_a_topic:  /swerve_controller/odom     # 검사 대상 A
    odom_b_topic:  /fused_odom                 # 검사 대상 B
    gt_topic:      /mcl_pose                   # ← 로컬라이제이션 (마지막에 여기만 수정)
    gt_type:       pose_with_cov               # pose_with_cov | odom | pose
    feedback_source: b                          # 주행 제어에 쓸 odom (a|b) — MCL 아님
    map_frame:  map
    odom_frame: odom
    base_frame: base_footprint

    # 품질 게이팅 (MCL 신뢰 구간만 채택)
    max_cov_pos: 0.05        # [m^2] 위치 분산 상한 — 초과 샘플은 low-confidence 표시
    max_cov_yaw: 0.02        # [rad^2] yaw 분산 상한
    gt_timeout:  1.0         # [s] mcl_pose 갱신 끊김 허용

    # 루틴 / 속도·타이밍 / 안전 가드 — 방법 ①과 동일 기본값
    side_length: 1.0
    loops: 5
    repeats: 1
    v_lin: 0.15
    v_ang: 0.3
    publish_rate: 30.0
    settle_time: 1.5         # MCL 안정화 위해 방법①보다 약간 길게
    reach_tol_lin: 0.001
    reach_tol_ang: 0.002
    v_lin_min: 0.01
    v_ang_min: 0.02
    slowdown_lin: 0.10
    slowdown_ang: 0.10
    max_seg_time: 30.0
    max_seg_dist: 1.5

    # 출력
    output_dir: results
```

---

## 7. 프레임·정렬 규약 (map ↔ odom) — 방법 ①과 다른 핵심

`odom`은 `odom` 프레임, `/mcl_pose`는 `map` 프레임 → **두 프레임 축이 회전으로 어긋나** 있다.
단순히 시작값을 빼는 것(방법 ①)으론 안 되고 **시작 시점 강체 정렬(SE(2))** 이 필요하다.

- 테스트 시작 t₀에 두 pose가 **같은 물리 로봇 자세**라고 보고, 고정 변환을 잡는다:
  ```
  T_map←odom = mcl(t0) ∘ odom(t0)^-1        # SE(2) 강체 (evo의 원점정렬 -s 와 동일)
  odom_in_map(t) = T_map←odom ∘ odom(t)     # odom 궤적을 map 프레임으로
  err_pos(t) = ‖ odom_in_map(t).xy − mcl(t).xy ‖
  err_yaw(t) = wrap( odom_in_map(t).yaw − mcl(t).yaw )
  ```
- 이렇게 하면 프레임 회전차가 제거되고 **"시작 후 궤적이 얼마나 벌어졌나"** 만 남는다.
- A/B 각각에 대해 별도 정렬·잔차 계산(`err_a`, `err_b`).
- **raw는 손실 없이 로깅**(odom_a/b, mcl, 공분산) → 정렬·게이팅은 분석 단계에서 토글 가능.

---

## 8. 결과값 (Outputs)

- **실시간 피드백/토픽**: `err_pos(t)`, `err_yaw(t)`, 현재 MCL 공분산.
- **저장 CSV**: 타임스탬프별 raw(odom_a/b, mcl x/y/yaw) + 공분산 + 정렬 후 err_a/err_b +
  바퀴별 체크포인트 + **품질 플래그**(공분산 게이팅 통과 여부).
- **요약**: 종료 시점 잔차, 미터당·회전당 잔차율, 조건별(cw/ccw/strafe) 비교,
  **snap-back 통계**(구간 내 잔차 증가폭 ≈ 보정 사이 누적 odom 오차 — 순환성의 부산물).
- **full-rate TUM + evo**: `record_tum: true`면 실행마다 `*_odom_a.tum`·`*_odom_b.tum`·`*_mcl.tum`
  (전 구간 궤적, 원본 타임스탬프)을 저장한다. `scripts/run_evo.py`가 세트를 묶어 **mcl 기준**으로
  `evo_ape`/`evo_rpe`를 돌린다(기본 `--align_origin` — 노드 시작정렬과 같은 관점, scale `-s`는
  오도메트리 스케일오차를 지우므로 미사용). 체크포인트 CSV(요약)와 별개의 연속 궤적이라 evo·(추후)
  snap-back 분석의 입력이 된다.

---

## 9. 결과 해석 (Interpretation)

- **잔차가 회전·거리에 따라 커진다** → odom이 드리프트하고 MCL이 보정 중. 증가폭이 실제 odom
  드리프트의 하한(순환성 탓 과소평가).
- **snap-back(잔차가 커졌다 스캔 보정으로 급감)** → 급감폭 ≈ 보정 사이 누적 odom 오차. 이 값이
  방법 ④에서 원시 odom 오차에 가장 근접한 신호.
- **`strafe_square`에서 잔차가 유의미** → 병진 스케일 문제를 라이다가 잡아냄(방법 ①로는 안 보였던
  것) → 방법 ②·③의 병진 스케일 조준점.
- **공분산이 커지는 구간의 잔차는 신뢰 불가** → 품질 플래그로 배제. MCL이 틀어진 것이지 odom
  문제가 아닐 수 있음.
- **경계(내릴 수 없는 결론)**: 진짜 절대 오차·어느 쪽이 맞는지. 순환성·MCL 자체 오차(cm급, yaw
  약함) 때문. → 독립 GT인 방법 ③이 확정.

---

## 10. 시각화 (Visualization)

- PlotJuggler: `err_pos/err_yaw`를 시간축, odom·mcl 궤적을 X-Y(map 프레임)로. 공분산 오버레이.
- RViz2: `map` 프레임에서 mcl pose와 (정렬된) odom Path 겹치기.
- 오프라인: CSV → matplotlib 잔차 곡선·XY 오버레이 (`scripts/analyze_method4.py`, ROS 비의존).

---

## 11. 구현 전 확인 (로봇에서 — 읽기 전용 조회)

- **`/mcl_pose` 실제 존재·타입·주기** 확인 → `gt_topic`/`gt_type` 확정. (현재 미확인 — 이 방법의 전제)
- 맵 로드·초기 pose 수렴 상태, 공분산 정상 범위 → `max_cov_*` 튜닝.
- feature-rich 주행 공간 확보.
- `cmd_vel` 발행 경합 정리(방법 ①과 동일).

## 12. 관련 패키지

- `odom_test_interfaces` — `ListTests.srv`, `RunTest.action` (공유).
- `odom_test_core` — `drive/strafe/rotate` 프리미티브 + `pose_utils`(SE(2) `compose/inverse`) +
  `recorder`(full-rate TUM 저장, 세 방법 공유).
- `odom_mcl` — 방법 ④ 구현. `/method4` 네임스페이스로 info 서비스 + run 액션.
- `odom_tester_bringup` — `config/method4.yaml` + `launch/method4.launch.py`.
