# 방법 ① baseline — square_cw 1 m × 5바퀴 (2026-07-08)

> 방법 ①(swerve vs EKF fused 상대 비교)의 **첫 정식 baseline 실험** 기록.
> 실험 1건의 근거+분석이다. 방법 단위 종합은 [`conclusion/method-1-conclusion.md`](../../conclusion/method-1-conclusion.md) 참조.

## 목적
회전이 누적되는 조건에서 **swerve 원본과 EKF 융합 오도메트리가 서로 얼마나 갈리는지**(상호 드리프트)와, 로봇 **자기 odom의 정사각형 폐합 특성**을 본다. (절대 정확도 아님.)

## 셋업·파라미터
- 로봇: 자체 스워브(ROS2 Jazzy). 테스트 노드는 개발 PC에서 네트워크로 `cmd_vel` 발행.
- 비교: `A=/swerve_controller/odom`(바퀴) vs `B=/fused_odom`(EKF). 공통 `odom→base_footprint`.
- 경로: `square_cw`, **1 m × 1 m, 5바퀴 연속**(`loops=5, repeats=1`). vx 0.15, wz 0.3 m/s·rad/s, 저속.
- 허용오차: **당시 `reach_tol_lin=0.02 m`, `reach_tol_ang=0.02 rad`(≈1.15°)** — 튜닝 전 값(이후 1 mm/0.11°로 조임).
- EKF(실측): `odom0`=절대 yaw+vx+vy, `imu0`=vyaw만, 거부 임계 ∞.

## 데이터
- `square_cw_5loops_1m.csv` — 5바퀴 체크포인트 (`loop, ax,ay,ayaw, bx,by,byaw, drift_pos, drift_yaw`). **핵심 데이터.**
- `precursor_square_cw_1loop_0p5m.csv` — 선행 0.5 m×1바퀴 파이프라인 검증 run(체크포인트 1개).
- `plots/`(gitignore·재생성): `*_drift.png`, `*_nonclosure.png`, `*_xy.png`, `summary_method1.md`.
  재생성: `python3 scripts/analyze_method1.py --input archive/20260708-m1-square-cw-5loops-baseline/square_cw_5loops_1m.csv --output <이 폴더>/plots`

## 결과
- **상호 드리프트 (A↔B)**: 최종 `pos = 0.185 mm`, `yaw = 0.041°`. 증가율 0.036 mm/loop, 0.0067°/loop → **사실상 일치**.
- **자기 비폐합 (핀휠)**: swerve 최종 `pos = 0.242 m`, `yaw = −19.45°`(≈ **−4.1°/loop**). 로봇 odom이 정사각형을 못 닫고 회전·이탈.

## 해석
- **swerve ≈ fused**: EKF 구조로 설명됨 — 바퀴가 절대 yaw 앵커라 저속·저슬립에선 IMU(vyaw) 기여가 작아 융합이 사실상 통과(pass-through).
- **핀휠 −4.1°/loop**: 당시 각도 허용오차(0.02 rad ≈ 1.15°/turn) × 4 turn ≈ 4.6°/loop과 정합 → **대부분 명령기 허용오차 아티팩트로 추정**(확정 아님).

## 결론
**✅ 확실**
- 방법 ① 파이프라인 신뢰(자동 주행·피드백·취소·기록 동작).
- 저속·직진에서 `fused ≈ swerve`(구조적, IMU 무시 신호 아님).
- **방법 ①은 절대 정확도 측정 불가 확정**(swerve·fused가 공통 바퀴 앵커 공유).

**⚠️ 경계 (이 실험으로 판정 불가 → GT 필요)**
- 절대 오차 크기(cm·°).
- 핀휠이 **실제 미회전**인지 **odom 오차**인지 — GT(방법 ②·③)로만 구분.

## 다음
- 튜닝 후(1 mm/0.11°) 재주행으로 핀휠 아티팩트 축소 확인.
- `square_ccw` 5바퀴 정식, `strafe_square` 1회 sanity.
- 절대 정확도: 방법 ④(MCL) → ②(UMBmark) → ③(AprilTag GT).
