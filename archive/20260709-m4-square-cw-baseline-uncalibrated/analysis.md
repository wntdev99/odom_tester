# 방법 ④ — square_cw 1 m × 5바퀴, MCL 잔차 (미보정 baseline, 2026-07-09)

> 방법 ④(`odom_mcl`)의 **첫 실주행**. odom(바퀴/EKF) vs `/mcl_pose`(LiDAR 로컬라이제이션)
> 잔차를 측정. **조향 영점 미보정 상태의 baseline.** 방법 단위 종합은
> [`conclusion/`](../../conclusion/). 성격(순환성·준-절대)은 [`docs/method-4-mcl-reference.md`](../../docs/method-4-mcl-reference.md) §0.

## 목적
방법 ①이 못 잡은 **절대(준-절대) odom 오차**를 외부 기준(MCL)으로 처음 정량화하고,
"직진 heading 드리프트가 실제 곡선(물리)인지 odom 오차인지"를 판별하는 단서를 얻는다.
미보정 상태 = 이후 영점 수정 전 **baseline**.

## 셋업
- 경로: `square_cw`, 1 m × 1 m, 5바퀴(≈20 m). 방법 ① baseline과 동일 조건.
- 기준: `gt_topic=/mcl_pose`(`PoseWithCovarianceStamped`), 30 Hz. 시작 정렬 품질 게이트 통과.
- EKF/명령/속도: 방법 ①과 동일. 데이터: `m4_square_cw_5loops.csv` + full-rate TUM(odom_a/odom_b/mcl).

## 결과
**체크포인트 잔차 (단조 증가):**
| loop | err pos (fused) | err yaw (fused) |
|---|---|---|
| 0 | 0.024 m | −3.1° |
| 1 | 0.058 m | −6.4° |
| 2 | 0.095 m | −9.5° |
| 3 | 0.143 m | −12.7° |
| 4 | **0.181 m** | **−15.0°** |

- 증가율 ≈ **4.0 cm/loop, −3.0°/loop**. swerve(A)≈fused(B) (0.181 vs 0.181) — 방법 ① 결과와 정합.

**snap-back (full-rate TUM):** 잔차가 **톱니(sawtooth)** — 매 직진에서 yaw 잔차가 −2~3° 자라고
MCL 보정에서 되돌아오며(빨간 X), 포락선이 −17°까지 누적. 위치 잔차 최대 **0.22 m**(mean 0.11).
→ **방법 ①의 "직진 1 m당 ≈ −2° heading 드리프트"를 외부 기준에서 그대로 재현.**

## 해석 — 물리 곡선 vs odom 오차 (판별 단서)
- odom(시작 정렬·dead-reckoning)이 **LiDAR가 추적하는 MCL로부터 ≥18 cm, ≥15°(20 m)** 벌어짐.
- **판별 논리**: 만약 "로봇이 물리적으로 곡선 주행 + odom은 정확"이면 odom belief = 실제 = MCL →
  잔차 ≈ 0이어야 한다. 그러나 위치 잔차가 0.18 m로 **odom belief ≠ 실제** → **odom 자체가
  드리프트(부정확)** 쪽으로 기운다. (LiDAR 위치는 신뢰도 높음 → 이 결론의 근거.)
- 즉 방법 ①에서 본 직진 heading 드리프트는 **단순 물리 곡선이 아니라 odom heading 오차 성분**이
  유력하다. (단, 아래 한계로 확정은 방법 ③.)

## 한계 (경계)
- **MCL 순환성**: MCL은 odom을 모션모델로 씀 → 원시 odom 오차를 **과소평가**. 실제 오차 ≥ 위 수치(하한).
- **MCL yaw 약함 + `cov_yaw=0`**: emcl2가 yaw 공분산을 0으로 보고 → yaw 품질 게이트는 무뎌짐
  (신선도 게이트만 유효). yaw 잔차는 참고, **위치 잔차(0.18 m)가 주 근거**.
- MCL은 독립 GT 아님 → **방법 ③(AprilTag)으로 진짜 크기·물리vs odom 확정 필요.**

## 결론
- **✅ 첫 외부기준 신호**: 방법 ①이 ~0이던 것과 달리, odom이 MCL 대비 **20 m에 ≥18 cm/≥15° 드리프트**.
  스워브 조향 영점(미보정)의 **운영 영향**을 처음 정량화(baseline).
- **✅ 원인 국소화 재확인**: 드리프트는 직진 구간에 실림(snap-back yaw 톱니) — 방법 ① TUM 진단과 일치.
- **⚠️ 물리 곡선보다 odom 오차 쪽으로 기움**(위치 잔차 근거), 단 순환성·MCL 한계로 **방법 ③ 확정 대기**.

## 다음
- **방법 ③(AprilTag GT)** — 독립 진실로 진짜 크기 + 물리 vs odom 확정, 그리고 ④의 과소평가폭 보정.
- 조향 영점 수정 → **동일 조건 재측정**(before/after로 보정 효과 정량화; 이 폴더가 before).
- `square_ccw` — 드리프트 방향 의존성.

## 운영 메모
- 이번엔 repo 디렉토리에서 실행해 결과가 `repo/results`에 생성됨(archive로 이관). 위치 일관 확보.
- snap-back 이벤트 카운트는 A(93)/B(7) 비대칭 — 시리즈 rate·임계 민감 휴리스틱이므로 참고만,
  주 지표는 최대/평균/최종 잔차(A≈B로 일치).
