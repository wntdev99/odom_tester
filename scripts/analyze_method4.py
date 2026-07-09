#!/usr/bin/env python3
"""방법 ④(odom_mcl) MCL 잔차 분석·시각화 스크립트.

방법 ④ 노드가 기록한 CSV(검사 대상 odom A/B 와 MCL(mcl_pose)의 raw pose,
공분산, 시작정렬 후 잔차 err_a/err_b, 품질 플래그)를 읽어 바퀴(loop) 경계
체크포인트 기준으로 잔차를 시각화한다.

주의(docs/method-4 §0): 여기서 말하는 잔차(residual)는 '절대 정확도'가 아니라
      'MCL 대비 운영 위치오차'다. MCL 은 odom 을 모션모델로 쓰므로(순환성)
      원시 odom 오차를 과소평가한다. 절대 판정은 독립 GT(방법 ③).

품질 게이팅: quality_ok==0 인 체크포인트는 MCL 공분산/신선도 미달 구간이라
      잔차를 신뢰할 수 없다. 플롯에 별도 표시하고, 요약 통계는 통과분만으로 낸다.

snap-back 주: 스캔 보정 사이 누적 odom 오차(급감폭)는 고주파 로깅이 있어야
      정확히 보인다. 이 체크포인트 CSV(바퀴당 1점)로는 '바퀴당 잔차 증분'만
      근사로 제공한다(진짜 snap-back 추출은 노드의 full-rate 로깅이 선행 필요).

ROS 의존성 없는 순수 파이썬(matplotlib) 스크립트.

CSV 형식:
    loop,ax,ay,ayaw,bx,by,byaw,gx,gy,gyaw,cov_pos,cov_yaw,quality_ok,
    err_a_pos,err_a_yaw,err_b_pos,err_b_yaw
    - a* = 검사대상 A(/swerve_controller/odom), b* = B(/fused_odom)
    - g* = MCL(mcl_pose), (x,y)[m], yaw[rad]
    - cov_pos[m^2], cov_yaw[rad^2] (게이팅 없는 타입이면 nan)
    - quality_ok: 1=신뢰, 0=공분산/신선도 미달
    - err_*_pos[m], err_*_yaw[rad]: 시작정렬(SE(2)) 후 각 odom 의 MCL 대비 잔차

사용법:
    python3 analyze_method4.py --input results
    python3 analyze_method4.py --input results/m4_square_cw_rep0_XXXX.csv
    python3 analyze_method4.py --input results --output results/plots
"""

import argparse
import csv
import glob
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")  # 화면(display) 없이 PNG 로만 저장
import matplotlib.pyplot as plt
from matplotlib import font_manager


def setup_korean_font():
    """한글이 깨지지 않도록 한국어 지원 폰트를 자동 선택해 설정한다."""
    preferred = ["Noto Sans CJK KR", "Noto Sans CJK JP", "Noto Sans KR",
                 "NanumGothic", "Malgun Gothic", "AppleGothic", "UnDotum"]
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in installed:
            plt.rcParams["font.family"] = name
            break
    else:
        print("[경고] 한국어 폰트를 찾지 못함 — 제목 한글이 깨질 수 있음",
              file=sys.stderr)
    plt.rcParams["axes.unicode_minus"] = False


# CSV 컬럼 순서(방법④ 노드가 기록하는 헤더)
COLUMNS = ["loop", "ax", "ay", "ayaw", "bx", "by", "byaw",
           "gx", "gy", "gyaw", "cov_pos", "cov_yaw", "quality_ok",
           "err_a_pos", "err_a_yaw", "err_b_pos", "err_b_yaw"]


def load_csv(path):
    """CSV 한 개를 읽어 컬럼별 리스트(float) dict 로 반환."""
    data = {c: [] for c in COLUMNS}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for c in COLUMNS:
                data[c].append(float(row[c]))
    data["loop"] = [int(v) for v in data["loop"]]
    data["quality_ok"] = [int(v) for v in data["quality_ok"]]
    return data


def rad2deg_list(values):
    return [math.degrees(v) for v in values]


def linear_slope(xs, ys):
    """xs, ys 최소제곱 1차 기울기(바퀴당 증가율). 2점 미만이면 None."""
    n = len(xs)
    if n < 2:
        return None
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    denom = sum((x - mean_x) ** 2 for x in xs)
    if denom == 0:
        return None
    num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    return num / denom


# --- SE(2) (시각화용 XY 재정렬 — 노드 pose_utils 와 동일 규약, standalone 재구현) ---
def _compose(a, b):
    """SE(2) 합성 a∘b. pose = (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def _inverse(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return (-(c * p[0] + s * p[1]), -(-s * p[0] + c * p[1]), -p[2])


def _align_traj(xs, ys, yaws, ref0):
    """궤적(odom 프레임)을 ref0(첫 체크포인트의 MCL pose) 기준으로 map 프레임에 정렬.

    노드의 시작정렬은 조건 시작 t0 를 기준하나, CSV 엔 t0 pose 가 없으므로
    시각화용으로 '첫 체크포인트'를 공통 앵커로 쓴다(수치 잔차는 err_* 컬럼이 authoritative)."""
    src0 = (xs[0], ys[0], yaws[0])
    T = _compose(ref0, _inverse(src0))
    ox, oy = [], []
    for x, y, yaw in zip(xs, ys, yaws):
        p = _compose(T, (x, y, yaw))
        ox.append(p[0])
        oy.append(p[1])
    return ox, oy


def _split_quality(values, quality):
    """(good_idx_vals, bad_idx_vals) — quality_ok 로 인덱스 분리(플롯 강조용)."""
    good = [(i, v) for i, (v, q) in enumerate(zip(values, quality)) if q == 1]
    bad = [(i, v) for i, (v, q) in enumerate(zip(values, quality)) if q == 0]
    return good, bad


def _mark_low_quality(ax, xs, values, quality):
    """저신뢰(quality_ok==0) 체크포인트를 빨간 X 로 강조."""
    _, bad = _split_quality(values, quality)
    if bad:
        bx = [xs[i] for i, _ in bad]
        bv = [v for _, v in bad]
        ax.scatter(bx, bv, color="red", marker="x", zorder=6, s=70,
                   label="low-confidence (게이팅 탈락)")


def plot_single(name, data, outdir):
    """조건(CSV) 1개: 잔차 라인(A/B, pos/yaw) + map 프레임 XY 오버레이 PNG."""
    saved = []
    loops = data["loop"]
    q = data["quality_ok"]

    # (1) 잔차 곡선: err_a/err_b, pos·yaw
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    ax1.plot(loops, data["err_a_pos"], marker="o", color="tab:green",
             label="A: swerve")
    ax1.plot(loops, data["err_b_pos"], marker="s", color="tab:purple",
             label="B: fused")
    _mark_low_quality(ax1, loops, data["err_b_pos"], q)
    ax1.set_ylabel("MCL 잔차 pos [m]")
    ax1.set_title(f"{name} — MCL 대비 잔차(운영 위치오차, 절대정확도 아님)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)

    ax2.plot(loops, rad2deg_list(data["err_a_yaw"]), marker="o",
             color="tab:green", label="A: swerve")
    ax2.plot(loops, rad2deg_list(data["err_b_yaw"]), marker="s",
             color="tab:purple", label="B: fused")
    ax2.set_ylabel("MCL 잔차 yaw [deg]")
    ax2.set_xlabel("loop (바퀴 체크포인트)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    p1 = os.path.join(outdir, f"{name}_residual.png")
    fig.savefig(p1, dpi=120)
    plt.close(fig)
    saved.append(p1)

    # (2) XY 오버레이(map 프레임): MCL + 정렬된 odom A/B
    fig, ax = plt.subplots(figsize=(7, 7))
    ref0 = (data["gx"][0], data["gy"][0], data["gyaw"][0])
    ax_x, ax_y = _align_traj(data["ax"], data["ay"], data["ayaw"], ref0)
    bx_x, bx_y = _align_traj(data["bx"], data["by"], data["byaw"], ref0)
    ax.plot(data["gx"], data["gy"], marker="D", linestyle="-",
            color="tab:orange", label="MCL (mcl_pose)")
    ax.plot(ax_x, ax_y, marker="o", linestyle="--",
            color="tab:green", label="swerve (A, 정렬)")
    ax.plot(bx_x, bx_y, marker="x", linestyle=":",
            color="tab:purple", label="fused (B, 정렬)")
    ax.scatter([data["gx"][0]], [data["gy"][0]], color="black",
               zorder=5, s=60, label="start (앵커)")
    ax.set_xlabel("x [m] (map)")
    ax.set_ylabel("y [m] (map)")
    ax.set_title(f"{name} — XY 오버레이 (map 프레임, 첫 체크포인트 정렬)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    fig.tight_layout()
    p2 = os.path.join(outdir, f"{name}_xy.png")
    fig.savefig(p2, dpi=120)
    plt.close(fig)
    saved.append(p2)

    return saved


def plot_comparison(datasets, outdir):
    """여러 조건을 겹쳐 그린 잔차 비교 플롯(대표값 B=fused)."""
    if len(datasets) < 2:
        return None

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 9), sharex=True)
    cmap = plt.get_cmap("tab10")
    for i, (name, data) in enumerate(datasets):
        color = cmap(i % 10)
        ax1.plot(data["loop"], data["err_b_pos"], marker="o",
                 color=color, label=name)
        ax2.plot(data["loop"], rad2deg_list(data["err_b_yaw"]), marker="s",
                 color=color, label=name)

    ax1.set_ylabel("MCL 잔차 pos [m]")
    ax1.set_title("조건별 MCL 잔차 비교 (B=fused 기준)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)

    ax2.set_ylabel("MCL 잔차 yaw [deg]")
    ax2.set_xlabel("loop (바퀴 체크포인트)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    p = os.path.join(outdir, "comparison_residual.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    return p


def _gated(values, quality):
    """quality_ok==1 인 값만 추린다(신뢰 통계용)."""
    return [v for v, q in zip(values, quality) if q == 1]


def print_summary(datasets):
    """조건별 최종/최대 잔차·바퀴당 증가율·품질 요약(콘솔). 통계는 게이팅 통과분만."""
    print("\n" + "=" * 74)
    print("방법 ④ MCL 잔차 요약 (운영 위치오차 — 절대 정확도 아님, 순환성 주의)")
    print("=" * 74)
    for name, data in datasets:
        loops = data["loop"]
        q = data["quality_ok"]
        n = len(loops)
        n_ok = sum(q)

        print(f"\n[{name}]  체크포인트 {n}개 (게이팅 통과 {n_ok}개)")
        if n_ok == 0:
            print("  ⚠ 신뢰 가능한 체크포인트 없음 — MCL 공분산/신선도 전부 미달. 통계 생략.")
            continue
        if n_ok < n:
            print(f"  ⚠ 저신뢰 {n - n_ok}개 제외하고 통계 산출.")

        for tag, kpos, kyaw in [("A swerve", "err_a_pos", "err_a_yaw"),
                                ("B fused ", "err_b_pos", "err_b_yaw")]:
            loops_ok = _gated(loops, q)
            pos_ok = _gated(data[kpos], q)
            yaw_ok_deg = rad2deg_list(_gated(data[kyaw], q))
            final_pos = pos_ok[-1]
            final_yaw = yaw_ok_deg[-1]
            max_pos = max(pos_ok)
            max_yaw = max(yaw_ok_deg, key=abs)
            slope_pos = linear_slope(loops_ok, pos_ok)
            slope_yaw = linear_slope(loops_ok, yaw_ok_deg)
            sp = f"{slope_pos:.6f} m/loop" if slope_pos is not None else "N/A"
            sy = f"{slope_yaw:.4f} deg/loop" if slope_yaw is not None else "N/A"
            print(f"  [{tag}] 최종 pos={final_pos:.6f} m  yaw={final_yaw:.4f} deg"
                  f" | 최대 pos={max_pos:.6f} m  yaw={max_yaw:.4f} deg"
                  f" | 증가율 {sp}, {sy}")
    print("=" * 74)
    print("주: 잔차 증가율은 원시 odom 드리프트의 '하한'(순환성 탓 과소평가).")
    print("    진짜 snap-back·절대오차는 full-rate 로깅·독립 GT(방법 ③)가 필요.")
    print("=" * 74)


def collect_inputs(input_path):
    """입력 경로 → CSV 파일 리스트. 디렉토리면 m4_*.csv 우선, 없으면 *.csv."""
    input_path = os.path.expanduser(input_path)
    if os.path.isdir(input_path):
        files = sorted(glob.glob(os.path.join(input_path, "m4_*.csv")))
        if not files:
            files = sorted(glob.glob(os.path.join(input_path, "*.csv")))
        return files, input_path
    elif os.path.isfile(input_path):
        return [input_path], os.path.dirname(input_path)
    else:
        return [], input_path


def main():
    parser = argparse.ArgumentParser(
        description="방법 ④(odom_mcl) CSV MCL 잔차 분석·시각화")
    parser.add_argument("--input", default="results",
                        help="CSV 파일 또는 디렉토리 (기본: results, CWD 기준)")
    parser.add_argument("--output", default=None,
                        help="PNG 저장 디렉토리 (기본: 입력 옆 plots/)")
    args = parser.parse_args()

    setup_korean_font()

    files, base_dir = collect_inputs(args.input)
    if not files:
        print(f"[오류] CSV 를 찾지 못함: {args.input}", file=sys.stderr)
        sys.exit(1)

    outdir = os.path.expanduser(args.output) if args.output \
        else os.path.join(base_dir, "plots")
    os.makedirs(outdir, exist_ok=True)

    print(f"입력 CSV {len(files)}개 처리, 출력 → {outdir}")

    datasets = []
    saved_all = []
    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        try:
            data = load_csv(path)
        except Exception as e:
            print(f"[경고] {path} 읽기 실패: {e}", file=sys.stderr)
            continue
        if not data["loop"]:
            print(f"[경고] {name}: 데이터 행 없음, 건너뜀", file=sys.stderr)
            continue
        datasets.append((name, data))
        for p in plot_single(name, data, outdir):
            saved_all.append(p)
            print(f"  저장: {p}")

    if not datasets:
        print("[오류] 유효한 데이터가 없음", file=sys.stderr)
        sys.exit(1)

    comp = plot_comparison(datasets, outdir)
    if comp:
        saved_all.append(comp)
        print(f"  저장(비교): {comp}")

    print_summary(datasets)

    print(f"\n총 {len(saved_all)}개 PNG 생성 완료.")


if __name__ == "__main__":
    main()
