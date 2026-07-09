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

snap-back: 노드의 full-rate TUM(*_odom_a/b/mcl.tum)이 있으면 시간축 잔차를
      복원해 스캔 보정 사이 누적 odom 오차(계단/급변)를 보여준다(compute_snapback).
      체크포인트 CSV(바퀴당 1점)는 누적 잔차 envelope, TUM 은 그 안의 급변을 담당.

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


# =====================================================================
# snap-back 분석 (full-rate TUM 기반)
# ---------------------------------------------------------------------
# 노드가 남긴 연속 궤적 TUM(odom_a/odom_b/mcl)을 읽어, 시작정렬 후
# odom(dead-reckoning) vs MCL 잔차를 '시간 축'으로 복원한다. 체크포인트
# CSV(바퀴당 1점)와 달리, 스캔 보정 사이 잔차 변화(계단/급변 = snap-back)를
# 보여준다. 잔차는 보정 때마다 계단식으로 뛰며, 각 계단 ≈ 그 구간 누적 odom 오차.
# =====================================================================
def _yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def parse_tum(path):
    """TUM(`t x y z qx qy qz qw`) → 시간순 list[(t, x, y, yaw)]."""
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            t, x, y = float(p[0]), float(p[1]), float(p[2])
            qx, qy, qz, qw = float(p[4]), float(p[5]), float(p[6]), float(p[7])
            samples.append((t, x, y, _yaw_from_quat(qx, qy, qz, qw)))
    samples.sort(key=lambda s: s[0])
    return samples


def _nearest(series, times, t):
    """시간 t 에 가장 가까운 샘플과 시간차를 반환. series 는 시간순, times 는 t 리스트."""
    import bisect
    i = bisect.bisect_left(times, t)
    cands = []
    if i < len(series):
        cands.append(series[i])
    if i > 0:
        cands.append(series[i - 1])
    best = min(cands, key=lambda s: abs(s[0] - t))
    return best, abs(best[0] - t)


def compute_snapback(odom, mcl, match_tol=0.1, snap_thresh=0.005):
    """정렬된 odom 궤적 vs MCL 의 시간축 잔차 + snap(급변) 이벤트.

    - 시작정렬: t0 의 odom·MCL 이 같은 물리 자세라 보고 odom→map 변환 T 고정.
    - 각 odom 샘플에서 시간 최근접 MCL 과 잔차(pos[m], yaw[rad]) 계산(시간차 match_tol 이내).
    - snap 이벤트: 연속 잔차 pos 변화 |Δ| > snap_thresh 를 보정 이벤트로 집계.
    반환 dict: t(상대초), res_pos, res_yaw, events(인덱스), stats.
    """
    if len(odom) < 2 or len(mcl) < 2:
        return None
    mcl_times = [s[0] for s in mcl]
    # 시작정렬 T = mcl0 ∘ inverse(odom0), odom0 시각에 최근접한 mcl 을 기준으로.
    o0 = odom[0]
    m0, dt0 = _nearest(mcl, mcl_times, o0[0])
    T = _compose((m0[1], m0[2], m0[3]), _inverse((o0[1], o0[2], o0[3])))

    t0 = o0[0]
    ts, rpos, ryaw = [], [], []
    for (t, x, y, yaw) in odom:
        m, dt = _nearest(mcl, mcl_times, t)
        if dt > match_tol:
            continue
        in_map = _compose(T, (x, y, yaw))
        rp = math.hypot(in_map[0] - m[1], in_map[1] - m[2])
        ry = math.atan2(math.sin(in_map[2] - m[3]), math.cos(in_map[2] - m[3]))
        ts.append(t - t0)
        rpos.append(rp)
        ryaw.append(ry)
    if len(rpos) < 2:
        return None

    # snap 이벤트: 잔차 pos 의 큰 연속 변화(보정으로 인한 계단/급변)
    events, jumps = [], []
    for i in range(1, len(rpos)):
        d = rpos[i] - rpos[i - 1]
        if abs(d) > snap_thresh:
            events.append(i)
            jumps.append(d)

    stats = {
        "n": len(rpos),
        "max_pos": max(rpos),
        "mean_pos": sum(rpos) / len(rpos),
        "final_pos": rpos[-1],
        "max_yaw_deg": math.degrees(max(ryaw, key=abs)),
        "n_events": len(events),
        "mean_jump": (sum(abs(j) for j in jumps) / len(jumps)) if jumps else 0.0,
        "max_jump": (max(abs(j) for j in jumps)) if jumps else 0.0,
    }
    return {"t": ts, "res_pos": rpos, "res_yaw": ryaw,
            "events": events, "stats": stats}


def plot_snapback(run_name, a_path, b_path, mcl_path, outdir, snap_thresh):
    """A/B 각각의 시간축 잔차(snap-back)를 그린다. 반환: (저장경로|None, 요약문|None)."""
    mcl = parse_tum(mcl_path)
    series = {}
    for tag, path in (("A swerve", a_path), ("B fused", b_path)):
        if not os.path.isfile(path):
            continue
        odom = parse_tum(path)
        sb = compute_snapback(odom, mcl, snap_thresh=snap_thresh)
        if sb:
            series[tag] = sb
    if not series:
        return None, None

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    colors = {"A swerve": "tab:green", "B fused": "tab:purple"}
    for tag, sb in series.items():
        ax1.plot(sb["t"], sb["res_pos"], color=colors[tag], lw=1.0, label=tag)
        # snap 이벤트 표시
        ev_t = [sb["t"][i] for i in sb["events"]]
        ev_v = [sb["res_pos"][i] for i in sb["events"]]
        ax1.scatter(ev_t, ev_v, color="red", marker="x", s=25, zorder=5)
        ax2.plot(sb["t"], rad2deg_list(sb["res_yaw"]), color=colors[tag],
                 lw=1.0, label=tag)
    ax1.set_ylabel("MCL 잔차 pos [m]")
    ax1.set_title(f"{run_name} — snap-back (시간축 잔차, 빨간 X=보정 급변)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)
    ax2.set_ylabel("MCL 잔차 yaw [deg]")
    ax2.set_xlabel("시간 [s] (조건 시작 기준)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(outdir, f"{run_name}_snapback.png")
    fig.savefig(path, dpi=120)
    plt.close(fig)

    lines = [f"[{run_name}] snap-back:"]
    for tag, sb in series.items():
        s = sb["stats"]
        lines.append(
            f"  [{tag}] 최대 잔차 pos={s['max_pos']:.4f} m (mean {s['mean_pos']:.4f}), "
            f"최종 {s['final_pos']:.4f} m, yaw|max|={s['max_yaw_deg']:.3f}°; "
            f"보정 이벤트 {s['n_events']}회(평균 급변 {s['mean_jump']*1000:.2f} mm, "
            f"최대 {s['max_jump']*1000:.2f} mm)")
    return path, "\n".join(lines)


def find_tum_sets(base_dir):
    """base_dir 에서 (run_name, odom_a, odom_b, mcl) TUM 세트를 찾는다."""
    sets = []
    for mcl_path in sorted(glob.glob(os.path.join(base_dir, "*_mcl.tum"))):
        prefix = mcl_path[:-len("_mcl.tum")]
        a_path = prefix + "_odom_a.tum"
        b_path = prefix + "_odom_b.tum"
        if os.path.isfile(a_path) or os.path.isfile(b_path):
            sets.append((os.path.basename(prefix), a_path, b_path, mcl_path))
    return sets


def main():
    parser = argparse.ArgumentParser(
        description="방법 ④(odom_mcl) CSV MCL 잔차 분석·시각화")
    parser.add_argument("--input", default="results",
                        help="CSV 파일 또는 디렉토리 (기본: results, CWD 기준)")
    parser.add_argument("--output", default=None,
                        help="PNG 저장 디렉토리 (기본: 입력 옆 plots/)")
    parser.add_argument("--snap-drop", type=float, default=0.005,
                        help="snap 이벤트로 볼 연속 잔차 변화 임계 [m] (기본 0.005)")
    args = parser.parse_args()

    setup_korean_font()

    files, base_dir = collect_inputs(args.input)
    # base_dir 이 유효한지(디렉토리 or 파일의 부모) 확인 — CSV·TUM 둘 다 없을 수 있음
    if not os.path.isdir(base_dir):
        base_dir = base_dir if os.path.isdir(base_dir) else \
            (os.path.dirname(base_dir) or ".")

    outdir = os.path.expanduser(args.output) if args.output \
        else os.path.join(base_dir, "plots")
    os.makedirs(outdir, exist_ok=True)

    datasets = []
    saved_all = []
    if files:
        print(f"입력 CSV {len(files)}개 처리, 출력 → {outdir}")
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

        comp = plot_comparison(datasets, outdir)
        if comp:
            saved_all.append(comp)
            print(f"  저장(비교): {comp}")
        if datasets:
            print_summary(datasets)
    else:
        print(f"[정보] CSV 없음 — TUM snap-back 만 시도 (출력 → {outdir})")

    # --- snap-back: full-rate TUM 세트가 있으면 시간축 잔차 분석 ---
    tum_sets = find_tum_sets(base_dir)
    if tum_sets:
        print(f"\nfull-rate TUM {len(tum_sets)}세트 발견 → snap-back 분석")
        for run_name, a_path, b_path, mcl_path in tum_sets:
            sb_path, sb_summary = plot_snapback(
                run_name, a_path, b_path, mcl_path, outdir, args.snap_drop)
            if sb_path:
                saved_all.append(sb_path)
                print(f"  저장(snap-back): {sb_path}")
                print(sb_summary)
    else:
        if not files:
            print(f"[오류] CSV·TUM 둘 다 없음: {args.input}", file=sys.stderr)
            sys.exit(1)
        print("\n(full-rate TUM 없음 — snap-back 생략. 방법④ 노드가 *_odom_a/b/mcl.tum 생성)")

    print(f"\n총 {len(saved_all)}개 PNG 생성 완료.")


if __name__ == "__main__":
    main()
