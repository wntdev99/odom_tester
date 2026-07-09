#!/usr/bin/env python3
"""방법 ①(odom_compare) 드리프트 분석·시각화 스크립트.

방법 ① 노드가 기록한 CSV(swerve 오도메트리 vs fused 오도메트리의 조건 시작
시점 재영점된 위치·헤딩과, 두 추정기 간 상호 불일치 drift)를 읽어
바퀴(loop) 경계 체크포인트 기준으로 드리프트를 시각화한다.

주의: 여기서 말하는 drift 는 '절대 정확도'가 아니라 두 추정기(swerve↔fused)의
      '상호 불일치(mutual disagreement)'다. CSV 는 연속 궤적이 아니라 바퀴
      경계 체크포인트만 담으므로 XY 플롯은 점+선으로 그린다.

ROS 의존성 없는 순수 파이썬(matplotlib) 스크립트.

CSV 형식:
    loop,ax,ay,ayaw,bx,by,byaw,drift_pos,drift_yaw
    - a* = swerve(/swerve_controller/odom), b* = fused(/fused_odom)
    - (x,y)[m], yaw[rad], drift_pos[m], drift_yaw[rad]

사용법:
    python3 analyze_method1.py --input results
    python3 analyze_method1.py --input results/square_cw_rep0_XXXX.csv
    python3 analyze_method1.py --input results --output results/plots

옵션:
    --input   CSV 파일 또는 디렉토리(디렉토리면 *.csv 전부). 기본 ~/odom_tests
    --output  PNG 저장 디렉토리. 기본은 입력 옆 plots/
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
    """한글이 깨지지 않도록 한국어 지원 폰트를 자동 선택해 설정한다.

    선호 폰트 순서대로 시스템에 설치된 것을 찾아 rcParams 에 적용.
    없으면 경고만 출력하고 진행(플롯 제목 한글이 네모로 보일 수 있음)."""
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
    # 마이너스 부호가 유니코드 문자로 나와 깨지는 것 방지
    plt.rcParams["axes.unicode_minus"] = False


# CSV 컬럼 순서(방법① 노드가 기록하는 헤더)
COLUMNS = ["loop", "ax", "ay", "ayaw", "bx", "by", "byaw", "drift_pos", "drift_yaw"]


def load_csv(path):
    """CSV 한 개를 읽어 컬럼별 리스트(float) dict 로 반환."""
    data = {c: [] for c in COLUMNS}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for c in COLUMNS:
                data[c].append(float(row[c]))
    # loop 은 정수로 취급
    data["loop"] = [int(v) for v in data["loop"]]
    return data


def rad2deg_list(values):
    return [math.degrees(v) for v in values]


def linear_slope(xs, ys):
    """xs, ys 에 대한 최소제곱(least-squares) 1차 기울기(바퀴당 증가율).

    데이터가 1점 이하면 기울기 정의 불가 → None."""
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


def classify(name):
    """CSV 파일명에서 조건 종류 추정: cw | ccw | strafe | unknown.

    'square_ccw' 는 'cw' 를 부분문자열로 포함하므로 ccw 를 먼저 검사한다."""
    low = name.lower()
    if "ccw" in low:
        return "ccw"
    if "cw" in low:
        return "cw"
    if "strafe" in low:
        return "strafe"
    return "unknown"


def compute_nonclosure(data):
    """자기-odom 폐루프 비폐합(핀휠) — 상호 드리프트와 별개.

    CSV 의 (ax,ay,ayaw)/(bx,by,byaw) 는 조건 시작 시점 재영점된 각 추정기의 위치·헤딩.
    정사각형/strafe 경로는 매 loop 완주 시 시작점(0,0,0)으로 돌아와야 하므로,
    원점으로부터의 편차 = 그 추정기가 '자기 믿음 안에서' 경로를 못 닫은 정도(비폐합).
    이는 두 추정기 간 drift(mutual)와 성격이 다르다(자기일관성 vs 상호일치).

    반환: dict(swerve_pos[], swerve_yaw_deg[], fused_pos[], fused_yaw_deg[])."""
    sp = [math.hypot(x, y) for x, y in zip(data["ax"], data["ay"])]
    fp = [math.hypot(x, y) for x, y in zip(data["bx"], data["by"])]
    syaw = rad2deg_list(data["ayaw"])
    fyaw = rad2deg_list(data["byaw"])
    return {"swerve_pos": sp, "swerve_yaw_deg": syaw,
            "fused_pos": fp, "fused_yaw_deg": fyaw}


def plot_nonclosure(name, data, outdir):
    """자기-odom 비폐합(핀휠)을 swerve·fused 각각 pos·yaw 로 시각화."""
    loops = data["loop"]
    nc = compute_nonclosure(data)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    ax1.plot(loops, nc["swerve_pos"], marker="o", color="tab:green",
             label="swerve (a)")
    ax1.plot(loops, nc["fused_pos"], marker="s", color="tab:purple",
             label="fused (b)")
    ax1.set_ylabel("비폐합 pos [m] (원점 이탈)")
    ax1.set_title(f"{name} — 자기 odom 폐루프 비폐합(핀휠), 상호 드리프트와 별개")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)

    ax2.plot(loops, nc["swerve_yaw_deg"], marker="o", color="tab:green",
             label="swerve (a)")
    ax2.plot(loops, nc["fused_yaw_deg"], marker="s", color="tab:purple",
             label="fused (b)")
    ax2.set_ylabel("비폐합 yaw [deg]")
    ax2.set_xlabel("loop (바퀴 체크포인트)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    p = os.path.join(outdir, f"{name}_nonclosure.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    return p


def plot_single(name, data, outdir):
    """조건(CSV) 1개에 대한 drift 라인 플롯 + XY 오버레이 PNG 생성.

    반환: 저장된 PNG 경로 리스트."""
    saved = []
    loops = data["loop"]
    drift_pos = data["drift_pos"]
    drift_yaw_deg = rad2deg_list(data["drift_yaw"])  # rad → deg

    # (1) drift_pos / drift_yaw vs loop
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    ax1.plot(loops, drift_pos, marker="o", color="tab:red")
    ax1.set_ylabel("drift_pos [m]")
    ax1.set_title(f"{name} — 상호 불일치 드리프트(swerve↔fused)")
    ax1.grid(True, alpha=0.3)

    ax2.plot(loops, drift_yaw_deg, marker="s", color="tab:blue")
    ax2.set_ylabel("drift_yaw [deg]")
    ax2.set_xlabel("loop (바퀴 체크포인트)")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    p1 = os.path.join(outdir, f"{name}_drift.png")
    fig.savefig(p1, dpi=120)
    plt.close(fig)
    saved.append(p1)

    # (2) XY 오버레이: swerve(a) vs fused(b), 체크포인트 점+선
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(data["ax"], data["ay"], marker="o", linestyle="-",
            color="tab:green", label="swerve (a)")
    ax.plot(data["bx"], data["by"], marker="x", linestyle="--",
            color="tab:purple", label="fused (b)")
    # 시작점 강조
    if data["ax"]:
        ax.scatter([data["ax"][0]], [data["ay"][0]], color="black",
                   zorder=5, s=60, label="start")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{name} — XY 궤적 오버레이 (체크포인트)")
    ax.axis("equal")  # 축 등비율
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.tight_layout()
    p2 = os.path.join(outdir, f"{name}_xy.png")
    fig.savefig(p2, dpi=120)
    plt.close(fig)
    saved.append(p2)

    return saved


def plot_comparison(datasets, outdir):
    """여러 조건(CSV)을 겹쳐 그린 drift_pos·drift_yaw 비교 플롯.

    datasets: [(name, data), ...]. 반환: 저장 경로(없으면 None)."""
    if len(datasets) < 2:
        return None

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 9), sharex=True)
    cmap = plt.get_cmap("tab10")
    for i, (name, data) in enumerate(datasets):
        color = cmap(i % 10)
        ax1.plot(data["loop"], data["drift_pos"], marker="o",
                 color=color, label=name)
        ax2.plot(data["loop"], rad2deg_list(data["drift_yaw"]), marker="s",
                 color=color, label=name)

    ax1.set_ylabel("drift_pos [m]")
    ax1.set_title("조건별 드리프트 비교 (swerve↔fused 상호 불일치)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)

    ax2.set_ylabel("drift_yaw [deg]")
    ax2.set_xlabel("loop (바퀴 체크포인트)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    p = os.path.join(outdir, "comparison_drift.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    return p


def summarize_one(name, data):
    """조건 1개의 통계 dict 계산(콘솔·리포트 공용)."""
    loops = data["loop"]
    dpos = data["drift_pos"]
    dyaw_deg = rad2deg_list(data["drift_yaw"])
    nc = compute_nonclosure(data)
    return {
        "name": name,
        "kind": classify(name),
        "n": len(loops),
        "final_drift_pos": dpos[-1] if dpos else float("nan"),
        "final_drift_yaw": dyaw_deg[-1] if dyaw_deg else float("nan"),
        "slope_drift_pos": linear_slope(loops, dpos),
        "slope_drift_yaw": linear_slope(loops, dyaw_deg),
        "final_nc_swerve_pos": nc["swerve_pos"][-1] if nc["swerve_pos"] else float("nan"),
        "final_nc_swerve_yaw": nc["swerve_yaw_deg"][-1] if nc["swerve_yaw_deg"] else float("nan"),
        "final_nc_fused_pos": nc["fused_pos"][-1] if nc["fused_pos"] else float("nan"),
        "final_nc_fused_yaw": nc["fused_yaw_deg"][-1] if nc["fused_yaw_deg"] else float("nan"),
        "slope_nc_swerve_yaw": linear_slope(loops, nc["swerve_yaw_deg"]),
        "slope_nc_swerve_pos": linear_slope(loops, nc["swerve_pos"]),
    }


def _sign_verdict(a, b, eps=1e-6):
    """두 값의 부호 비교 → 해석 문구."""
    if abs(a) < eps or abs(b) < eps:
        return "한쪽≈0 — 판정 보류"
    if (a > 0) != (b > 0):
        return "부호 뒤집힘 → 회전 스케일성 비대칭(과/과소회전) 시사"
    return "부호 동일 → 방향 독립 바이어스 시사"


def cw_ccw_note(stats):
    """CW/CCW 부호 비교(문서 §9). 상호 drift_yaw + 자기 비폐합 yaw 둘 다 비교.

    이 로봇은 상호 drift 가 구조적으로 ≈0(공통 앵커)이라, 실질 신호는 비폐합 yaw 부호다."""
    cw = next((s for s in stats if s["kind"] == "cw"), None)
    ccw = next((s for s in stats if s["kind"] == "ccw"), None)
    if not cw or not ccw:
        return None
    d_cw, d_ccw = cw["final_drift_yaw"], ccw["final_drift_yaw"]
    n_cw, n_ccw = cw["final_nc_swerve_yaw"], ccw["final_nc_swerve_yaw"]
    return (
        f"상호 drift_yaw: CW={d_cw:+.4f}° CCW={d_ccw:+.4f}° → {_sign_verdict(d_cw, d_ccw)}; "
        f"자기 비폐합 yaw(swerve): CW={n_cw:+.3f}° CCW={n_ccw:+.3f}° → {_sign_verdict(n_cw, n_ccw)} "
        "(절대 판정은 방법 ②·③)")


def print_summary(datasets):
    """조건별 상호 드리프트 + 자기 비폐합 + CW/CCW 비교 콘솔 요약."""
    stats = [summarize_one(name, data) for name, data in datasets]
    print("\n" + "=" * 72)
    print("방법 ① 요약 — (A) 상호 불일치(swerve↔fused)  (B) 자기 odom 비폐합(핀휠)")
    print("=" * 72)
    for s in stats:
        sp = f"{s['slope_drift_pos']:.6f} m/loop" if s["slope_drift_pos"] is not None else "N/A"
        sy = f"{s['slope_drift_yaw']:.4f} deg/loop" if s["slope_drift_yaw"] is not None else "N/A"
        ncy = (f"{s['slope_nc_swerve_yaw']:.4f} deg/loop"
               if s["slope_nc_swerve_yaw"] is not None else "N/A")
        print(f"\n[{s['name']}]  (kind={s['kind']}, 체크포인트 {s['n']}개)")
        print(f"  (A) 상호 최종 pos={s['final_drift_pos']:.6f} m  "
              f"yaw={s['final_drift_yaw']:.4f} deg  | 증가율 {sp}, {sy}")
        print(f"  (B) 자기 비폐합  swerve pos={s['final_nc_swerve_pos']:.4f} m "
              f"yaw={s['final_nc_swerve_yaw']:.3f}° | fused pos={s['final_nc_fused_pos']:.4f} m "
              f"yaw={s['final_nc_fused_yaw']:.3f}° | swerve yaw 증가율 {ncy}")
    note = cw_ccw_note(stats)
    if note:
        print(f"\n[CW/CCW] {note}")
    print("=" * 72)
    print("주: (A)는 절대정확도 아님(공통 EKF 앵커). (B) 핀휠이 실제 미회전인지 odom 오차인지는")
    print("    GT(방법 ②·③)로만 구분. 실행 허용오차 아티팩트 가능성 병기.")
    print("=" * 72)
    return stats


def write_report(stats, outdir):
    """조건별 통계를 마크다운 리포트로 저장(리뷰·핸드오프용)."""
    lines = []
    lines.append("# 방법 ① 분석 요약 리포트\n")
    lines.append("> (A) swerve↔fused **상호 불일치**(절대정확도 아님) / "
                 "(B) 자기 odom **폐루프 비폐합(핀휠)**. 성격이 다른 두 지표를 분리한다.\n")
    lines.append("\n## 조건별 통계\n")
    lines.append("| 조건 | kind | 체크포인트 | 상호 pos [m] | 상호 yaw [°] | "
                 "상호 yaw 증가율 [°/loop] | 비폐합 swerve pos [m] | 비폐합 swerve yaw [°] | "
                 "비폐합 fused pos [m] | 비폐합 fused yaw [°] |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for s in stats:
        sy = f"{s['slope_drift_yaw']:.4f}" if s["slope_drift_yaw"] is not None else "N/A"
        lines.append(
            f"| {s['name']} | {s['kind']} | {s['n']} | "
            f"{s['final_drift_pos']:.6f} | {s['final_drift_yaw']:.4f} | {sy} | "
            f"{s['final_nc_swerve_pos']:.4f} | {s['final_nc_swerve_yaw']:.3f} | "
            f"{s['final_nc_fused_pos']:.4f} | {s['final_nc_fused_yaw']:.3f} |")
    note = cw_ccw_note(stats)
    if note:
        lines.append(f"\n## CW/CCW 부호 비교\n\n- {note}\n")
    lines.append("\n## 해석 경계\n")
    lines.append("- (A) 상호 불일치는 공통 EKF yaw 앵커 탓 절대정확도를 못 잡는다(문서 §9).\n")
    lines.append("- (B) 비폐합(핀휠)이 실제 미회전인지 odom 오차인지는 GT(방법 ②·③)로만 구분.\n")
    lines.append("- 타이트 허용오차 재주행으로 실행 아티팩트 성분을 줄여 비교할 것.\n")
    path = os.path.join(outdir, "summary_method1.md")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return path


def collect_inputs(input_path):
    """입력 경로 → CSV 파일 리스트. 디렉토리면 *.csv 전부(정렬)."""
    input_path = os.path.expanduser(input_path)
    if os.path.isdir(input_path):
        files = sorted(glob.glob(os.path.join(input_path, "*.csv")))
        return files, input_path
    elif os.path.isfile(input_path):
        return [input_path], os.path.dirname(input_path)
    else:
        return [], input_path


def main():
    parser = argparse.ArgumentParser(
        description="방법 ①(odom_compare) CSV 드리프트 분석·시각화")
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

    # 출력 디렉토리 결정
    if args.output:
        outdir = os.path.expanduser(args.output)
    else:
        outdir = os.path.join(base_dir, "plots")
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
        pnc = plot_nonclosure(name, data, outdir)
        saved_all.append(pnc)
        print(f"  저장(비폐합): {pnc}")

    if not datasets:
        print("[오류] 유효한 데이터가 없음", file=sys.stderr)
        sys.exit(1)

    comp = plot_comparison(datasets, outdir)
    if comp:
        saved_all.append(comp)
        print(f"  저장(비교): {comp}")

    stats = print_summary(datasets)
    report = write_report(stats, outdir)
    print(f"  저장(리포트): {report}")

    print(f"\n총 {len(saved_all)}개 PNG + 리포트 1개 생성 완료.")


if __name__ == "__main__":
    main()
