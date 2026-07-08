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
    python3 analyze_method1.py --input ~/odom_tests
    python3 analyze_method1.py --input ~/odom_tests/square_cw_rep0_XXXX.csv
    python3 analyze_method1.py --input ~/odom_tests --output ~/odom_tests/plots

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


def print_summary(datasets):
    """조건별 최종 drift 와 바퀴당 증가율(선형 기울기) 콘솔 요약."""
    print("\n" + "=" * 72)
    print("방법 ① 드리프트 요약 (swerve↔fused 상호 불일치)")
    print("=" * 72)
    for name, data in datasets:
        loops = data["loop"]
        dpos = data["drift_pos"]
        dyaw_deg = rad2deg_list(data["drift_yaw"])
        n = len(loops)
        final_pos = dpos[-1] if dpos else float("nan")
        final_yaw = dyaw_deg[-1] if dyaw_deg else float("nan")
        slope_pos = linear_slope(loops, dpos)
        slope_yaw = linear_slope(loops, dyaw_deg)

        print(f"\n[{name}]  (체크포인트 {n}개)")
        print(f"  최종 drift_pos : {final_pos:.6f} m")
        print(f"  최종 drift_yaw : {final_yaw:.4f} deg")
        if slope_pos is not None:
            print(f"  바퀴당 증가율(pos) : {slope_pos:.6f} m/loop")
        else:
            print("  바퀴당 증가율(pos) : N/A (데이터 부족)")
        if slope_yaw is not None:
            print(f"  바퀴당 증가율(yaw) : {slope_yaw:.4f} deg/loop")
        else:
            print("  바퀴당 증가율(yaw) : N/A (데이터 부족)")
    print("=" * 72)


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
    parser.add_argument("--input", default="~/odom_tests",
                        help="CSV 파일 또는 디렉토리 (기본: ~/odom_tests)")
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
