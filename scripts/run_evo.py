#!/usr/bin/env python3
"""노드가 남긴 TUM 궤적을 evo(evo_ape/evo_rpe)로 비교하는 래퍼.

노드는 실행마다 시리즈별 TUM 파일을 남긴다(문서 §8):
    방법④: m4_<cond>_rep<r>_<stamp>_odom_a.tum / _odom_b.tum / _mcl.tum
    방법①: <cond>_rep<r>_<stamp>_swerve.tum / _fused.tum
이 스크립트는 같은 실행(공통 prefix)의 TUM 들을 묶어, 기준(reference)을 정하고
나머지를 estimate 로 evo 에 넣는다.

기준(reference) 선택:
    - mcl 시리즈가 있으면(방법④) → mcl 이 기준, odom_a/odom_b 를 평가
    - 없으면(방법①) → fused 가 기준, swerve 를 평가 (상호 비교 — 절대 진실 아님)

정렬(align):
    - origin(기본): `--align_origin` — 첫 pose 정렬. 노드의 시작정렬(문서 §7)과 같은 관점.
    - full: `-a` — Umeyama 전체 정렬(궤적 전체 최적 맞춤).
    - scale 보정(`-s`)은 오도메트리 스케일오차를 지워버리므로 기본 사용 안 함.

evo 미설치 시: `pip install evo` (ROS 비의존). 설치 없이 명령만 보려면 --dry-run.

사용법:
    python3 run_evo.py --input results                 # results/의 모든 TUM 세트
    python3 run_evo.py --input results --mode ape      # ATE만
    python3 run_evo.py --input results --align full    # 전체 정렬
    python3 run_evo.py --input results --dry-run       # 명령만 출력(실행 안 함)
"""

import argparse
import glob
import os
import shutil
import subprocess
import sys
from collections import defaultdict

# 노드가 쓰는 시리즈 이름(접미사). 'odom_a' 처럼 밑줄 포함이 있어 단순 rsplit 불가 →
# 알려진 이름으로 접미사 매칭한다. 기준 우선순위: 앞일수록 우선.
SERIES_NAMES = ["odom_a", "odom_b", "mcl", "swerve", "fused"]
REF_PRIORITY = ["mcl", "fused"]


def split_series(base):
    """파일 basename(확장자 제거)에서 (prefix, series) 분리. 알 수 없으면 (None, None)."""
    for s in SERIES_NAMES:
        if base.endswith("_" + s):
            return base[:-(len(s) + 1)], s
    if "_" in base:  # 미등록 시리즈 대비 fallback
        return base.rsplit("_", 1)
    return None, None


def find_tum_groups(input_dir):
    """*.tum 을 공통 prefix(끝의 _<series> 제거)로 묶는다.

    반환: {prefix: {series: path}}."""
    groups = defaultdict(dict)
    for path in sorted(glob.glob(os.path.join(input_dir, "*.tum"))):
        base = os.path.basename(path)[:-4]  # .tum 제거
        prefix, series = split_series(base)
        if prefix is None:
            continue
        groups[prefix][series] = path
    return groups


def pick_reference(series_map):
    """시리즈 dict 에서 기준을 고른다. 우선순위 없으면 None."""
    for ref in REF_PRIORITY:
        if ref in series_map:
            return ref
    return None


def build_commands(prefix, series_map, outdir, mode, align):
    """한 그룹에 대한 evo 명령 리스트 생성. 반환: [(설명, argv), ...]."""
    ref = pick_reference(series_map)
    if ref is None:
        print(f"[건너뜀] {prefix}: 기준 시리즈(mcl|fused) 없음 "
              f"(발견: {sorted(series_map)})", file=sys.stderr)
        return []
    ref_path = series_map[ref]
    estimates = [(s, p) for s, p in sorted(series_map.items()) if s != ref]
    if not estimates:
        print(f"[건너뜀] {prefix}: 평가 대상(estimate) 없음", file=sys.stderr)
        return []

    align_flag = "--align_origin" if align == "origin" else "-a"
    tools = {"ape": "evo_ape", "rpe": "evo_rpe"}
    modes = ["ape", "rpe"] if mode == "both" else [mode]

    cmds = []
    for s, est_path in estimates:
        for m in modes:
            tag = f"{prefix}__{s}_vs_{ref}__{m}"
            zip_path = os.path.join(outdir, tag + ".zip")
            argv = [tools[m], "tum", ref_path, est_path,
                    align_flag, "--save_results", zip_path]
            cmds.append((tag, argv))
    return cmds


def main():
    parser = argparse.ArgumentParser(description="TUM 궤적 evo 비교 래퍼")
    parser.add_argument("--input", default="results",
                        help="TUM 디렉토리 (기본: results)")
    parser.add_argument("--output", default=None,
                        help="evo 결과(.zip) 디렉토리 (기본: <input>/evo)")
    parser.add_argument("--mode", choices=["ape", "rpe", "both"], default="both",
                        help="ape(ATE) | rpe(상대) | both (기본 both)")
    parser.add_argument("--align", choices=["origin", "full"], default="origin",
                        help="origin=--align_origin(기본) | full=-a(Umeyama)")
    parser.add_argument("--dry-run", action="store_true",
                        help="명령만 출력, 실행 안 함")
    args = parser.parse_args()

    input_dir = os.path.expanduser(args.input)
    if not os.path.isdir(input_dir):
        print(f"[오류] 디렉토리 아님: {input_dir}", file=sys.stderr)
        sys.exit(1)

    outdir = os.path.expanduser(args.output) if args.output \
        else os.path.join(input_dir, "evo")

    groups = find_tum_groups(input_dir)
    if not groups:
        print(f"[오류] TUM 파일 없음: {input_dir} "
              "(record_tum=true 로 주행했는지 확인)", file=sys.stderr)
        sys.exit(1)

    all_cmds = []
    for prefix, series_map in sorted(groups.items()):
        all_cmds.extend(build_commands(prefix, series_map, outdir, args.mode, args.align))

    if not all_cmds:
        print("[오류] 실행할 evo 명령 없음", file=sys.stderr)
        sys.exit(1)

    print(f"TUM 세트 {len(groups)}개 → evo 명령 {len(all_cmds)}개 "
          f"(mode={args.mode}, align={args.align})")

    have_evo = shutil.which("evo_ape") is not None
    if args.dry_run or not have_evo:
        if not have_evo and not args.dry_run:
            print("[경고] evo 미설치(`pip install evo`) — 명령만 출력합니다.",
                  file=sys.stderr)
        for tag, argv in all_cmds:
            print(f"# {tag}\n" + " ".join(argv))
        return

    os.makedirs(outdir, exist_ok=True)
    failed = 0
    for tag, argv in all_cmds:
        print(f"\n=== {tag} ===\n{' '.join(argv)}")
        rc = subprocess.run(argv).returncode
        if rc != 0:
            failed += 1
            print(f"[경고] {tag}: evo 리턴코드 {rc}", file=sys.stderr)
    print(f"\n완료: {len(all_cmds) - failed}/{len(all_cmds)} 성공. 결과 → {outdir}")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
