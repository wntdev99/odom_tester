"""Full-rate 궤적 레코더 — 메시지 원본 주기·타임스탬프로 pose 를 모아 TUM 으로 저장.

바퀴당 체크포인트 CSV(요약)와 별개로, evo(ATE/RPE)·snap-back 분석에 필요한
연속 궤적을 남긴다. 세 방법이 공유한다(odom_compare, odom_mcl, 이후 방법②③).

TUM 포맷(evo 표준): `timestamp tx ty tz qx qy qz qw` (공백 구분, 한 줄 한 샘플).
2D 이므로 tz=0, 쿼터니언은 yaw 만(z축): qz=sin(yaw/2), qw=cos(yaw/2).

스레드 안전: 구독 콜백(여러 스레드)에서 add() 하고 실행 스레드에서 start/stop/write
하므로 락으로 보호한다.
"""
import math
import os
import threading


def stamp_to_sec(stamp):
    """ROS 메시지 header.stamp(builtin_interfaces/Time) → float 초."""
    return stamp.sec + stamp.nanosec * 1e-9


class TrajectoryRecorder:
    def __init__(self):
        self._lock = threading.Lock()
        self._active = False
        self._series = {}   # name -> list[(t, x, y, yaw)]

    def start(self, names):
        """기록 시작 — 주어진 시리즈 이름들의 버퍼를 비우고 활성화."""
        with self._lock:
            for n in names:
                self._series[n] = []
            self._active = True

    def stop(self):
        with self._lock:
            self._active = False

    def add(self, name, t, x, y, yaw):
        """활성 상태면 한 샘플 추가(구독 콜백에서 호출)."""
        with self._lock:
            if not self._active:
                return
            buf = self._series.get(name)
            if buf is not None:
                buf.append((t, x, y, yaw))

    def count(self, name):
        with self._lock:
            return len(self._series.get(name, []))

    def write_tum(self, name, path):
        """시리즈를 TUM 파일로 저장. 반환: (경로, 샘플수). 빈 시리즈면 (None, 0)."""
        with self._lock:
            samples = list(self._series.get(name, []))
        if not samples:
            return None, 0
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w') as f:
            for t, x, y, yaw in samples:
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                f.write(f'{t:.9f} {x:.6f} {y:.6f} 0.0 0.0 0.0 {qz:.9f} {qw:.9f}\n')
        return path, len(samples)
