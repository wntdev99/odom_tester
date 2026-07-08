"""모션 프리미티브 — cmd_vel(속도)로 목표 거리/각도만큼 이동한다.

cmd_vel은 속도 명령이므로, "L m 이동"은 여기서 odom 피드백으로 구현한다:
시작 pose를 기록하고 목표에 닿을 때까지 속도를 주기적으로 발행하다가 0을 발행해 멈춘다.
세 방법(①②③)이 공유하는 코어다.
"""
import math
import time
from dataclasses import dataclass

from .pose_utils import Pose2D, wrap_angle


class SegmentAborted(Exception):
    """취소(action cancel) 또는 상위 중단 요청."""


class SegmentGuardTripped(Exception):
    """안전 가드(시간/거리 초과) 발동."""


@dataclass
class MotionParams:
    v_lin: float = 0.15          # [m/s]
    v_ang: float = 0.3           # [rad/s]
    publish_rate: float = 30.0   # [Hz] cmd_vel timeout(0.5s) 대비 주기 발행
    settle_time: float = 1.0     # [s]
    reach_tol_lin: float = 0.02  # [m]
    reach_tol_ang: float = 0.02  # [rad]
    slowdown_lin: float = 0.5    # 감속 시작 남은거리 [m]
    slowdown_ang: float = 0.3    # 감속 시작 남은각도 [rad]
    v_lin_min: float = 0.03      # [m/s]
    v_ang_min: float = 0.05      # [rad/s]
    max_seg_time: float = 30.0   # [s] 구간 타임아웃
    max_seg_dist: float = 1.5    # [m] 폭주 방지


class MotionPrimitives:
    def __init__(self, logger, publish_fn, get_pose_fn, params: MotionParams,
                 abort_fn=None, sleep_fn=None):
        self._log = logger
        self._publish = publish_fn          # publish(vx, vy, wz)
        self._get_pose = get_pose_fn        # () -> Pose2D (feedback source)
        self._p = params
        self._abort = abort_fn or (lambda: False)
        self._sleep = sleep_fn or time.sleep

    # --- public primitives ---
    def drive(self, distance: float):
        """전방(+x, body) 직진. distance 부호로 전/후."""
        self._translate(distance, axis='x')

    def strafe(self, distance: float):
        """측면(+y, body) 이동. distance 부호로 좌/우."""
        self._translate(distance, axis='y')

    def rotate(self, angle: float):
        """제자리 회전 angle[rad]. 다회전(|angle|>pi) 지원."""
        self._rotate(angle)

    def stop(self):
        self._publish(0.0, 0.0, 0.0)

    def settle(self):
        """정지 명령을 settle_time 동안 유지(관성 제거)."""
        self._hold_zero(self._p.settle_time)

    # --- internal ---
    def _translate(self, distance: float, axis: str):
        p = self._p
        start = self._get_pose()
        target = abs(distance)
        sign = 1.0 if distance >= 0.0 else -1.0
        period = 1.0 / p.publish_rate
        t0 = time.monotonic()
        while True:
            self._check_abort()
            cur = self._get_pose()
            traveled = math.hypot(cur.x - start.x, cur.y - start.y)
            remaining = target - traveled
            if remaining <= p.reach_tol_lin:
                break
            self._check_guards(t0, traveled)
            v = self._ramp(p.v_lin, p.v_lin_min, remaining, p.slowdown_lin)
            if axis == 'x':
                self._publish(sign * v, 0.0, 0.0)
            else:
                self._publish(0.0, sign * v, 0.0)
            self._sleep(period)
        self.stop()

    def _rotate(self, angle: float):
        p = self._p
        target = abs(angle)
        sign = 1.0 if angle >= 0.0 else -1.0
        period = 1.0 / p.publish_rate
        last = self._get_pose().yaw
        accumulated = 0.0
        t0 = time.monotonic()
        while True:
            self._check_abort()
            cur_yaw = self._get_pose().yaw
            accumulated += abs(wrap_angle(cur_yaw - last))
            last = cur_yaw
            remaining = target - accumulated
            if remaining <= p.reach_tol_ang:
                break
            if time.monotonic() - t0 > p.max_seg_time:
                self.stop()
                raise SegmentGuardTripped('rotate: max_seg_time 초과')
            w = self._ramp(p.v_ang, p.v_ang_min, remaining, p.slowdown_ang)
            self._publish(0.0, 0.0, sign * w)
            self._sleep(period)
        self.stop()

    def _ramp(self, v_nom, v_min, remaining, slowdown):
        """목표 근처 선형 감속(P). 오버슈트 억제."""
        if remaining >= slowdown:
            return v_nom
        return max(v_min, v_nom * (remaining / slowdown))

    def _hold_zero(self, duration: float):
        period = 1.0 / self._p.publish_rate
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            self._check_abort()
            self._publish(0.0, 0.0, 0.0)
            self._sleep(period)

    def _check_abort(self):
        if self._abort():
            self.stop()
            raise SegmentAborted()

    def _check_guards(self, t0, traveled):
        if time.monotonic() - t0 > self._p.max_seg_time:
            self.stop()
            raise SegmentGuardTripped('translate: max_seg_time 초과')
        if traveled > self._p.max_seg_dist:
            self.stop()
            raise SegmentGuardTripped('translate: max_seg_dist 초과')
