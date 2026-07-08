"""2D pose 유틸리티 — 오도메트리(nav_msgs/Odometry)에서 (x, y, yaw) 추출·재영점.

외부 의존성 없이 표준 수식만 사용한다(tf_transformations 불필요).
"""
import math
from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → yaw(z축 회전, rad)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose2d_from_odom(msg) -> Pose2D:
    """nav_msgs/Odometry → Pose2D(x, y, yaw)."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


def wrap_angle(a: float) -> float:
    """각도를 [-pi, pi]로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


def rezero(pose: Pose2D, start: Pose2D) -> Pose2D:
    """시작 pose 기준으로 재영점 — 위치는 시작값을 빼고, yaw는 wrap 차이.

    같은 프레임 쌍(swerve/fused, 둘 다 odom)에는 회전을 적용하지 않는다.
    공통 odom 축을 유지한 채 시작 위치·heading만 제거한다(method-1 문서 §7).
    """
    return Pose2D(
        x=pose.x - start.x,
        y=pose.y - start.y,
        yaw=wrap_angle(pose.yaw - start.yaw),
    )


def planar_distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
