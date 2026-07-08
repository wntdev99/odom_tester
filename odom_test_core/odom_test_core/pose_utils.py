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


def pose2d_from_pose(pos, ori) -> Pose2D:
    """geometry_msgs Point + Quaternion → Pose2D.

    PoseStamped/PoseWithCovarianceStamped 등 Odometry가 아닌 메시지용
    (mcl_pose 처럼 타입이 다를 수 있는 GT 소스, method-4 문서 §1).
    """
    return Pose2D(pos.x, pos.y, yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """SE(2) 합성 a ∘ b — a 프레임 기준으로 b를 얹는다."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(
        x=a.x + c * b.x - s * b.y,
        y=a.y + s * b.x + c * b.y,
        yaw=wrap_angle(a.yaw + b.yaw),
    )


def inverse(p: Pose2D) -> Pose2D:
    """SE(2) 역변환 — compose(p, inverse(p)) == identity."""
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return Pose2D(
        x=-(c * p.x + s * p.y),
        y=-(-s * p.x + c * p.y),
        yaw=-p.yaw,
    )


def align_transform(ref_start: Pose2D, src_start: Pose2D) -> Pose2D:
    """t0에 두 pose가 같은 물리 자세라고 보고 src→ref 강체 변환을 구한다.

    프레임이 다른 두 궤적(예: map의 mcl_pose vs odom의 odometry)을 시작 시점에
    정렬한다(method-4 문서 §7, evo의 원점정렬 -s 와 동일). 반환값 T로
    `compose(T, src(t))` 하면 src 궤적이 ref 프레임으로 옮겨진다.
    """
    return compose(ref_start, inverse(src_start))
