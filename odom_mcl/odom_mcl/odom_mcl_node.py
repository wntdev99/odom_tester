"""방법 ④ — MCL(mcl_pose) 기준 오도메트리 잔차 측정 노드.

- info 서비스: `~/list_tests` (ListTests) — 실행 가능한 테스트 목록.
- run 액션:   `~/run_test`   (RunTest)   — 테스트 하나 실행(피드백·취소 지원).
네임스페이스는 launch에서 `/method4` 로 부여한다(문서 §5).

주의(문서 §0): MCL은 odometry를 모션 모델로 쓰므로 독립 GT가 아니다(순환성).
이 방법이 재는 것은 "원시 odom 절대오차"가 아니라 "MCL 대비 잔차 = 운영 위치오차"다.
진짜 절대 정확도는 독립 GT인 방법 ③이 담당한다.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from odom_test_interfaces.srv import ListTests
from odom_test_interfaces.action import RunTest

from odom_test_core.pose_utils import (
    Pose2D, pose2d_from_odom, pose2d_from_pose, wrap_angle,
    compose, align_transform, planar_distance,
)
from odom_test_core.primitives import (
    MotionParams, MotionPrimitives, SegmentAborted, SegmentGuardTripped,
)
from odom_test_core.recorder import TrajectoryRecorder, stamp_to_sec

HALF_PI = math.pi / 2.0

TESTS = {
    'square_cw':     '1x1 정사각형, 코너 90° 시계방향 회전 (누적 +회전)',
    'square_ccw':    '1x1 정사각형, 코너 90° 반시계방향 회전 (누적 -회전)',
    'strafe_square': '1x1 정사각형, 헤딩 고정 병진(+x,+y,-x,-y) — 회전 0',
    'full':          '위 세 조건을 순차 실행',
}

# gt_type → (메시지 클래스, extractor). extractor(msg) -> (Pose2D, cov|None)
# cov = (pos_var[m^2], yaw_var[rad^2]) 또는 공분산이 없으면 None.
# 토픽·타입은 config로 스위칭(문서 §1) — mcl_pose 실제 타입 확인 후 yaml만 수정.


def _extract_pose_with_cov(msg):
    p = msg.pose.pose
    c = msg.pose.covariance          # 6x6 row-major (x,y,z,roll,pitch,yaw)
    cov = (max(c[0], c[7]), c[35])   # 위치 분산(보수적 상한), yaw 분산
    return pose2d_from_pose(p.position, p.orientation), cov


def _extract_odom(msg):
    p = msg.pose.pose
    c = msg.pose.covariance
    return pose2d_from_pose(p.position, p.orientation), (max(c[0], c[7]), c[35])


def _extract_pose(msg):
    return pose2d_from_pose(msg.pose.position, msg.pose.orientation), None


GT_TYPES = {
    'pose_with_cov': (PoseWithCovarianceStamped, _extract_pose_with_cov),
    'odom':          (Odometry, _extract_odom),
    'pose':          (PoseStamped, _extract_pose),
}


class OdomMclNode(Node):
    def __init__(self):
        super().__init__('odom_mcl')
        self._cbg = ReentrantCallbackGroup()

        # --- 파라미터 (config에서 오버라이드) ---
        self.declare_parameter('cmd_vel_topic', '/swerve_controller/cmd_vel')
        self.declare_parameter('odom_a_topic', '/swerve_controller/odom')
        self.declare_parameter('odom_b_topic', '/fused_odom')
        self.declare_parameter('gt_topic', '/mcl_pose')       # ← 마지막에 여기만 수정
        self.declare_parameter('gt_type', 'pose_with_cov')    # pose_with_cov|odom|pose
        self.declare_parameter('feedback_source', 'b')        # 주행 제어용 odom (a|b), MCL 아님
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # 품질 게이팅
        self.declare_parameter('max_cov_pos', 0.05)   # [m^2]
        self.declare_parameter('max_cov_yaw', 0.02)   # [rad^2]
        self.declare_parameter('gt_timeout', 1.0)     # [s]
        # 루틴
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('loops', 5)
        self.declare_parameter('repeats', 1)
        # 속도·타이밍
        self.declare_parameter('v_lin', 0.15)
        self.declare_parameter('v_ang', 0.3)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('settle_time', 1.5)    # MCL 안정화 위해 방법①보다 길게
        self.declare_parameter('reach_tol_lin', 0.001)
        self.declare_parameter('reach_tol_ang', 0.002)
        self.declare_parameter('v_lin_min', 0.01)
        self.declare_parameter('v_ang_min', 0.02)
        self.declare_parameter('slowdown_lin', 0.10)
        self.declare_parameter('slowdown_ang', 0.10)
        self.declare_parameter('max_seg_time', 30.0)
        self.declare_parameter('max_seg_dist', 1.5)
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('record_tum', True)   # full-rate 궤적(evo용) 기록

        gp = self.get_parameter
        self._cmd_topic = gp('cmd_vel_topic').value
        self._fb_src = gp('feedback_source').value
        self._max_cov_pos = gp('max_cov_pos').value
        self._max_cov_yaw = gp('max_cov_yaw').value
        self._gt_timeout = gp('gt_timeout').value
        self._output_dir = os.path.abspath(os.path.expanduser(gp('output_dir').value))
        self._record_tum = gp('record_tum').value
        self._recorder = TrajectoryRecorder()

        # --- I/O ---
        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._pose_a = None
        self._pose_b = None
        self._gt = None
        self._gt_cov = None
        self._gt_stamp = None      # 마지막 gt 수신 시각(monotonic)
        # 조건 시작 시점 정렬 변환(문서 §7). odom(t)→map 프레임으로 옮긴다.
        self._align_a = None
        self._align_b = None

        self.create_subscription(
            Odometry, gp('odom_a_topic').value, self._on_odom_a,
            20, callback_group=self._cbg)
        self.create_subscription(
            Odometry, gp('odom_b_topic').value, self._on_odom_b,
            20, callback_group=self._cbg)

        gt_type = gp('gt_type').value
        if gt_type not in GT_TYPES:
            raise ValueError(
                f'알 수 없는 gt_type: {gt_type} (가능: {list(GT_TYPES.keys())})')
        gt_cls, self._gt_extract = GT_TYPES[gt_type]
        self.create_subscription(
            gt_cls, gp('gt_topic').value, self._on_gt,
            20, callback_group=self._cbg)

        # --- 서비스 / 액션 ---
        self.create_service(
            ListTests, 'list_tests', self._on_list_tests,
            callback_group=self._cbg)
        self._action = ActionServer(
            self, RunTest, 'run_test',
            execute_callback=self._on_run,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self._cbg)

        self.get_logger().info(
            f'odom_mcl 준비됨 (gt_topic={gp("gt_topic").value}, gt_type={gt_type})')

    # ------- 구독 콜백 (pose 갱신 + full-rate 기록) -------
    def _on_odom_a(self, msg):
        p = pose2d_from_odom(msg)
        self._pose_a = p
        self._recorder.add('odom_a', stamp_to_sec(msg.header.stamp), p.x, p.y, p.yaw)

    def _on_odom_b(self, msg):
        p = pose2d_from_odom(msg)
        self._pose_b = p
        self._recorder.add('odom_b', stamp_to_sec(msg.header.stamp), p.x, p.y, p.yaw)

    def _on_gt(self, msg):
        pose, cov = self._gt_extract(msg)
        self._gt = pose
        self._gt_cov = cov
        self._gt_stamp = time.monotonic()
        stamp = msg.header.stamp if hasattr(msg, 'header') else None
        if stamp is not None:
            self._recorder.add('mcl', stamp_to_sec(stamp), pose.x, pose.y, pose.yaw)

    def _gt_quality_ok(self):
        """MCL 신뢰 구간인지: 최신 수신 + 공분산 임계 이하."""
        if self._gt_stamp is None:
            return False
        if time.monotonic() - self._gt_stamp > self._gt_timeout:
            return False
        if self._gt_cov is None:      # 공분산 없는 타입(pose) → 게이팅 비활성, 통과 취급
            return True
        return (self._gt_cov[0] <= self._max_cov_pos
                and self._gt_cov[1] <= self._max_cov_yaw)

    # ------- info 서비스 -------
    def _on_list_tests(self, request, response):
        response.tests = list(TESTS.keys())
        response.descriptions = list(TESTS.values())
        response.default_params = [
            f"side_length={self.get_parameter('side_length').value} "
            f"loops={self.get_parameter('loops').value} "
            f"repeats={self.get_parameter('repeats').value}"
        ] * len(TESTS)
        return response

    # ------- run 액션 -------
    def _on_run(self, goal_handle):
        g = goal_handle.request
        test = g.test
        result = RunTest.Result()
        if test not in TESTS:
            goal_handle.abort()
            result.success = False
            result.message = f'알 수 없는 테스트: {test} (가능: {list(TESTS.keys())})'
            return result

        L = g.side_length if g.side_length > 0 else self.get_parameter('side_length').value
        loops = g.loops if g.loops > 0 else self.get_parameter('loops').value
        repeats = g.repeats if g.repeats > 0 else self.get_parameter('repeats').value

        if self._pose_a is None or self._pose_b is None:
            goal_handle.abort()
            result.success = False
            result.message = 'odom 토픽 수신 전 — 두 오도메트리가 발행 중인지 확인'
            return result
        if self._gt is None:
            goal_handle.abort()
            result.success = False
            gt_topic = self.get_parameter('gt_topic').value
            result.message = f'{gt_topic} 수신 전 — MCL(로컬라이제이션)이 발행 중인지 확인'
            return result

        prim = self._make_primitives(goal_handle)
        conditions = ['square_cw', 'square_ccw', 'strafe_square'] if test == 'full' else [test]

        try:
            for rep in range(repeats):
                for cond in conditions:
                    if g.dry_run:
                        self.get_logger().info(f'[dry_run] {cond} loops={loops} L={L}')
                        continue
                    self._run_condition(cond, goal_handle, prim, loops, L, rep)
        except SegmentAborted:
            prim.stop()
            goal_handle.canceled()
            result.success = False
            result.message = '사용자 취소로 중단 (cmd_vel 0 발행)'
            return result
        except SegmentGuardTripped as e:
            prim.stop()
            goal_handle.abort()
            result.success = False
            result.message = f'안전 가드 발동: {e}'
            return result

        prim.stop()
        e_pos, e_yaw = self._residual()   # feedback_source odom의 MCL 잔차(대표값)
        goal_handle.succeed()
        result.success = True
        result.message = f'{test} 완료 (repeats={repeats}, loops={loops})'
        result.output_path = self._output_dir
        result.final_drift_pos = e_pos    # 공유 필드 재사용 → "MCL 잔차"로 읽음(문서 §5)
        result.final_drift_yaw = e_yaw
        return result

    # ------- 조건별 실행 -------
    def _run_condition(self, cond, gh, prim, loops, L, rep):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        writer, fh = self._open_csv(cond, rep, stamp)
        # 시작 시점 정렬(문서 §7): odom(t)를 map 프레임으로 옮기는 고정 변환.
        start_gt = self._gt
        self._align_a = align_transform(start_gt, self._pose_a)
        self._align_b = align_transform(start_gt, self._pose_b)
        if not self._gt_quality_ok():
            self.get_logger().warn(
                f'[{cond}] 시작 시 MCL 공분산/신선도 미달 — 정렬 기준 신뢰 낮음(품질 플래그 기록)')
        if self._record_tum:
            self._recorder.start(['odom_a', 'odom_b', 'mcl'])
        try:
            for loop in range(loops):
                if cond == 'square_cw':
                    self._square(gh, prim, L, sign=-1.0, loop=loop, loops=loops, cond=cond)
                elif cond == 'square_ccw':
                    self._square(gh, prim, L, sign=+1.0, loop=loop, loops=loops, cond=cond)
                elif cond == 'strafe_square':
                    self._strafe_square(gh, prim, L, loop=loop, loops=loops, cond=cond)
                self._log_checkpoint(writer, cond, loop)
        finally:
            fh.close()
            if self._record_tum:
                self._recorder.stop()
                self._write_tum(cond, rep, stamp)

    def _write_tum(self, cond, rep, stamp):
        """세 시리즈를 TUM 으로 저장(evo_ape/rpe 입력, 문서 §8)."""
        for name in ('odom_a', 'odom_b', 'mcl'):
            path = os.path.join(
                self._output_dir, f'm4_{cond}_rep{rep}_{stamp}_{name}.tum')
            written, n = self._recorder.write_tum(name, path)
            if written:
                self.get_logger().info(f'TUM 기록: {written} ({n} 샘플)')
            else:
                self.get_logger().warn(f'TUM {name}: 샘플 없음 — 토픽 수신 확인')

    def _square(self, gh, prim, L, sign, loop, loops, cond):
        for _ in range(4):
            self._feedback(gh, cond, loop, loops, 'drive')
            prim.drive(L)
            prim.settle()
            self._feedback(gh, cond, loop, loops, 'rotate')
            prim.rotate(sign * HALF_PI)
            prim.settle()

    def _strafe_square(self, gh, prim, L, loop, loops, cond):
        seq = [('drive', prim.drive, +L), ('strafe', prim.strafe, +L),
               ('drive', prim.drive, -L), ('strafe', prim.strafe, -L)]
        for name, fn, d in seq:
            self._feedback(gh, cond, loop, loops, name)
            fn(d)
            prim.settle()

    # ------- 헬퍼 -------
    def _make_primitives(self, goal_handle):
        p = MotionParams(
            v_lin=self.get_parameter('v_lin').value,
            v_ang=self.get_parameter('v_ang').value,
            publish_rate=self.get_parameter('publish_rate').value,
            settle_time=self.get_parameter('settle_time').value,
            reach_tol_lin=self.get_parameter('reach_tol_lin').value,
            reach_tol_ang=self.get_parameter('reach_tol_ang').value,
            v_lin_min=self.get_parameter('v_lin_min').value,
            v_ang_min=self.get_parameter('v_ang_min').value,
            slowdown_lin=self.get_parameter('slowdown_lin').value,
            slowdown_ang=self.get_parameter('slowdown_ang').value,
            max_seg_time=self.get_parameter('max_seg_time').value,
            max_seg_dist=self.get_parameter('max_seg_dist').value,
        )
        return MotionPrimitives(
            logger=self.get_logger(),
            publish_fn=self._publish_cmd,
            get_pose_fn=self._feedback_pose,
            params=p,
            abort_fn=goal_handle.is_cancel_requested,
        )

    def _publish_cmd(self, vx, vy, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self._pub.publish(t)

    def _feedback_pose(self) -> Pose2D:
        # 주행 제어 피드백은 매끄러운 odom만 사용 — MCL(점프·지연) 금지(문서 §4).
        return self._pose_a if self._fb_src == 'a' else self._pose_b

    def _error_of(self, pose, align):
        """정렬된 odom(pose)을 map 프레임으로 옮겨 MCL과의 잔차 계산."""
        in_map = compose(align, pose)
        return (planar_distance(in_map, self._gt),
                wrap_angle(in_map.yaw - self._gt.yaw))

    def _residual(self):
        """대표 잔차 = feedback_source odom의 MCL 잔차. (상세 A/B는 CSV)"""
        if self._align_a is None or self._gt is None:
            return 0.0, 0.0
        if self._fb_src == 'a':
            return self._error_of(self._pose_a, self._align_a)
        return self._error_of(self._pose_b, self._align_b)

    def _feedback(self, gh, cond, loop, loops, segment):
        e_pos, e_yaw = self._residual()
        fb = RunTest.Feedback()
        fb.phase = cond
        fb.current_loop = loop + 1
        fb.total_loops = loops
        fb.current_segment = segment
        fb.drift_pos_so_far = e_pos     # "MCL 잔차"로 읽음
        fb.drift_yaw_so_far = e_yaw
        gh.publish_feedback(fb)

    def _open_csv(self, cond, rep, stamp):
        os.makedirs(self._output_dir, exist_ok=True)
        path = os.path.join(self._output_dir, f'm4_{cond}_rep{rep}_{stamp}.csv')
        fh = open(path, 'w', newline='')
        w = csv.writer(fh)
        w.writerow(['loop', 'ax', 'ay', 'ayaw', 'bx', 'by', 'byaw',
                    'gx', 'gy', 'gyaw', 'cov_pos', 'cov_yaw', 'quality_ok',
                    'err_a_pos', 'err_a_yaw', 'err_b_pos', 'err_b_yaw'])
        self.get_logger().info(f'기록 파일: {path}')
        return w, fh

    def _log_checkpoint(self, writer, cond, loop):
        a, b, gt = self._pose_a, self._pose_b, self._gt
        ea_pos, ea_yaw = self._error_of(a, self._align_a)
        eb_pos, eb_yaw = self._error_of(b, self._align_b)
        cov_pos = self._gt_cov[0] if self._gt_cov else float('nan')
        cov_yaw = self._gt_cov[1] if self._gt_cov else float('nan')
        writer.writerow([loop, a.x, a.y, a.yaw, b.x, b.y, b.yaw,
                         gt.x, gt.y, gt.yaw, cov_pos, cov_yaw,
                         int(self._gt_quality_ok()),
                         ea_pos, ea_yaw, eb_pos, eb_yaw])


def main(args=None):
    rclpy.init(args=args)
    node = OdomMclNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0, 0.0)  # 안전: 종료 시 정지
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
