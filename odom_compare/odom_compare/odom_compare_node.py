"""방법 ① — swerve vs EKF 오도메트리 드리프트 비교 노드.

- info 서비스: `~/list_tests` (ListTests) — 실행 가능한 테스트 목록.
- run 액션:   `~/run_test`   (RunTest)   — 테스트 하나 실행(피드백·취소 지원).
네임스페이스는 launch에서 `/method1` 등으로 부여한다(문서 §5).

주의: 이 방법은 절대 정확도가 아니라 두 추정기의 상호 드리프트만 본다(문서 §0).
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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from odom_test_interfaces.srv import ListTests
from odom_test_interfaces.action import RunTest

from odom_test_core.pose_utils import Pose2D, pose2d_from_odom, wrap_angle
from odom_test_core.primitives import (
    MotionParams, MotionPrimitives, SegmentAborted, SegmentGuardTripped,
)

HALF_PI = math.pi / 2.0

TESTS = {
    'square_cw':     '1x1 정사각형, 코너 90° 시계방향 회전 (누적 +회전)',
    'square_ccw':    '1x1 정사각형, 코너 90° 반시계방향 회전 (누적 -회전)',
    'strafe_square': '1x1 정사각형, 헤딩 고정 병진(+x,+y,-x,-y) — 회전 0',
    'full':          '위 세 조건을 순차 실행',
}


class OdomCompareNode(Node):
    def __init__(self):
        super().__init__('odom_compare')
        self._cbg = ReentrantCallbackGroup()

        # --- 파라미터 (config에서 오버라이드) ---
        self.declare_parameter('cmd_vel_topic', '/swerve_controller/cmd_vel')
        self.declare_parameter('odom_a_topic', '/swerve_controller/odom')
        self.declare_parameter('odom_b_topic', '/fused_odom')
        self.declare_parameter('feedback_source', 'b')  # a|b
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('loops', 5)
        self.declare_parameter('repeats', 1)
        self.declare_parameter('v_lin', 0.15)
        self.declare_parameter('v_ang', 0.3)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('settle_time', 1.0)
        self.declare_parameter('reach_tol_lin', 0.001)   # [m] 도달 허용오차(위치)
        self.declare_parameter('reach_tol_ang', 0.002)   # [rad] 도달 허용오차(각도)
        # 목표 근처 최소 속도 — 주기당 이동량이 곧 정지 분해능. 허용오차보다 충분히 작게.
        self.declare_parameter('v_lin_min', 0.01)        # [m/s] 30Hz → 주기당 ~0.33mm
        self.declare_parameter('v_ang_min', 0.02)        # [rad/s] 30Hz → 주기당 ~0.038°
        self.declare_parameter('slowdown_lin', 0.10)     # [m] 감속 시작 남은거리
        self.declare_parameter('slowdown_ang', 0.10)     # [rad] 감속 시작 남은각도
        self.declare_parameter('max_seg_time', 30.0)
        self.declare_parameter('max_seg_dist', 1.5)
        self.declare_parameter('output_dir', 'results')

        gp = self.get_parameter
        self._cmd_topic = gp('cmd_vel_topic').value
        self._fb_src = gp('feedback_source').value
        # 상대경로면 실행 위치(CWD) 기준 절대경로로 해석, "~"도 확장.
        # 결과는 프로젝트 로컬(예: repo의 results/, .gitignore 대상)에 저장.
        self._output_dir = os.path.abspath(os.path.expanduser(gp('output_dir').value))

        # --- I/O ---
        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._pose_a = None
        self._pose_b = None
        self.create_subscription(
            Odometry, gp('odom_a_topic').value,
            lambda m: setattr(self, '_pose_a', pose2d_from_odom(m)),
            20, callback_group=self._cbg)
        self.create_subscription(
            Odometry, gp('odom_b_topic').value,
            lambda m: setattr(self, '_pose_b', pose2d_from_odom(m)),
            20, callback_group=self._cbg)

        # --- 서비스 / 액션 ---
        # 상대 이름 → 네임스페이스(/method1) 아래로: /method1/list_tests, /method1/run_test
        self.create_service(
            ListTests, 'list_tests', self._on_list_tests,
            callback_group=self._cbg)
        self._action = ActionServer(
            self, RunTest, 'run_test',
            execute_callback=self._on_run,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT,
            callback_group=self._cbg)

        self.get_logger().info('odom_compare 준비됨 (list_tests 서비스, run_test 액션)')

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
        d_pos, d_yaw = self._drift()
        goal_handle.succeed()
        result.success = True
        result.message = f'{test} 완료 (repeats={repeats}, loops={loops})'
        result.output_path = self._output_dir
        result.final_drift_pos = d_pos
        result.final_drift_yaw = d_yaw
        return result

    # ------- 조건별 실행 -------
    def _run_condition(self, cond, gh, prim, loops, L, rep):
        writer, fh = self._open_csv(cond, rep)
        start_a = self._pose_a
        start_b = self._pose_b
        try:
            for loop in range(loops):
                if cond == 'square_cw':
                    self._square(gh, prim, L, sign=-1.0, loop=loop, loops=loops, cond=cond)
                elif cond == 'square_ccw':
                    self._square(gh, prim, L, sign=+1.0, loop=loop, loops=loops, cond=cond)
                elif cond == 'strafe_square':
                    self._strafe_square(gh, prim, L, loop=loop, loops=loops, cond=cond)
                self._log_checkpoint(writer, start_a, start_b, cond, loop)
        finally:
            fh.close()

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
        return self._pose_a if self._fb_src == 'a' else self._pose_b

    def _drift(self):
        """두 추정기의 상호 드리프트(재영점은 분석 단계 몫이라 여기선 상대 차이 근사)."""
        a, b = self._pose_a, self._pose_b
        d_pos = math.hypot(a.x - b.x, a.y - b.y)
        d_yaw = wrap_angle(a.yaw - b.yaw)
        return d_pos, d_yaw

    def _feedback(self, gh, cond, loop, loops, segment):
        d_pos, d_yaw = self._drift()
        fb = RunTest.Feedback()
        fb.phase = cond
        fb.current_loop = loop + 1
        fb.total_loops = loops
        fb.current_segment = segment
        fb.drift_pos_so_far = d_pos
        fb.drift_yaw_so_far = d_yaw
        gh.publish_feedback(fb)

    def _open_csv(self, cond, rep):
        os.makedirs(self._output_dir, exist_ok=True)
        # 실행 시각으로 파일명 (런타임이므로 time 사용 가능)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self._output_dir, f'{cond}_rep{rep}_{stamp}.csv')
        fh = open(path, 'w', newline='')
        w = csv.writer(fh)
        w.writerow(['loop', 'ax', 'ay', 'ayaw', 'bx', 'by', 'byaw',
                    'drift_pos', 'drift_yaw'])
        self.get_logger().info(f'기록 파일: {path}')
        return w, fh

    def _log_checkpoint(self, writer, start_a, start_b, cond, loop):
        # 시작 재영점(같은 odom 프레임, 회전 없음 — 문서 §7)
        a, b = self._pose_a, self._pose_b
        ra = (a.x - start_a.x, a.y - start_a.y, wrap_angle(a.yaw - start_a.yaw))
        rb = (b.x - start_b.x, b.y - start_b.y, wrap_angle(b.yaw - start_b.yaw))
        d_pos = math.hypot(ra[0] - rb[0], ra[1] - rb[1])
        d_yaw = wrap_angle(ra[2] - rb[2])
        writer.writerow([loop, *ra, *rb, d_pos, d_yaw])


def main(args=None):
    rclpy.init(args=args)
    node = OdomCompareNode()
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
