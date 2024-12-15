import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math  # 쿼터니언 계산에 필요


class TurtlebotNav2(Node):
    def __init__(self):
        super().__init__('turtlebot_nav2')

        # NavigateToPose 액션 클라이언트 생성
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 목표 위치 설정
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = -10.0  # 목표 x 좌표
        self.goal_pose.pose.position.y = -2.0  # 목표 y 좌표
        self.goal_pose.pose.position.z = 0.0  # z는 항상 0

        # 목표 방향 설정 (왼쪽 방향: yaw = π/2)
        yaw = math.pi / 2  # 90도 (라디안)
        self.goal_pose.pose.orientation = self.euler_to_quaternion(0, 0, yaw)

        self.get_logger().info('Waiting for NavigateToPose action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('NavigateToPose action server is ready.')

        # 목표로 이동 요청
        self.send_goal()

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Euler 각도를 쿼터니언으로 변환.
        roll, pitch, yaw는 라디안 단위로 입력.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        quaternion = PoseStamped().pose.orientation
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw
        return quaternion

    def send_goal(self):
        self.get_logger().info('Sending goal to NavigateToPose action server...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self.future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current progress: {feedback.current_pose}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # TurtlebotNav2 노드 실행
    node = TurtlebotNav2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
