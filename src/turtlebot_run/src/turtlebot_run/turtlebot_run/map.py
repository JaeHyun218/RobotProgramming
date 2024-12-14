import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MoveAndAvoid(Node):
    def __init__(self):
        super().__init__('move_and_avoid')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.timer = self.create_timer(0.01, self.follow_wall)
        self.move_cmd = Twist()
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.front_range = []  # 11시 ~ 1시 방향 거리 데이터를 저장
        self.right_range = []  # 3시 주변 거리 데이터를 저장
        self.is_turning = False
        self.wall_distance = 0.5  # 목표 오른쪽 벽과의 거리 (단위: m)

    def laser_callback(self, msg):
        total_ranges = len(msg.ranges)

        # 11시 ~ 1시 방향 (330도 ~ 30도)
        self.front_range = (
            msg.ranges[305:] + msg.ranges[:55]  # 데이터를 이어 붙임
        )
        self.front_distance = min(self.front_range)  # 최소 거리 사용

        # 3시 주변 (80도 ~ 100도)
        self.right_range = msg.ranges[80:101]
        self.right_distance = sum(self.right_range) / len(self.right_range)  # 평균 계산

    def follow_wall(self):
        if self.is_turning:
            if self.front_distance > 0.75:  # 앞이 충분히 뚫릴 때까지 회전
                self.is_turning = False
                self.move_straight()
            else:
                self.turn_left()
        elif self.front_distance < 0.7:  # 전방 센서 중 하나라도 가까우면
            self.is_turning = True
            self.turn_left()
        else:
            self.adjust_to_wall()

    def adjust_to_wall(self):
        if self.right_distance > self.wall_distance + 0.15:  # 벽에서 멀어지면
            self.turn_right_slightly()
        elif self.right_distance < self.wall_distance - 0.15:  # 벽에 가까워지면
            self.turn_left_slightly()
        else:
            self.move_straight()

    def move_straight(self):
        self.move_cmd.linear.x = 0.5  # 전진 속도를 낮춤
        self.move_cmd.angular.z = 0.0
        self.publisher.publish(self.move_cmd)

    def turn_left(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.4  # 좌회전 속도를 높임
        self.publisher.publish(self.move_cmd)

    def turn_right_slightly(self):
        self.move_cmd.linear.x = 0.3  # 전진 속도를 낮춤
        self.move_cmd.angular.z = -0.4  # 약간 우회전
        self.publisher.publish(self.move_cmd)

    def turn_left_slightly(self):
        self.move_cmd.linear.x = 0.3  # 전진 속도를 낮춤
        self.move_cmd.angular.z = 0.4  # 약간 좌회전
        self.publisher.publish(self.move_cmd)

def main(args=None):
    rclpy.init(args=args)
    move_and_avoid_node = MoveAndAvoid()
    rclpy.spin(move_and_avoid_node)
    move_and_avoid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
