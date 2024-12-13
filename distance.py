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
            '/scan',  # 라이다 토픽 이름 (실제 환경에 맞게 수정 필요)
            self.laser_callback,
            10
        )
        self.timer = self.create_timer(0.01, self.follow_wall)  # 0.01초마다 호출
        self.move_cmd = Twist()
        self.front_distance = float('inf')  # 초기 값은 무한대
        self.right_distance = float('inf')  # 초기 값은 무한대
        self.is_turning = False  # 유턴 상태 플래그
        self.wall_distance = 0.5  # 목표 오른쪽 벽과의 거리 (단위: m)

    def laser_callback(self, msg):
        self.front_distance = msg.ranges[-1]  # 12시 방향 거리
        self.right_distance = msg.ranges[len(msg.ranges) // 4]  # 3시 방향 거리

    def follow_wall(self):
        if self.is_turning:
            # 유턴 중일 때 충분히 회전하도록 계속 회전 명령 유지
            if self.front_distance > 1.0:  # 앞이 충분히 뚫릴 때까지 회전
                self.is_turning = False
                self.move_straight()
            else:
                self.turn_left()
        elif self.front_distance < 0.7:  # 앞 장애물과의 최소 거리
            self.is_turning = True
            self.turn_left()
        else:
            self.adjust_to_wall()

    def adjust_to_wall(self):
        if self.right_distance > self.wall_distance + 0.1:  # 벽에서 멀어지면
            self.turn_right_slightly()
        elif self.right_distance < self.wall_distance - 0.1:  # 벽에 가까워지면
            self.turn_left_slightly()
        else:
            self.move_straight()

    def move_straight(self):
        self.move_cmd.linear.x = 0.2  # 전진 속도 (단위: m/s)
        self.move_cmd.angular.z = 0.0  # 회전 속도 (단위: rad/s)
        self.publisher.publish(self.move_cmd)

    def turn_left(self):
        self.move_cmd.linear.x = 0.0  # 전진 속도는 0
        self.move_cmd.angular.z = 0.3  # 좌회전 속도 (단위: rad/s)
        self.publisher.publish(self.move_cmd)

    def turn_right_slightly(self):
        self.move_cmd.linear.x = 0.2  # 전진하면서
        self.move_cmd.angular.z = -0.1  # 약간 우회전
        self.publisher.publish(self.move_cmd)

    def turn_left_slightly(self):
        self.move_cmd.linear.x = 0.1  # 전진하면서
        self.move_cmd.angular.z = 0.2  # 약간 좌회전
        self.publisher.publish(self.move_cmd)

def main(args=None):
    rclpy.init(args=args)
    move_and_avoid_node = MoveAndAvoid()
    rclpy.spin(move_and_avoid_node)
    move_and_avoid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
