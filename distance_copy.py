import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('distance_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # 라이다 토픽 이름 (실제 환경에 맞게 수정 필요)
            self.listener_callback,
            10
        )
        self.front_range = []  # 11시 ~ 1시 방향 거리 데이터를 저장

    def listener_callback(self, msg):
        # 뒤쪽을 향한 라이다 센서의 경우, 배열 끝 부분이 전방의 거리
        # self.front_distance = msg.ranges[-1]   # 12시 방향 거리
        # self.right_distance = msg.ranges[int(len(msg.ranges) * (11/12))]  # 2시 방향 거리
        # self.left_distance = msg.ranges[int(len(msg.ranges) * (1/12))]  # 10시 방향 거리
        # self.get_logger().info(f'front distance: {self.front_distance:.2f} meters')
        # self.get_logger().info(f'rignt distance: {self.right_distance:.2f} meters')
        # self.get_logger().info(f'left distance: {self.left_distance:.2f} meters')


        self.front_range = msg.ranges[330:] + msg.ranges[:30]  # 데이터를 이어 붙임
        self.front_distance = min(self.front_range)  # 최소 거리 사용
        self.get_logger().info(f'minimum front distance: {self.front_distance:.2f} meters')



def main(args=None):
    rclpy.init(args=args)
    distance_subscriber = DistanceSubscriber()
    rclpy.spin(distance_subscriber)
    distance_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
