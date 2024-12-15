import rclpy
import numpy as np
import sys
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import rclpy.duration
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node

class local_subscriber(Node):
    def __init__(self):
        super().__init__('local_subscriber')
        self.sub_costmap = self.create_subscription(
            Costmap,
            '/local_costmap/costmap_raw',
            self.sub_costmap_callback,
            20
        )
        self.sub_cmdvel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.sub_cmdvel_callback,
            20
        )
        self.pub_cmdvel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.costmap = None
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)
        self.cmdvel = Twist()
        self.prev_cmdvel = Twist()
        self.forced = False
        self.timer = None
        
        # 목표 위치 설정
        self.goal = Point()
        self.goal.x = -10.0
        self.goal.y = -2.0
        
        # 장애물 회피 여부
        self.is_avoiding = False

    def sub_costmap_callback(self, msg):
        """
        장애물 회피 및 목표 위치 추적
        """
        # Local costmap 데이터를 처리
        costmap = np.array(msg.data)
        self.costmap = costmap.reshape(msg.metadata.size_x, msg.metadata.size_y).T
        # Costmap의 기울기 계산
        self.costmap_grad_x, self.costmap_grad_y = np.gradient(self.costmap)
        
        # Transform object for coordinate transformation
        transform = self.tf2_buffer.lookup_transform(
            'map',
            'base_footprint',
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=0.2)
        )
        
        # 로봇의 위치 계산
        x_local = int((transform.transform.translation.x - msg.metadata.origin.position.x) * 20)
        y_local = int((transform.transform.translation.y - msg.metadata.origin.position.y) * 20)

        # 로봇이 costmap 내에 있도록 보장
        x_local = np.clip(x_local, 0, self.costmap.shape[0] - 1)
        y_local = np.clip(y_local, 0, self.costmap.shape[1] - 1)

        # 로봇 위치에서의 기울기 계산
        x = self.costmap_grad_x[x_local, y_local]
        y = self.costmap_grad_y[x_local, y_local]
        
        # 로봇의 회전 정보
        quaternion = transform.transform.rotation
        roll, pitch, yaw = self.euler_from_quaternion(quaternion)
        
        self.robot_grad_x = x * np.cos(yaw) + y * np.sin(yaw)
        self.robot_grad_y = -x * np.sin(yaw) + y * np.cos(yaw)
        self.robot_grad = np.array([self.robot_grad_x, self.robot_grad_y])

        # 로봇이 목표 위치에 가까워졌는지 확인
        self.check_goal_reached(transform.transform.translation.x, transform.transform.translation.y)
        
        # 장애물 회피 및 목표 위치로 이동
        twist_trans = np.array([self.cmdvel.linear.x, self.cmdvel.linear.y, self.cmdvel.linear.z])
        twist_trans_2d = twist_trans[0:2]
        
        if np.dot(twist_trans_2d, self.robot_grad) > 0:
            vel = Twist()
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = -0.5 if self.robot_grad_y > 0 else 0.5
            self.prev_cmdvel = self.cmdvel
            self.pub_cmdvel.publish(vel)
            self.timer = self.create_timer(0.2, self.get_forced)
        elif self.is_avoiding and self.robot_grad_x ** 2 < 80:
            self.pub_cmdvel.publish(self.prev_cmdvel)
            self.is_avoiding = False

    def sub_cmdvel_callback(self, msg):
        """
        현재 속도 정보 저장
        """
        self.cmdvel = msg
        if self.is_avoiding:
            self.is_avoiding = False

    def get_forced(self):
        """
        장애물 회피 중인 상태로 전환
        """
        self.is_avoiding = True
        self.timer.cancel()

    def euler_from_quaternion(self, quaternion):
        """
        쿼터니언을 오일러 각도로 변환
        """
        import math
        sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def check_goal_reached(self, x, y):
        """
        목표 좌표에 도달했는지 확인
        """
        distance = np.sqrt((self.goal.x - x) ** 2 + (self.goal.y - y) ** 2)
        if distance < 1.0:
            self.get_logger().info(f"Goal reached: {x}, {y}")
            self.cmdvel.linear.x = 0.0
            self.cmdvel.angular.z = 0.0
            self.pub_cmdvel.publish(self.cmdvel)

def main(args=None):
    rclpy.init(args=args)
    local_sub = local_subscriber()
    rclpy.spin(local_sub)
    local_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
