import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class TurtleBotMapper(Node):
    def __init__(self):
        super().__init__('turtlebot_mapper')

        # Publisher for controlling the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LaserScan data
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        # Timer for periodic movement
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.safe_distance = 0.5  # Safe distance from obstacles (meters)
        self.forward_speed = 0.2  # Linear speed (m/s)
        self.turn_speed = 0.5    # Angular speed (rad/s)

        self.is_obstacle_close = False

    def laser_callback(self, msg):
        # Process laser scan data
        self.is_obstacle_close = any(
            distance < self.safe_distance
            for distance in msg.ranges
            if not math.isinf(distance)
        )

    def timer_callback(self):
        twist = Twist()

        if self.is_obstacle_close:
            # Obstacle detected, stop and turn
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
        else:
            # No obstacle, move forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

    def save_map(self):
        # Save map using map_saver_cli command
        import subprocess
        try:
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', '~/map'
            ])
            self.get_logger().info('Map saved successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')


def main(args=None):
    rclpy.init(args=args)
    turtlebot_mapper = TurtleBotMapper()

    try:
        rclpy.spin(turtlebot_mapper)
    except KeyboardInterrupt:
        turtlebot_mapper.save_map()
        turtlebot_mapper.get_logger().info('Shutting down...')
    finally:
        turtlebot_mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()