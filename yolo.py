import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import time


class YOLOv8ObjectDetection(Node):
    def __init__(self):
        super().__init__('yolov8_object_detection')

        # 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # TurtleBot3의 기본 카메라 토픽
            self.image_callback,
            10
        )

        # 속도 명령 퍼블리셔 설정
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 주기적 제어를 위한 타이머 설정
        self.timer = self.create_timer(0.1, self.control_robot)  # 0.1초 간격

        self.bridge = CvBridge()  # ROS 이미지 메시지를 OpenCV로 변환
        self.model = YOLO('yolov8n.pt')  # YOLOv8 모델 로드 (YOLOv8n으로 설정)

        self.person_detected = False
        self.detection_start_time = None
        self.current_rotation_direction = 0  # 회전 방향 (0: 정지, 1: 시계 반대, -1: 시계 방향)

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 형식으로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # YOLOv8로 객체 감지 수행
            results = self.model(frame)

            # 결과에서 각 객체의 클래스 및 좌표 확인
            detected_classes = results[0].boxes.cls.cpu().numpy()
            class_names = self.model.names  # YOLOv8 클래스 이름 목록

            self.person_detected = False  # 초기화

            for cls in detected_classes:
                class_name = class_names[int(cls)]
                if class_name == 'person':
                    self.person_detected = True
                    if self.detection_start_time is None:
                        self.detection_start_time = time.time()
                    self.get_logger().info('Person detected!')
                    break

            # 결과 이미지 가져오기
            result_image = results[0].plot()

            # OpenCV 창에 표시
            cv2.imshow("YOLOv8 Object Detection", result_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def control_robot(self):
        twist = Twist()
        current_time = time.time()

        if self.person_detected and self.detection_start_time is not None:
            elapsed_time = current_time - self.detection_start_time
            if elapsed_time <= 5.0:
                # 5초 동안 좌우 회전
                self.current_rotation_direction = 1 if int(elapsed_time * 2) % 2 == 0 else -1
                twist.angular.z = 0.5 * self.current_rotation_direction
            else:
                # 5초 이후 정지
                self.person_detected = False
                self.detection_start_time = None
                twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    # 노드 실행
    yolov8_node = YOLOv8ObjectDetection()

    try:
        rclpy.spin(yolov8_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolov8_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()