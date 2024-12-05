import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

        self.bridge = CvBridge()  # ROS 이미지 메시지를 OpenCV로 변환
        self.model = YOLO('yolov8n.pt')  # YOLOv8 모델 로드 (YOLOv8n으로 설정)

    def image_callback(self, msg):
        try:
            start_time = time.time()

            # ROS 이미지 메시지를 OpenCV 형식으로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLOv8로 객체 감지 수행
            results = self.model(frame)

            # 결과에서 각 객체의 클래스 및 좌표 확인
            detected_classes = results[0].boxes.cls.cpu().numpy()
            class_names = self.model.names  # YOLOv8 클래스 이름 목록

            # Person 클래스가 감지되었는지 확인
            for cls in detected_classes:
                class_name = class_names[int(cls)]
                if class_name == 'person':
                    self.get_logger().info('Hello! Person detected!')
                    break

            # 결과 이미지 가져오기
            result_image = results[0].plot()

            # OpenCV 창에 표시
            cv2.imshow("YOLOv8 Object Detection", result_image)
            cv2.waitKey(1)

            # 처리 속도 제한 (최대 10 FPS)
            elapsed_time = time.time() - start_time
            if elapsed_time < 0.1:
                time.sleep(0.1 - elapsed_time)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


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