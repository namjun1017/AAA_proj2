import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from burger.yunet import YuNetModel
from std_msgs.msg import Bool
import os
import time

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.yunet_model = YuNetModel()
        self.face_publisher = self.create_publisher(Bool, '/face', 10)
        self.cap = cv2.VideoCapture(0)
        # 1. 노드 종료를 위한 플래그 추가
        self.should_shutdown = False 

    def detect_face_and_order(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        if frame is not None:
            # 2. 얼굴 감지 로직
            faces = self.yunet_model.get_detections(frame)
            
            # faces[1]이 None이 아니면 얼굴이 감지된 것입니다.
            if faces[1] is not None:
                for face in faces[1]:
                    box = face[0:4].astype(np.int32)
                    cv2.rectangle(frame, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0, 255, 0), 2)
                
                # 3. True 발행 및 종료 플래그 설정
                msg = Bool()
                msg.data = True
                self.face_publisher.publish(msg)
                self.get_logger().info('✅ Face detected! Publishing True to /face topic and initiating shutdown.')
                
                # **종료 플래그를 True로 설정**
                self.should_shutdown = True 

            # 얼굴 감지 여부와 관계없이 화면 표시 및 Key Wait는 유지
            cv2.imshow("Webcam Feed", frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    face_detection_node = FaceDetectionNode()
    
    # 4. 루프 조건 변경: rclpy.ok()와 should_shutdown 플래그 모두 확인
    while rclpy.ok() and not face_detection_node.should_shutdown:
        face_detection_node.detect_face_and_order()
        # 짧은 시간만 스핀하여 감지 및 종료 플래그를 빠르게 확인할 수 있게 함
        rclpy.spin_once(face_detection_node, timeout_sec=0.01)

    # 5. 종료 정리
    face_detection_node.get_logger().info('👋 Shutting down the node...')
    face_detection_node.cap.release()
    cv2.destroyAllWindows()
    face_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()