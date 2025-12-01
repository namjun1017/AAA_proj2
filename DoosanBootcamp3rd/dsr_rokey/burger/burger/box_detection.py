import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from burger.realsense import ImgNode
from burger.yolo import YoloModel

import cv2
import cv2.aruco as aruco

PACKAGE_NAME = 'burger'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('object_detection_node')

        # RealSense 이미지 노드
        self.img_node = ImgNode()

        # YOLO 모델
        self.model = self._load_model(model_name)

        # 카메라 intrinsics
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )

        # ✅ YOLO 기반 3D 좌표 서비스
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )

        # ✅ ArUco 기반 3D 좌표 서비스
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.aruco_params = aruco.DetectorParameters()
        self.create_service(
            SrvDepthPosition,
            'get_marker_position',
            self.handle_get_marker_position
        )

        self.get_logger().info("ObjectDetectionNode initialized (YOLO + ArUco).")

    # ===== YOLO 관련 =====

    def _load_model(self, name):
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """YOLO로 target 클래스 찾아서 3D 카메라 좌표 반환"""
        self.get_logger().info(f"[YOLO] Received request: {request.target}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        """YOLO + depth 로 객체의 카메라 좌표 계산"""
        rclpy.spin_once(self.img_node)

        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            self.get_logger().warn("[YOLO] No detection found.")
            return 0.0, 0.0, 0.0

        self.get_logger().info(f"[YOLO] Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("[YOLO] Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    # ===== ArUco 관련 =====

    def handle_get_marker_position(self, request, response):
        """
        request.target: 마커 ID (예: "0", "5")
        응답: depth_position = [X, Y, Z] (카메라 좌표계)
        """
        self.get_logger().info(f"[ARUCO] Received marker request: {request.target}")

        try:
            marker_id = int(request.target)
        except ValueError:
            self.get_logger().warn(f"[ARUCO] Invalid marker id: {request.target}")
            response.depth_position = [0.0, 0.0, 0.0]
            return response

        # 컬러 / 뎁스 프레임 가져오기
        color = self._wait_for_valid_data(self.img_node.get_color_frame, "color frame")
        depth = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")

        if color is None or depth is None:
            self.get_logger().warn("[ARUCO] Failed to get color/depth frame")
            response.depth_position = [0.0, 0.0, 0.0]
            return response

        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None:
            self.get_logger().warn("[ARUCO] No markers detected")
            response.depth_position = [0.0, 0.0, 0.0]
            return response

        cx = cy = None
        for c, i in zip(corners, ids):
            if int(i[0]) == marker_id:
                pts = c[0]  # (4,2)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                break

        if cx is None:
            self.get_logger().warn(f"[ARUCO] Marker id {marker_id} not found")
            response.depth_position = [0.0, 0.0, 0.0]
            return response

        # depth에서 z 값 읽기
        try:
            z = depth[cy, cx]
        except IndexError:
            self.get_logger().warn(f"[ARUCO] Depth index out of range at ({cx},{cy})")
            response.depth_position = [0.0, 0.0, 0.0]
            return response

        if z <= 0:
            self.get_logger().warn(f"[ARUCO] Invalid depth at ({cx},{cy}): {z}")
            response.depth_position = [0.0, 0.0, 0.0]
            return response

        X, Y, Z = self._pixel_to_camera_coords(cx, cy, z)
        self.get_logger().info(f"[ARUCO] Marker {marker_id} camera coords: ({X:.1f}, {Y:.1f}, {Z:.1f})")
        response.depth_position = [float(X), float(Y), float(Z)]
        return response

    # ===== 공통 유틸 =====

    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
