import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import DR_init
from collections import Counter, deque

from od_msg.srv import SrvDepthPosition
from ament_index_python.packages import get_package_share_directory
from order_interfaces.msg import Order

import cv2
import cv2.aruco as aruco

package_path = get_package_share_directory("burger")

ingredient_dict = {1: "bun_bottom", 2: "bun_top", 3: "cheese", 4: "lettuce", 
                   5: "onion", 6: "patty", 7: "shrimp", 8: "tomato"}

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()


class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place_integrated")

        # 메뉴 DB
        self.menu_db = {
            "불고기버거": ["bun_bottom", "patty", "lettuce", "tomato", "bun_top"],
            "치즈버거": ["bun_bottom", "patty", "cheese", "bun_top"],
            "새우버거": ["bun_bottom", "shrimp", "lettuce", "bun_top"],
        }

        self.ingredient_map_korean_to_yolo = {
            "빵": "bun_bottom", "불고기": "patty", "치즈": "cheese", 
            "상추": "lettuce", "토마토": "tomato", "새우": "shrimp"
        }

        self.order_queue = deque(maxlen=1)

        # 카메라 초기화
        self.cap = cv2.VideoCapture(2)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.aruco_params = aruco.DetectorParameters()

        self.init_robot()

        # self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        # while not self.depth_client.wait_for_service(timeout_sec=3):
        #     self.get_logger().info("Waiting for depth service...")


        self.order_subscription = self.create_subscription(
            Order, '/cmd', self.order_callback, QoSProfile(depth=10)
        )

        self.get_logger().info("Integrated Robot Controller is running (NO GRIPPER).")

    def order_callback(self, msg):
        self.get_logger().info("New order received.")
        self.order_queue.append(msg)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)

        base2gripper = self.get_robot_pose_matrix(*robot_pos)
        base2camera = base2gripper @ gripper2cam
        world = base2camera @ coord
        return world[:3]

    def find_marked_box(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to read from camera")
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        detected = []

        if ids is not None:
            # 마커 그리기
            aruco.drawDetectedMarkers(frame, corners, ids)

            for c, i in zip(corners, ids):
                marker_id = int(i[0])
                pts = c[0]

                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                # 중심점도 보고 싶으면 원으로 찍기
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                detected.append((marker_id, cx, cy))

        # ==== 여기서 실시간 창 띄우기 ====
        cv2.imshow("Aruco Marker View", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # q 누르면 창 닫고, 빈 리스트 리턴 (필요하면 rclpy.shutdown 쪽 처리도 가능)
            cv2.destroyAllWindows()
        # =================================

        return detected

    def pick_and_place_target(self, target_pos):
        # 이 함수는 하나의 재료를 집어서 고정된 위치(BUCKET_POS)에 놓는 동작을 가정합니다.
        # 간단한 시연을 위해, 집은 후 바로 BUCKET_POS로 이동합니다.
        
        # 집기
        pick_pos_above = trans(target_pos, [0, 0, 100, 0, 0, 0])
        movel(pick_pos_above, vel=VELOCITY, acc=ACC)
        mwait()
        movel(target_pos, vel=VELOCITY/2, acc=ACC/2)
        mwait()
        #gripper.close_gripper()
        time.sleep(1)
        movel(pick_pos_above, vel=VELOCITY, acc=ACC)
        mwait()

        # 놓기 (고정된 위치)
        self.get_logger().info(f"Placing ingredient at BUCKET_POS: {BUCKET_POS}")
        movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
        mwait()
        #gripper.open_gripper()
        time.sleep(1)

    def box_setting(self, boxes):
        global BUCKET_POS
        if boxes == 1:
            BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]
        elif boxes == 2:
            BUCKET_POS = [445.5, -142.6, 174.4, 156.4, 180.0, -112.5]
        elif boxes == 3:
            BUCKET_POS = [445.5, -42.6, 174.4, 156.4, 180.0, -112.5]

    def robot_control(self):

        box = self.find_marked_box()
        print("Detected markers:", box)

        if not self.order_queue:
            return

        for i in len(box):
            self.pick_and_place_target(i)
        

        

    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        mwait()

    def pick_and_place_target(self, target_pos):

        pick_above = trans(target_pos, [0, 0, 100, 0, 0, 0])
        movel(pick_above, vel=VELOCITY, acc=ACC)
        mwait()

        movel(target_pos, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        movel(pick_above, vel=VELOCITY, acc=ACC)
        mwait()

        movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
        mwait()


def main(args=None):
    node = RobotController()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        node.robot_control()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
