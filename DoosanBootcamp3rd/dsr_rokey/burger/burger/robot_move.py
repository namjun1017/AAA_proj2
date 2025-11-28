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
from burger.onrobot import RG
from order_interfaces.msg import Order 

package_path = get_package_share_directory("burger")

# --- [수정된 부분 1] ---
# [주의] 아래 목록은 학습된 YOLO 모델의 클래스 이름과 정확히 일치해야 합니다.
# 모델에 있는 실제 클래스 이름으로 수정해주세요. 예: 'patty', 'lettuce', 'tomato' 등
ingredient_dict = {1: "bun_bottom", 2: "bun_top", 3: "cheese", 4: "lettuce", 5: "onion", 6: "patty", 7: "shrimp", 8: "tomato"}
# ----------------------

# for single robot
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

########### Gripper Setup. Do not modify this area ############

GRIPPER_NAME = "rg2"
TOOLCHANGER_IP = "192.168.1.1"
TOOLCHANGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)


########### Robot Controller ############


class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # 🚨 [추가] 메뉴 DB 및 매핑 정의
        self.menu_db = {
            "불고기버거": ["빵", "불고기", "상추", "토마토", "빵"],
            "치즈버거": ["빵", "불고기", "치즈", "빵"],
            "새우버거": ["빵", "새우", "상추", "빵"],
        }
        
        self.ingredient_map = {
            "빵": "bun_bottom", "불고기": "patty", "치즈": "cheese", "상추": "lettuce", 
            "토마토": "tomato", "새우": "shrimp", "bun_top": "bun_top" 
        }
        
        # 🚨 [추가] 작업 큐 초기화
        self.pending_tasks = [] 

        self.init_robot()
        self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.depth_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for depth position service...")
        self.depth_request = SrvDepthPosition.Request()
        self.robot_control()

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def robot_control(self):
        # --- [수정된 부분 2] ---
        print("====================================")
        print("Available ingredients: ")
        for key, value in ingredient_dict.items():
            print(f"  {key} : {value}")
        print("")

        user_input = input("What do you want to bring?: ")
        # ----------------------

        if user_input.lower() == "q":
            self.get_logger().info("Quit the program...")
            sys.exit()

        if user_input:
            try:
                user_input_int = int(user_input)
                # --- [수정된 부분 3] ---
                user_input = ingredient_dict.get(user_input_int, user_input)
                # ----------------------
            except ValueError:
                pass  # 변환 불가능하면 원래 문자열 유지
            self.depth_request.target = user_input
            self.get_logger().info("call depth position service with yolo")
            depth_future = self.depth_client.call_async(self.depth_request)
            rclpy.spin_until_future_complete(self, depth_future)

            if depth_future.result():
                result = depth_future.result().depth_position.tolist()
                self.get_logger().info(f"Received depth position: {result}")
                if sum(result) == 0:
                    print("No target position")
                    return

                gripper2cam_path = os.path.join(
                    package_path, "resource", "T_gripper2camera.npy"
                )
                robot_posx = get_current_posx()[0]
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

                if td_coord[2] and sum(td_coord) != 0:
                    td_coord[2] += 50  # DEPTH_OFFSET
                    td_coord[2] = max(td_coord[2], 2)  # MIN_DEPTH: float = 2.0

                target_pos = list(td_coord[:3]) + robot_posx[3:]

                self.get_logger().info(f"target position: {target_pos}")
                self.pick_and_place_target(target_pos)
                self.init_robot()
        self.init_robot()

    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        # delete
        target_pos[3] += 10

        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()

        while gripper.get_status()[0]:
            time.sleep(0.5)

        # target_pos_up = trans(target_pos, [0, 0, 200, 0, 0, 0]).tolist()
        target_pos_up = trans(target_pos, [0, 0, 20, 0, 0, 0]).tolist()

        movel(target_pos_up, vel=VELOCITY, acc=ACC)
        # movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
        mwait()

        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)


def main(args=None):
    node = RobotController()
    while rclpy.ok():
        node.robot_control()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()