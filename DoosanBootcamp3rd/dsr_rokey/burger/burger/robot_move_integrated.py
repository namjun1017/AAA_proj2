import os
import time
import sys
import yaml
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

# YOLO 클래스 ID 매핑
ingredient_dict = {
    1: "bun_bottom",
    2: "bun_top",
    3: "cheese",
    4: "lettuce",
    5: "onion",
    6: "patty",
    7: "shrimp",
    8: "tomato",
}

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


# ================================
#     GRIPPER INITIALIZATION
# ================================
GRIPPER_NAME = "rg2"
TOOLCHANGER_IP = "192.168.1.1"
TOOLCHANGER_PORT = "502"
gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)


# ================================
#     ROBOT CONTROLLER CLASS
# ================================
class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place_integrated")

        # 중앙 설정 파일(recipes.yaml) 로드
        try:
            package_share_directory = os.path.expanduser("/home/rokey/nj_ws/src/AAA_proj2/DoosanBootcamp3rd/dsr_rokey/burger/burger") 
            config_path = os.path.join(package_share_directory, 'config', 'recipes.yaml')
            
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            # YAML 파일에서 레시피 정보 로드
            korean_menu_db = config['menu_db'] 
            self.ingredient_map_korean_to_yolo = config['ingredient_map_korean_to_yolo']

            # 로봇이 사용할 영문(YOLO) 레시피 DB를 동적으로 생성
            self.menu_db = {}
            for menu_name, ingredients in korean_menu_db.items():
                self.menu_db[menu_name] = [self.ingredient_map_korean_to_yolo.get(item, item) for item in ingredients]

            self.get_logger().info(f"Successfully loaded recipes from {config_path}")

        except (FileNotFoundError, yaml.YAMLError, KeyError) as e:
            self.get_logger().fatal(f"Failed to load or parse recipes.yaml: {e}")
            raise e

        self.order_queue = deque(maxlen=1)

        self.init_robot()

        self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.depth_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for depth position service...")
        self.depth_request = SrvDepthPosition.Request()

        self.order_subscription = self.create_subscription(
            Order, "/cmd", self.order_callback, QoSProfile(depth=10)
        )

        self.get_logger().info("Integrated Robot Controller is running.")

    def order_callback(self, msg):
        self.get_logger().info("New order received, adding to queue.")
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
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    # ====================================
    #      MAIN ORDER PROCESSING LOGIC
    # ====================================
    def robot_control(self):
        if not self.order_queue:
            return

        order = self.order_queue.popleft()
        self.get_logger().info(f"Processing order: {order.notes}")

        for burger in order.burgers:
            self.get_logger().info(f"--- Making a '{burger.menu_name}' ---")

            # ====================================
            #   🔥 핵심 수정 — 옵션을 bun_top 앞에 삽입
            # ====================================
            final_assembly_list = list(self.menu_db.get(burger.menu_name, []))

            for option in burger.options:
                self.get_logger().info(
                    f"[DEBUG] option.item={option.item}, type={option.type}, amount={option.amount}"
                )

                yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)
                if not yolo_item:
                    continue

                # ADD 옵션 : bun_top 바로 앞에 삽입
                if option.type == "add":
                    for _ in range(option.amount):
                        if "bun_top" in final_assembly_list:
                            idx = final_assembly_list.index("bun_top")
                            final_assembly_list.insert(idx, yolo_item)
                        else:
                            final_assembly_list.append(yolo_item)

                # REMOVE 옵션
                elif option.type == "remove":
                    for _ in range(option.amount):
                        if yolo_item in final_assembly_list:
                            final_assembly_list.remove(yolo_item)

            self.get_logger().info(f"Final Assembly list: {final_assembly_list}")

            # ====================================
            #           PICK & PLACE LOOP
            # ====================================
            for ingredient_name in final_assembly_list:

                self.get_logger().info(f"--- Picking ingredient: {ingredient_name} ---")

                self.depth_request.target = ingredient_name
                depth_future = self.depth_client.call_async(self.depth_request)
                rclpy.spin_until_future_complete(self, depth_future)

                if not (depth_future.result() and sum(depth_future.result().depth_position) != 0):
                    self.get_logger().error(f"Could not find '{ingredient_name}'. Skipping.")
                    continue

                result = depth_future.result().depth_position.tolist()
                self.get_logger().info(f"Received depth position: {result}")

                gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                robot_posx = get_current_posx()[0]
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

                td_coord[2] += 50
                td_coord[2] = max(td_coord[2], 2)

                target_pos = list(td_coord[:3]) + robot_posx[3:]

                self.get_logger().info(f"Target position: {target_pos}")
                self.pick_and_place_target(target_pos)
                self.init_robot()

            self.get_logger().info(f"--- Finished '{burger.menu_name}' ---")

    # ================================
    #            ROBOT MOVES
    # ================================
    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        pick_pos_above = trans(target_pos, [0, 0, 100, 0, 0, 0])
        movel(pick_pos_above, vel=VELOCITY, acc=ACC)
        mwait()
        movel(target_pos, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()
        gripper.close_gripper()
        time.sleep(1)
        movel(pick_pos_above, vel=VELOCITY, acc=ACC)
        mwait()

        self.get_logger().info(f"Placing ingredient at BUCKET_POS: {BUCKET_POS}")
        movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.open_gripper()
        time.sleep(1)


# ================================
#            MAIN LOOP
# ================================
def main(args=None):
    node = RobotController()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        node.robot_control()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()