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

import cv2
import cv2.aruco as aruco

package_path = get_package_share_directory("burger")

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]
COUNTER_POS = [215, 250, 5, 105, 180, 105]
INGREDIENT_THICKNESS = 20.0

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


########### Robot Controller ############

class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place_integrated")

        # 중앙 설정 파일(recipes.yaml) 로드
        try:
            package_share_directory = os.path.expanduser(
                "/home/changbeom/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/burger/burger"
            )
            config_path = os.path.join(package_share_directory, 'config', 'recipes.yaml')

            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)

            korean_menu_db = config['menu_db']
            self.ingredient_map_korean_to_yolo = config['ingredient_map_korean_to_yolo']

            self.menu_db = {}
            for menu_name, ingredients in korean_menu_db.items():
                self.menu_db[menu_name] = [
                    self.ingredient_map_korean_to_yolo.get(item, item)
                    for item in ingredients
                ]

            self.get_logger().info(f"Successfully loaded recipes.")

        except Exception as e:
            self.get_logger().fatal(f"Failed to load recipes: {e}")
            raise e

        self.order_queue = deque(maxlen=1)
        self.init_robot()

        # YOLO 기반 3D 좌표 서비스
        self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.depth_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for depth position service...")
        self.depth_request = SrvDepthPosition.Request()

        # ArUco 마커 서비스
        self.marker_client = self.create_client(SrvDepthPosition, "/get_marker_position")
        while not self.marker_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for marker position service...")
        self.marker_request = SrvDepthPosition.Request()

        self.order_subscription = self.create_subscription(
            Order, "/cmd", self.order_callback, QoSProfile(depth=10)
        )

        self.get_logger().info("Integrated Robot Controller is running.")


    ### 주문 콜백 ###
    def order_callback(self, msg):
        self.order_queue.append(msg)

    ### Pose 변환 ###
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
    #     SWEEP FOR BOX MARKERS
    # ====================================
    def sweep_find_marked_box(
        self, x=600.0, y_min=-100.0, y_max=100.0, y_step=50.0,
        z=200.0, rx=105.0, ry=180.0, rz=105.0, candidate_ids=None):

        if candidate_ids is None:
            candidate_ids = [0, 1, 2, 3, 4, 5]

        detected_ids = []

        y = y_min
        while y <= y_max:
            target_pose = [x, y, z, rx, ry, rz]
            movel(target_pose, vel=VELOCITY, acc=ACC)
            mwait()
            time.sleep(0.5)

            for mid in candidate_ids:
                self.marker_request.target = str(mid)
                future = self.marker_client.call_async(self.marker_request)
                rclpy.spin_until_future_complete(self, future)
                if not future.result():
                    continue

                cam_xyz = future.result().depth_position
                if len(cam_xyz) == 3 and sum(cam_xyz) != 0.0:
                    detected_ids.append(mid)

            if detected_ids:
                break

            y += y_step

        return detected_ids


    # ====================================
    #  박스 1개를 카운터로 옮기는 함수
    # ====================================
    def move_to_marker(self, marker_id: int, counter_pos=None):

        # 1) 마커 좌표 읽기
        self.marker_request.target = str(marker_id)
        future = self.marker_client.call_async(self.marker_request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            return

        cam_xyz = future.result().depth_position

        if len(cam_xyz) != 3 or sum(cam_xyz) == 0.0:
            self.get_logger().warn("Invalid marker position received")
            return

        robot_posx = get_current_posx()[0]
        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")

        # 2) transform
        base_xyz = self.transform_to_base(cam_xyz, gripper2cam_path, robot_posx)
        bx, by, bz = base_xyz

        # 박스 픽업 offset
        bx -= 50
        bz += 50

        target_pos = [bx, by, bz, robot_posx[3], robot_posx[4], robot_posx[5]]

        # counter pos offset
        counter_box_pos = counter_pos.copy()
        counter_box_pos[0] -= 65
        counter_box_pos[2] += 50

        # 박스 픽업 → counter_pos로 옮김
        self.pick_and_place_target(target_pos, width_val=0, counter_pos=counter_box_pos)

        self.init_robot()


    # ====================================
    #        🔥 핵심: 전체 로직 재구성
    # ====================================
    def robot_control(self):
        if not self.order_queue:
            return

        order = self.order_queue.popleft()
        burgers = order.burgers
        num_burgers = len(burgers)

        # ================================
        #   1) 주문 수량만큼 박스를 먼저 전부 배치
        # ================================
        box_positions = []

        for i in range(num_burgers):
            counter_pos_i = COUNTER_POS.copy()
            counter_pos_i[0] += i * 200.0

            marker_ids = self.sweep_find_marked_box(
                x=600.0, y_min=-200.0, y_max=100.0, y_step=50.0,
                z=150.0, rx=105.0, ry=180.0, rz=105.0
            )

            if not marker_ids:
                self.get_logger().warn("더 이상 사용할 박스가 없습니다.")
                break

            marker = marker_ids[0]

            self.move_to_marker(marker, counter_pos=counter_pos_i)

            box_positions.append(counter_pos_i.copy())

        if len(box_positions) != num_burgers:
            return

        # ================================
        #   2) 각 박스 위에 각 버거 재료 쌓기
        # ================================
        for b_idx, burger in enumerate(burgers):
            final_list = list(self.menu_db[burger.menu_name])

            # 옵션 처리
            for option in burger.options:
                yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)
                if not yolo_item:
                    continue

                if option.type == "add":
                    for _ in range(option.amount):
                        if "bun_top" in final_list:
                            idx_bt = final_list.index("bun_top")
                            final_list.insert(idx_bt, yolo_item)
                        else:
                            final_list.append(yolo_item)

                elif option.type == "remove":
                    for _ in range(option.amount):
                        if yolo_item in final_list:
                            final_list.remove(yolo_item)

            # 현재 박스 위치
            base_box_pos = box_positions[b_idx]

            # 재료 쌓기
            for layer, ingredient in enumerate(final_list):
                self.depth_request.target = ingredient
                depth_future = self.depth_client.call_async(self.depth_request)
                rclpy.spin_until_future_complete(self, depth_future)
                if not depth_future.result() or sum(depth_future.result().depth_position) == 0:
                    continue

                result = depth_future.result().depth_position.tolist()
                robot_posx = get_current_posx()[0]

                gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

                td_coord[1] -= 15
                td_coord[2] -= 10

                target_pos = list(td_coord[:3]) + robot_posx[3:]

                # 박스 위 층 쌓기
                place_pos = base_box_pos.copy()
                place_pos[2] += layer * INGREDIENT_THICKNESS

                self.pick_and_place_target(target_pos, width_val=600, counter_pos=place_pos)

                self.init_robot()


    # ================================
    #            ROBOT MOVES
    # ================================
    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.move_gripper(1000)
        mwait()

    def pick_and_place_target(self, target_pos, width_val, counter_pos=None):
        if counter_pos is None:
            counter_pos = COUNTER_POS

        pick_pos_above = target_pos.copy()
        pick_pos_above[2] += 100.0

        movel(pick_pos_above, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        movel(target_pos, vel=VELOCITY / 3, acc=ACC / 3)
        mwait()

        gripper.move_gripper(width_val, force_val=100)
        time.sleep(3)

        movel(pick_pos_above, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        place_pos_above = counter_pos.copy()
        place_pos_above[2] += 100.0

        movel(place_pos_above, vel=VELOCITY, acc=ACC)
        mwait()

        movel(counter_pos, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        gripper.move_gripper(750)
        time.sleep(1)

        movel(place_pos_above, vel=VELOCITY, acc=ACC)
        mwait()


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
