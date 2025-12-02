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
            package_share_directory = os.path.expanduser("/home/hyochan/ros2_ws/src/AAA_proj2/DoosanBootcamp3rd/dsr_rokey/burger/burger") 
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
        # 기본 메뉴 구성
        self.menu_db = {
            "불고기버거": ["bun_bottom", "patty", "lettuce", "tomato", "bun_top"],
            "치즈버거": ["bun_bottom", "patty", "cheese", "bun_top"],
            "새우버거": ["bun_bottom", "shrimp", "lettuce", "bun_top"],
        }

        # 한국어 옵션 명 → YOLO 클래스로 매핑
        self.ingredient_map_korean_to_yolo = {
            "빵": "bun_bottom",
            "불고기": "patty",
            "치즈": "cheese",
            "상추": "lettuce",
            "토마토": "tomato",
            "새우": "shrimp",
            "번": "bun_bottom",
        }

        self.order_queue = deque(maxlen=1)

        self.init_robot()

        # YOLO 기반 3D 좌표 서비스
        self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        while not self.depth_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for depth position service...")
        self.depth_request = SrvDepthPosition.Request()

        # ArUco 마커 3D 좌표 서비스
        self.marker_client = self.create_client(SrvDepthPosition, "/get_marker_position")
        while not self.marker_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for marker position service...")
        self.marker_request = SrvDepthPosition.Request()

        # /cmd 토픽 구독
        self.order_subscription = self.create_subscription(
            Order, "/cmd", self.order_callback, QoSProfile(depth=10)
        )

        self.get_logger().info("Integrated Robot Controller is running.")

    ### 주문 콜백 ###
    def order_callback(self, msg):
        self.get_logger().info("New order received, adding to queue.")
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
    #      MAIN ORDER PROCESSING LOGIC
    # ====================================
    def sweep_find_marked_box(
        self,
        x=600.0,
        y_min=-100.0,
        y_max=100.0,
        y_step=50.0,
        z=200.0,
        rx=105.0,
        ry=180.0,
        rz=105.0,
        candidate_ids=None,
    ):
        """
        x=600, z=200, RPY=[105,180,105] 고정하고
        y를 y_min ~ y_max까지 y_step 간격으로 움직이며
        /get_marker_position 서비스로 마커를 탐색한다.
        """
        if candidate_ids is None:
            candidate_ids = [0, 1, 2, 3, 4, 5]

        detected_ids = []

        y = y_min
        while y <= y_max:
            target_pose = [x, y, z, rx, ry, rz]
            self.get_logger().info(f"[SWEEP] move to pose: {target_pose}")
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
                    self.get_logger().info(
                        f"[SWEEP] Marker {mid} detected at {list(cam_xyz)} (y={y})"
                    )
                    detected_ids.append(mid)

            if detected_ids:
                break

            y += y_step

        return detected_ids

    def move_to_marker(self, marker_id: int, counter_pos=None):
        """
        마커 id 위치로 가서 박스를 집어 counter_pos로 옮김.
        (마커 중심 좌표는 지금은 사용하지 않고, 박스는 counter_pos 로 놓는다.)
        """
        self.marker_request.target = str(marker_id)
        future = self.marker_client.call_async(self.marker_request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error("Marker service call failed")
            return

        cam_xyz = future.result().depth_position
        self.get_logger().info(f"[DEBUG] cam_xyz (camera coords): {list(cam_xyz)}")

        if len(cam_xyz) != 3 or sum(cam_xyz) == 0.0:
            self.get_logger().warn("Invalid marker position received")
            return

        robot_posx = get_current_posx()[0]
        self.get_logger().info(f"[DEBUG] robot_posx (current tcp): {robot_posx}")

        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
        base_xyz = self.transform_to_base(cam_xyz, gripper2cam_path, robot_posx)
        self.get_logger().info(f"[DEBUG] base_xyz (before offset): {base_xyz}")

        bx, by, bz = base_xyz

        # 박스 집기용 오프셋
        bx -= 50.0
        bz += 50.0

        target_pos = [bx, by, bz, robot_posx[3], robot_posx[4], robot_posx[5]]
        self.get_logger().info(f"[DEBUG] Final target_pos (TCP target): {target_pos}")\
        
        counter_box_pos = counter_pos.copy()
        counter_box_pos[0] -= 65
        counter_box_pos[2] += 50

        # 박스를 counter_pos로 옮김
        self.pick_and_place_target(target_pos,width_val=0,counter_pos = counter_box_pos)

        mwait()
        self.init_robot()

    def robot_control(self):
        if not self.order_queue:
            return

        # 2) 주문 꺼내기
        order = self.order_queue.popleft()
        self.get_logger().info(f"Processing order: {order.notes}")

        for burger_idx, burger in enumerate(order.burgers):
            self.get_logger().info(f"--- Making a '{burger.menu_name}' (idx={burger_idx}) ---")

            # x 위치: 버거 idx마다 150씩 증가
            counter_pos = COUNTER_POS.copy()
            counter_pos[0] += burger_idx * 200.0
            self.get_logger().info(f"[COUNTER POS] x={counter_pos[0]}, y={counter_pos[1]}")

            # 🔹 여기서 "이번 버거에 쓸 박스"를 새로 찾는다
            marker_ids = self.sweep_find_marked_box(
                x=600.0,
                y_min=-200.0,
                y_max=100.0,
                y_step=50.0,
                z=150.0,
                rx=105.0,
                ry=180.0,
                rz=105.0,
            )

            if not marker_ids:
                self.get_logger().warn("더 이상 사용할 박스가 없습니다. 남은 버거 작업을 중단합니다.")
                break

            # 마커 번호는 중요하지 않으니, 가장 먼저 찾은 것 하나만 사용
            marker = marker_ids[0]
            self.get_logger().info(
                f"[ROBOT_CONTROL] Using marker id: {marker} for burger idx {burger_idx}"
            )

            # 3) 박스를 먼저 해당 counter_pos로 옮기기
            self.move_to_marker(marker, counter_pos=counter_pos)

            # 4) 이 버거의 재료 스택 만들기
            final_assembly_list = list(self.menu_db.get(burger.menu_name, []))

            for option in burger.options:
                self.get_logger().info(
                    f"[DEBUG] option.item={option.item}, type={option.type}, amount={option.amount}"
                )

                yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)
                if not yolo_item:
                    continue

                if option.type == "add":
                    for _ in range(option.amount):
                        if "bun_top" in final_assembly_list:
                            idx = final_assembly_list.index("bun_top")
                            final_assembly_list.insert(idx, yolo_item)
                        else:
                            final_assembly_list.append(yolo_item)

                elif option.type == "remove":
                    for _ in range(option.amount):
                        if yolo_item in final_assembly_list:
                            final_assembly_list.remove(yolo_item)

            self.get_logger().info(f"Final Assembly list: {final_assembly_list}")

            # 5) 재료를 "방금 옮긴 박스 자리(counter_pos)" 위에 쌓기
            for idx, ingredient_name in enumerate(final_assembly_list):
                self.get_logger().info(f"--- Picking ingredient: {ingredient_name} ---")

                self.depth_request.target = ingredient_name

                self.get_logger().info(f"Calling depth service for '{ingredient_name}'")
                depth_future = self.depth_client.call_async(self.depth_request)
                rclpy.spin_until_future_complete(self, depth_future)

                if not depth_future.result() or sum(depth_future.result().depth_position) == 0:
                    self.get_logger().error(f"Could not find '{ingredient_name}'. Skipping.")
                    continue

                result = depth_future.result().depth_position.tolist()
                gripper2cam_path = os.path.join(
                    package_path, "resource", "T_gripper2camera.npy"
                )
                robot_posx = get_current_posx()[0]
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

                td_coord[1] -= 15
                td_coord[2] -= 10

                # 픽업용 target_pos (재료 있는 곳)
                target_pos = list(td_coord[:3]) + robot_posx[3:]
                counter_pos = COUNTER_POS.copy()
                counter_pos[0] += burger_idx * 200.0
                counter_pos[2] += idx * INGREDIENT_THICKNESS

                self.pick_and_place_target(target_pos,width_val=600,counter_pos=counter_pos)

                self.init_robot()

            self.get_logger().info(f"--- Finished '{burger.menu_name}' ---")


    # ================================
    #            ROBOT MOVES
    # ================================
    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.move_gripper(1000)  # 그리퍼 열기
        mwait()

    def pick_and_place_target(self, target_pos,width_val, counter_pos=None):
        if counter_pos is None:
            counter_pos = COUNTER_POS

        # 1) picking 위로 이동
        pick_pos_above = target_pos.copy()
        pick_pos_above[2] += 100.0

        # 픽업 단계
        movel(pick_pos_above, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        movel(target_pos, vel=VELOCITY / 3, acc=ACC / 3)
        mwait()

        gripper.move_gripper(width_val, force_val=100)  # 그리퍼 닫기
        time.sleep(3)

        movel(pick_pos_above, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        # 내려놓기용 상단 위치 만들기
        place_pos_above = counter_pos.copy()
        place_pos_above[2] += 100.0

        # 내려놓기 접근 = 위로 먼저 이동
        movel(place_pos_above, vel=VELOCITY, acc=ACC)
        mwait()

        # 내려놓기
        movel(counter_pos, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        gripper.move_gripper(750)  # 그리퍼 열기
        time.sleep(1)

        # 다시 위로 복귀
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
