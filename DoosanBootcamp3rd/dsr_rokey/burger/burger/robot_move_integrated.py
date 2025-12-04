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
from typing import List, Dict


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
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, DR_MV_MOD_REL
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
            package_share_directory = os.path.expanduser("/home/rokey/ros2_ws/src/AAA_proj2/DoosanBootcamp3rd/dsr_rokey/burger/burger") 
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

    def get_object_position(self, name: str):
        """YOLO 서비스로 재료의 카메라 좌표를 얻고 base로 변환한 뒤 반환(리스트)"""
        self.depth_request.target = name
        future = self.depth_client.call_async(self.depth_request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result() or sum(future.result().depth_position) == 0:
            self.get_logger().error(f"Depth service could not find {name}")
            return None

        cam_coords = future.result().depth_position
        # 변환을 위해 현재 TCP pose 가져오기
        robot_posx = get_current_posx()[0]
        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
        base_coord = self.transform_to_base(cam_coords, gripper2cam_path, robot_posx)
        return base_coord.tolist()

    def build_final_order_from_order(self, order: Order) -> Dict:
        """
        Order 메시지에서 각 버거별 final_assembly_list를 만들고,
        'burgers' 리스트와 'totals' 집계(dict)를 반환한다.

        반환 예:
        {
            "burgers": [
                {"id": 1, "counter_pos": <할당 전 None>, "box_stack_height": 0, "ingredients":[{"name":..., "qty":1}, ...]},
                ...
            ],
            "totals": {"bun_bottom":2, "patty":1, ...}
        }
        """
        burgers = []
        counter = Counter()

        for idx, burger in enumerate(order.burgers, start=1):
            # 기본 레시피
            final_assembly_list = list(self.menu_db.get(burger.menu_name, []))

            # 옵션 반영 (add/remove)
            for option in burger.options:
                yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)
                if not yolo_item:
                    continue

                if option.type == "add":
                    for _ in range(option.amount):
                        if "bun_top" in final_assembly_list:
                            insert_idx = final_assembly_list.index("bun_top")
                            final_assembly_list.insert(insert_idx, yolo_item)
                        else:
                            final_assembly_list.append(yolo_item)
                elif option.type == "remove":
                    for _ in range(option.amount):
                        if yolo_item in final_assembly_list:
                            final_assembly_list.remove(yolo_item)

            # ingredient entries with qty (currently each listed item is qty=1)
            ing_list = []
            for ing in final_assembly_list:
                ing_list.append({"name": ing, "qty": 1})
                counter[ing] += 1

            burgers.append({
                "id": idx,
                "counter_pos": None,            # prepare_boxes_for_order에서 채워질 값 (카운터상의 목표 pose)
                "marker_id": None,              # 사용한 마커 id
                "box_stack_height": 0,
                "ingredients": ing_list,
            })

        final_order = {"burgers": burgers, "totals": dict(counter)}
        return final_order
    
    def prepare_boxes_for_order(self, final_order: Dict):
        """
        각 버거(순서대로)에 대해 박스를 찾아서 counter 위로 옮긴다.
        - sweep_find_marked_box로 사용 가능한 마커를 찾고,
        - move_to_marker로 박스를 counter_pos으로 이동시킴.
        이후 final_order['burgers'][i]['counter_pos'] 와 marker_id를 설정.
        """
        for burger in final_order["burgers"]:
            bid = burger["id"]
            # counter 위치 결정 (버거 idx마다 x offset)
            counter_pos = COUNTER_POS.copy()
            counter_pos[0] += (bid - 1) * 200.0
            self.get_logger().info(f"[PREPARE] Allocating box for burger {bid} at counter_pos {counter_pos}")


            # 마커 스윕(1개만 필요)
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
                self.get_logger().warn("No available box found during prepare. Aborting remaining.")
                raise RuntimeError("Insufficient boxes")

            marker = marker_ids[0]
            self.get_logger().info(f"[PREPARE] Using marker {marker} for burger {bid}")

            # 박스를 가져와서 counter_pos로 옮김
            self.move_to_marker(marker, counter_pos=counter_pos)

            # move_to_marker 내부에서 init_robot도 호출되므로 여기서는 간단히 기록
            # 실제로는 box의 정확한 place pose를 marker->base 변환해서 저장하면 더 안전
            burger["counter_pos"] = counter_pos
            burger["marker_id"] = marker
            burger["box_stack_height"] = 0

            # 약간의 딜레이
            time.sleep(0.2)

    def assemble_from_totals(self, final_order: Dict):
        """
        final_order['totals'] 순회하면서 재료를 하나씩 획득하고,
        필요로 하는 버거 박스로 분배한다.
        박스별 box_stack_height를 올리고, counter_pos의 Z를 층수에 맞게 증가시켜 pick_and_place_target 호출.
        """
        burgers = final_order["burgers"]

        # ingredient -> deque(burger_id) mapping (분배 순서: 버거 순서대로)
        need_map = {}
        for b in burgers:
            bid = b["id"]
            for ing in b["ingredients"]:
                name = ing["name"]
                need_map.setdefault(name, deque()).append(bid)

        self.get_logger().info(f"[ASSEMBLE] Totals: {final_order['totals']}")

        # 재료별로 총 수량만큼 반복
        for ing_name, total_qty in final_order["totals"].items():
            self.get_logger().info(f"[ASSEMBLE] Processing ingredient '{ing_name}' x {total_qty}")

            # 1) 재료 위치(베이스 좌표) 얻기
            base_obj = self.get_object_position(ing_name)
            if base_obj is None:
                self.get_logger().error(f"Failed to get position of {ing_name}. Skipping this unit.")
                continue

            # 2) 재료가 필요한 모든 버거의 counter_pos 가져오기
            counter_info = []
            for burger in final_order["burgers"]:

                ing_qty_in_burger = sum(
                    ing["qty"] for ing in burger["ingredients"]
                    if ing["name"] == ing_name
                )
                # 이 버거에 해당 재료가 있으면
                if any(ing["name"] == ing_name for ing in burger["ingredients"]):
                    # counter_pos 확인
                    if burger.get("counter_pos") is not None:
                        counter_info.append({
                            "pos": burger["counter_pos"],
                            "qty": ing_qty_in_burger,
                            "burger": burger
                        })
                    else:
                        self.get_logger().warn(f"Burger {burger['id']} does not have counter_pos set.")
            
            # 3) target pos 지정하기
            robot_posx = get_current_posx()[0]
            target_pos = list(base_obj[:3]) + robot_posx[3:]

            width_val = 600


            try:
                self.pick_and_place_target(target_pos,total_qty, width_val=width_val, counter_positions=counter_info)
            except Exception as e:
                self.get_logger().error(f"pick_and_place_target failed: {e}")
                # 재시도 로직 또는 스킵 처리 가능
                continue

            self.init_robot()
            time.sleep(0.1)
            
    
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
        bz += 60.0

        target_pos = [bx, by, bz, robot_posx[3], robot_posx[4], robot_posx[5]]
        self.get_logger().info(f"[DEBUG] Final target_pos (TCP target): {target_pos}")\
        
        counter_box_pos = counter_pos.copy()
        counter_box_pos[0] -= 65
        counter_box_pos[2] += 50

        counter_info = [{
            "pos": counter_box_pos,
            "qty": 1,      # 박스는 항상 1개
            "burger": {"box_stack_height": 0}  # 임시 버거 정보
        }]

        # 박스를 counter_pos로 옮김
        self.pick_and_place_target(target_pos,1, width_val=0, counter_positions= counter_info)

        mwait()
        self.init_robot()

    def robot_control(self):
        if not self.order_queue:
            return

        # 1) 주문 꺼내기
        order = self.order_queue.popleft()
        self.get_logger().info(f"Processing order: {order.notes}")

        try:
            # 2) final_order 빌드 (버거별 재료 + totals)
            final_order = self.build_final_order_from_order(order)
            self.get_logger().info(f"[ORDER] Final order totals: {final_order['totals']}")

            # 3) 각 버거 박스 미리 찾아서 counter로 배치 (박스당 counter_pos, marker_id, stack 초기화)
            self.prepare_boxes_for_order(final_order)

            # 4) totals 기준으로 재료를 하나씩 가져와 각 박스에 분배
            self.assemble_from_totals(final_order)

            self.get_logger().info("Order assembly completed.")

        except Exception as e:
            self.get_logger().error(f"Failed processing order: {e}")
            # 필요시 재시도/롤백 로직

        


    # ================================
    #            ROBOT MOVES
    # ================================
    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel([0,-100,0,0,0,0],vel=VELOCITY, acc= ACC,mod=DR_MV_MOD_REL)
        gripper.move_gripper(1000)  # 그리퍼 열기
        mwait()

    def pick_and_place_target(self, target_pos, total_qty, width_val, counter_positions):
        if not counter_positions:
            self.get_logger().warn("No counter positions provided. Using default COUNTER_POS.")

        # --- 1) picking 위로 이동 ---
        pick_pos_above = target_pos.copy()
        pick_pos_above[2] += 100.0
        movel(pick_pos_above, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        # --- 2) 재료 개수에 따라 깊게 집기 ---
        pick_pos_deep = target_pos.copy()
        pick_pos_deep[2] -= 3
        pick_pos_deep[2] -= total_qty * 10  # total_qty에 비례하여 깊이 조절
        movel(pick_pos_deep, vel=VELOCITY / 3, acc=ACC / 3)
        mwait()

        # --- 3) 그리퍼 닫기 ---
        gripper.move_gripper(width_val, force_val=100)
        time.sleep(3)

        # --- 4) 집은 위치 위로 복귀 ---
        movel(pick_pos_above, vel=VELOCITY / 2, acc=ACC / 2)
        mwait()

        # --- 5) 각 counter_position에 재료를 순서대로 내려놓기 ---
        for idx, info in enumerate(counter_positions):
            pos = info["pos"]
            qty_for_this_burger = info["qty"]
            burger = info["burger"]

            place_pos = pos.copy()
            place_pos[2] += burger["box_stack_height"] * 10.0
            
            place_pos_above = place_pos.copy()
            place_pos_above[2] += 100.0
            self.get_logger().info(f"[ORDER] Final order totals: {pos},{qty_for_this_burger},{burger}")


            # 접근
            movel(place_pos_above, vel=VELOCITY, acc=ACC)
            mwait()

            movel(place_pos, vel=VELOCITY / 2, acc=ACC / 2)
            mwait()

            # 그리퍼 열기
            gripper.move_gripper(750)
            time.sleep(1)

            burger["box_stack_height"] +=qty_for_this_burger

            if idx < len(counter_positions) - 1:
                movel([0,0,+10*qty_for_this_burger,0,0,0],vel=VELOCITY, acc= ACC,mod=DR_MV_MOD_REL)
                gripper.move_gripper(width_val)
                mwait()
            
            movel([0,0,+100,0,0,0],vel=VELOCITY, acc= ACC,mod=DR_MV_MOD_REL)




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
g