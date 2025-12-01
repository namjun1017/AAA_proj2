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
COUNTER_POS = [300, 100.6, 50, 115, 180, 115]
INGREDIENT_THICKNESS = 50


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


########### Robot Controller (VideoCapture 제거 + 서비스 기반 ArUco 검출) ###########

class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place_integrated")

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

        # ✅ ArUco 마커 3D 좌표 서비스 (여기가 새로 추가된 부분)
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
    
    def find_marked_box(self, candidate_ids=None):
        """
        VideoCapture 없이 ArUco를 찾는다.
        /get_marker_position 서비스를 여러 ID에 대해 호출해서
        실제 보이는 마커 ID 리스트를 반환한다.
        """

        if candidate_ids is None:
            candidate_ids = [0, 1, 2, 3, 4, 5]

        detected_ids = []

        for mid in candidate_ids:
            self.marker_request.target = str(mid)
            future = self.marker_client.call_async(self.marker_request)
            rclpy.spin_until_future_complete(self, future)

            if not future.result():
                continue

            cam_xyz = future.result().depth_position

            if len(cam_xyz) == 3 and sum(cam_xyz) != 0.0:
                self.get_logger().info(f"Marker {mid} detected at {cam_xyz}")
                detected_ids.append(mid)

        return detected_ids

    def move_to_marker(self, marker_id: int):
        # 서비스 요청
        self.marker_request.target = str(marker_id)
        future = self.marker_client.call_async(self.marker_request)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error("Marker service call failed")
            return

        # 카메라 좌표 (정상 값)
        cam_xyz = future.result().depth_position
        self.get_logger().info(f"[DEBUG] cam_xyz (camera coords): {list(cam_xyz)}")

        if len(cam_xyz) != 3 or sum(cam_xyz) == 0.0:
            self.get_logger().warn("Invalid marker position received")
            return

        # 현재 TCP pose
        robot_posx = get_current_posx()[0]
        self.get_logger().info(f"[DEBUG] robot_posx (current tcp): {robot_posx}")

        # 변환 행렬 로드
        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")

        # 카메라 좌표 -> 베이스 좌표 변환
        base_xyz = self.transform_to_base(cam_xyz, gripper2cam_path, robot_posx)
        self.get_logger().info(f"[DEBUG] base_xyz (before offset): {base_xyz}")

        # base_xyz: [x, y, z]
        bx, by, bz = base_xyz

        bx -= 40.0
        bz+= 50

        # 최종 TCP 위치: [x, y, z, rx, ry, rz]
        target_pos = [bx, by, bz, robot_posx[3], robot_posx[4], robot_posx[5]]

        self.get_logger().info(f"[DEBUG] Final target_pos (TCP target): {target_pos}")

        self.pick_and_place_target(target_pos)
        
        #movel(COUNTER_POS, vel=VELOCITY, acc=ACC)
        mwait()




    ###################################################################
    # 🔥 메인 로봇 제어 — 첫 단계에서 마커 인식
    ###################################################################
    def robot_control(self):
        print("1")

        marker_ids = self.find_marked_box()

        if not marker_ids:
            self.get_logger().warn("No markers detected.")
            return

        # 첫 번째 마커 사용
        marker = marker_ids[0]
        self.move_to_marker(marker)

        # ---------- 주문 로직 ----------
        if not self.order_queue:
            return

        # (기존 코드 유지 — 삭제 X)
        self.marker_client = self.create_client(SrvDepthPosition, "/get_marker_position")
        while not self.marker_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for marker position service...")
        self.marker_request = SrvDepthPosition.Request()

        order = self.order_queue.popleft()
        self.get_logger().info(f"Processing order: {order.notes}")

        # 이하 버거 조립 로직은 동일
        for burger in order.burgers:
            self.get_logger().info(f"--- Making a '{burger.menu_name}' ---")

            base_ingredients = self.menu_db.get(burger.menu_name, [])
            ingredient_counts = Counter(base_ingredients)
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
                gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
                robot_posx = get_current_posx()[0]
                td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

                td_coord[2] += 50
                td_coord[2] = max(td_coord[2], 2)

                target_pos = list(td_coord[:3]) + robot_posx[3:]
                counter_pos = COUNTER_POS.copy()
                counter_pos[2] += idx * INGREDIENT_THICKNESS

                self.pick_and_place_target(target_pos,counter_pos)
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

    def pick_and_place_target(self, target_pos, counter_pos):
        # target_pos: [x, y, z, rx, ry, rz]

        # 1) 위에서 한 번 대기하는 위치 (베이스 좌표계 z만 +100 mm)
        pick_pos_above = target_pos.copy()
        pick_pos_above[2] += 100.0

        self.get_logger().info(f"[PICK] target_pos: {target_pos}")
        self.get_logger().info(f"[PICK] pick_pos_above (z+100): {pick_pos_above}")

        # 2) 먼저 위로 이동
        movel(pick_pos_above, vel=VELOCITY/2, acc=ACC/2)
        mwait()
        time.sleep(0.3)  # 동작이 눈에 보이게 잠깐 멈춤 (원하면 나중에 지워도 됨)

        # 3) 아래로 내려가서 집기
        movel(target_pos, vel=VELOCITY/3, acc=ACC/3)
        mwait()
        gripper.close_gripper()
        time.sleep(1)

        # 4) 다시 위로 올리기
        movel(pick_pos_above, vel=VELOCITY/2, acc=ACC/2)
        mwait()

        counter_pos_above = counter_pos.copy()
        counter_pos_above[2] += 100.0
 
        # 5) 카운터로 이동해서 내려놓기
        movel(counter_pos_above, vel=VELOCITY/2, acc=ACC/2)
        mwait()
        time.sleep(1)

        movel(counter_pos, vel=VELOCITY/3, acc=ACC/3)
        gripper.open_gripper()


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
