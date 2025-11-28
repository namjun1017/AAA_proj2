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
from order_interfaces.msg import Order # Custom Message 임포트

# 🚨 DSR_ROBOT2 함수들을 전역에서 제거합니다. main 함수에서 임포트됩니다.
# try...except 블록을 제거하고, main 함수에서 임포트됩니다.

package_path = get_package_share_directory("pick_and_place_text")

# --- [수정된 부분 1] ---
ingredient_dict = {1: "bun_bottom", 2: "bun_top", 3: "cheese", 4: "lettuce", 5: "onion", 6: "patty", 7: "shrimp", 8: "tomato"}
# ----------------------

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 🚨 [삭제] 전역 rclpy.init(), dsr_node 생성 블록은 삭제되었습니다.

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
        qos = QoSProfile(depth=1)
        self.order_sub = self.create_subscription(
            Order,
            '/cmd',
            self.cmd_callback,
            qos
        )
        self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        
        while not self.depth_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for depth position service...")
        self.depth_request = SrvDepthPosition.Request()
        
        # 🚨 [삭제] robot_control() 초기 호출은 main 루프가 처리합니다.

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

    def robot_control(self):
        """
        main 루프에 의해 반복 호출되며, pending_tasks에서 하나의 재료를 꺼내 처리합니다.
        """
        # 🚨 [핵심 수정 1] 큐가 비어있는지 확인. 비어있으면 즉시 반환 (IndexError 방지).
        if not self.pending_tasks:
            return 
            
        # 🚨 [핵심 수정 2] 작업 목록에서 하나의 작업 딕셔너리를 꺼냅니다. (FIFO)
        current_task = self.pending_tasks.pop(0) 
        target_item = current_task['item'] # YOLO 클래스명 (예: 'patty')

        # --- [수정된 부분 2] ---
        print("====================================")
        print(f"Executing Task: {target_item} for {current_task['menu']}")
        print("====================================")
        # ----------------------

        # 1. YOLO 서비스 요청 타겟 설정
        self.depth_request.target = target_item 
        self.get_logger().info(f"Calling depth position service for {target_item}")
        depth_future = self.depth_client.call_async(self.depth_request)
        rclpy.spin_until_future_complete(self, depth_future)

        if depth_future.result():
            result = depth_future.result().depth_position.tolist()
            
            if sum(result) == 0:
                self.get_logger().warn(f"No target position found for {target_item}. Skipping this task.")
                return

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            # 🚨 DSR 함수는 main에서 임포트되어 전역으로 사용됨
            robot_posx = get_current_posx()[0] 
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += 50
                td_coord[2] = max(td_coord[2], 2)

            target_pos = list(td_coord[:3]) + robot_posx[3:]

            self.get_logger().info(f"Target position: {target_pos}")
            self.pick_and_place_target(target_pos)
            
        else:
            self.get_logger().error(f"Failed to call depth position service for {target_item}")
            
        self.init_robot() 


    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def cmd_callback(self, msg: Order):
                self.get_logger().info("--- Received New Order ---")
                
                all_tasks = [] 
                ingredient_counts = Counter() 
                
                for item_instance in msg.burgers:
                    menu_name = item_instance.menu_name
                    
                    base_ingredients_raw = self.menu_db.get(menu_name, []) 
                    final_ingredients = list(base_ingredients_raw) 
                    
                    for option in item_instance.options:
                        item = option.item 
                        op_type = option.type
                        amount = option.amount
                        
                        if op_type == 'remove':
                            for i in range(amount):
                                try: final_ingredients.remove(item)
                                except ValueError: pass 
                                    
                        elif op_type == 'add':
                            for i in range(amount):
                                final_ingredients.insert(len(final_ingredients) - 1, item)

                    
                    self.get_logger().info(f"Final Ingredient List for {menu_name}: {final_ingredients}")
                    
                    if final_ingredients and final_ingredients[-1] == '빵':
                        final_ingredients[-1] = 'bun_top'
                    
                    instance_tasks = []
                    for ingredient_raw in final_ingredients:
                        yolo_class = self.ingredient_map.get(ingredient_raw, None)
                        
                        if yolo_class:
                            ingredient_counts[yolo_class] += 1 
                            
                            instance_tasks.append({
                                'menu': menu_name,
                                'item': yolo_class,
                                'type': 'add' 
                            })
                        else:
                            self.get_logger().warn(f"Unknown ingredient '{ingredient_raw}' skipped.")
                    
                    all_tasks.extend(instance_tasks)
                
                # 🚨 4. 전체 작업 목록 저장
                self.pending_tasks = list(all_tasks) 
                
                self.get_logger().info("\n==================================")
                self.get_logger().info(f"✅ SUMMARY: TOTAL PICKUP INGREDIENTS")
                for item, count in ingredient_counts.items():
                    self.get_logger().info(f"  - {item}: {count} ea")
                self.get_logger().info("----------------------------------")
                self.get_logger().info(f"Total {len(all_tasks)} pick/place tasks generated. Notes: {msg.notes}")
                self.get_logger().info("==================================")
                

    def pick_and_place_target(self, target_pos):
        target_pos[3] += 10

        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        gripper.close_gripper()

        while gripper.get_status()[0]:
            time.sleep(0.5)

        target_pos_up = trans(target_pos, [0, 0, 20, 0, 0, 0]).tolist()

        movel(target_pos_up, vel=VELOCITY, acc=ACC)
        mwait()

        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)


def main(args=None):
    # 🚨 [수정 1] rclpy.init()은 여기서 한 번만 호출되어야 합니다.
    rclpy.init(args=args) 
    
    node = RobotController()
    
    # 🚨 [수정 2] DSR_ROBOT2 임포트 지연 및 노드 할당
    # DSR_ROBOT2 초기화가 노드 객체 생성 후에 이루어지도록 순서를 강제합니다.
    DR_init.__dsr__node = node 
    try:
        global movej, movel, get_current_posx, mwait, trans
        from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2 in main: {e}")
        rclpy.shutdown()
        return

    try:
        # 🚨 [유지] main 루프는 robot_control을 반복 호출하여 작업을 소모합니다.
        while rclpy.ok():
            node.robot_control()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()