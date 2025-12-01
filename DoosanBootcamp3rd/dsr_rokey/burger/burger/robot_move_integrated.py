# # # import os
# # # import time
# # # import sys
# # # from scipy.spatial.transform import Rotation
# # # import numpy as np
# # # import rclpy
# # # from rclpy.node import Node
# # # from rclpy.qos import QoSProfile
# # # import DR_init
# # # from collections import Counter, deque

# # # from od_msg.srv import SrvDepthPosition
# # # from ament_index_python.packages import get_package_share_directory
# # # from burger.onrobot import RG
# # # from order_interfaces.msg import Order

# # # package_path = get_package_share_directory("burger")

# # # # --- [수정된 부분 1] ---
# # # # [주의] 아래 목록은 학습된 YOLO 모델의 클래스 이름과 정확히 일치해야 합니다.
# # # # 모델에 있는 실제 클래스 이름으로 수정해주세요. 예: 'patty', 'lettuce', 'tomato' 등
# # # ingredient_dict = {1: "bun_bottom", 2: "bun_top", 3: "cheese", 4: "lettuce", 5: "onion", 6: "patty", 7: "shrimp", 8: "tomato"}
# # # # ----------------------

# # # # for single robot
# # # ROBOT_ID = "dsr01"
# # # ROBOT_MODEL = "m0609"
# # # VELOCITY, ACC = 60, 60
# # # BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]

# # # DR_init.__dsr__id = ROBOT_ID
# # # DR_init.__dsr__model = ROBOT_MODEL

# # # rclpy.init()
# # # dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
# # # DR_init.__dsr__node = dsr_node

# # # try:
# # #     from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
# # # except ImportError as e:
# # #     print(f"Error importing DSR_ROBOT2: {e}")
# # #     sys.exit()

# # # ########### Gripper Setup. Do not modify this area ############

# # # GRIPPER_NAME = "rg2"
# # # TOOLCHANGER_IP = "192.168.1.1"
# # # TOOLCHANGER_PORT = "502"
# # # gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)


# # # ########### Robot Controller ############


# # # class RobotController(Node):
# # #     def __init__(self):
# # #         super().__init__("pick_and_place_integrated")

# # #         # 메뉴 DB와 재료 이름 매핑
# # #         self.menu_db = {
# # #             "불고기버거": ["bun_bottom", "patty", "lettuce", "tomato", "bun_top"],
# # #             "치즈버거": ["bun_bottom", "patty", "cheese", "bun_top"],
# # #             "새우버거": ["bun_bottom", "shrimp", "lettuce", "bun_top"],
# # #         }
# # #         self.ingredient_map_korean_to_yolo = {
# # #             "빵": "bun_bottom", "불고기": "patty", "치즈": "cheese", "상추": "lettuce",
# # #             "토마토": "tomato", "새우": "shrimp", "번": "bun_bottom"
# # #         }
        
# # #         # ROS 토픽에서 받은 주문을 저장할 큐
# # #         self.order_queue = deque(maxlen=1)

# # #         self.init_robot()
# # #         self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
# # #         while not self.depth_client.wait_for_service(timeout_sec=3.0):
# # #             self.get_logger().info("Waiting for depth position service...")
# # #         self.depth_request = SrvDepthPosition.Request()

# # #         # /cmd 토픽 구독자 설정
# # #         self.order_subscription = self.create_subscription(
# # #             Order, '/cmd', self.order_callback, QoSProfile(depth=10)
# # #         )
# # #         self.get_logger().info("Integrated Robot Controller is running.")

# # #     def order_callback(self, msg):
# # #         """ /cmd 토픽 메시지를 받으면 큐에 추가 """
# # #         self.get_logger().info("New order received, adding to queue.")
# # #         self.order_queue.append(msg)

# # #     def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
# # #         R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
# # #         T = np.eye(4)
# # #         T[:3, :3] = R
# # #         T[:3, 3] = [x, y, z]
# # #         return T

# # #     def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
# # #         gripper2cam = np.load(gripper2cam_path)
# # #         coord = np.append(np.array(camera_coords), 1)
# # #         x, y, z, rx, ry, rz = robot_pos
# # #         base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
# # #         base2cam = base2gripper @ gripper2cam
# # #         td_coord = np.dot(base2cam, coord)
# # #         return td_coord[:3]

# # #     def robot_control(self):
# # #         # 큐에 처리할 주문이 없으면 아무것도 하지 않고 반환
# # #         if not self.order_queue:
# # #             return

# # #         # 큐에서 주문을 하나 꺼냄
# # #         order = self.order_queue.popleft()
# # #         self.get_logger().info(f"Processing order: {order.notes}")

# # #         # 주문에 포함된 모든 버거를 순서대로 조립
# # #         for burger in order.burgers:
# # #             self.get_logger().info(f"--- Making a '{burger.menu_name}' ---")
            
# # #             # 조립할 재료 목록 생성
# # #             base_ingredients = self.menu_db.get(burger.menu_name, [])
# # #             ingredient_counts = Counter(base_ingredients)
# # #             for option in burger.options:
# # #                 yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)
# # #                 if yolo_item:
# # #                     if option.type == 'add':
# # #                         ingredient_counts[yolo_item] += option.amount
# # #                     elif option.type == 'remove':
# # #                         ingredient_counts[yolo_item] = max(0, ingredient_counts[yolo_item] - option.amount)
            
# # #             final_assembly_list = []
# # #             for item in base_ingredients:
# # #                  count = ingredient_counts.pop(item, 0)
# # #                  if count > 0:
# # #                     final_assembly_list.extend([item] * count)
# # #             for item, count in ingredient_counts.items():
# # #                 if count > 0:
# # #                     final_assembly_list.extend([item] * count)

# # #             self.get_logger().info(f"Assembly list: {final_assembly_list}")

# # #             # 각 재료를 순서대로 집어서 놓기
# # #             for ingredient_name in final_assembly_list:
# # #                 self.get_logger().info(f"--- Picking ingredient: {ingredient_name} ---")
                
# # #                 # 원본 코드의 로직을 재사용하여 재료 하나를 처리
# # #                 self.depth_request.target = ingredient_name
                
# # #                 # 서비스 호출 및 결과 대기 (spin_until_future_complete는 블로킹 방식)
# # #                 self.get_logger().info(f"Calling depth service for '{ingredient_name}'")
# # #                 depth_future = self.depth_client.call_async(self.depth_request)
# # #                 rclpy.spin_until_future_complete(self, depth_future)

# # #                 if not (depth_future.result() and sum(depth_future.result().depth_position) != 0):
# # #                     self.get_logger().error(f"Could not find '{ingredient_name}'. Skipping.")
# # #                     continue

# # #                 result = depth_future.result().depth_position.tolist()
# # #                 self.get_logger().info(f"Received depth position: {result}")

# # #                 gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
# # #                 robot_posx = get_current_posx()[0]
# # #                 td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

# # #                 td_coord[2] += 50  # DEPTH_OFFSET
# # #                 td_coord[2] = max(td_coord[2], 2)  # MIN_DEPTH

# # #                 target_pos = list(td_coord[:3]) + robot_posx[3:]

# # #                 self.get_logger().info(f"Target position: {target_pos}")
# # #                 self.pick_and_place_target(target_pos)
# # #                 self.init_robot()
            
# # #             self.get_logger().info(f"--- Finished '{burger.menu_name}' ---")


# # #     def init_robot(self):
# # #         JReady = [0, 0, 90, 0, 90, 0]
# # #         movej(JReady, vel=VELOCITY, acc=ACC)
# # #         gripper.open_gripper()
# # #         mwait()

# # #     def pick_and_place_target(self, target_pos):
# # #         # 이 함수는 하나의 재료를 집어서 고정된 위치(BUCKET_POS)에 놓는 동작을 가정합니다.
# # #         # 간단한 시연을 위해, 집은 후 바로 BUCKET_POS로 이동합니다.
        
# # #         # 집기
# # #         pick_pos_above = trans(target_pos, [0, 0, 100, 0, 0, 0])
# # #         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
# # #         mwait()
# # #         movel(target_pos, vel=VELOCITY/2, acc=ACC/2)
# # #         mwait()
# # #         gripper.close_gripper()
# # #         time.sleep(1)
# # #         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
# # #         mwait()

# # #         # 놓기 (고정된 위치)
# # #         self.get_logger().info(f"Placing ingredient at BUCKET_POS: {BUCKET_POS}")
# # #         movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
# # #         mwait()
# # #         gripper.open_gripper()
# # #         time.sleep(1)


# # # def main(args=None):
# # #     node = RobotController()
# # #     # 메인 루프: rclpy.ok()가 참인 동안 계속 실행
# # #     while rclpy.ok():
# # #         # spin_once를 호출하여 콜백 처리 (논블로킹)
# # #         rclpy.spin_once(node, timeout_sec=0.1)
# # #         # 로봇 제어 로직 실행
# # #         node.robot_control()

# # #     node.destroy_node()
# # #     rclpy.shutdown()


# # # if __name__ == "__main__":
# # #     main()
# # import os
# # import time
# # import sys
# # from scipy.spatial.transform import Rotation
# # import numpy as np
# # import rclpy
# # from rclpy.node import Node
# # from rclpy.qos import QoSProfile
# # import DR_init
# # from collections import Counter, deque

# # from od_msg.srv import SrvDepthPosition
# # from ament_index_python.packages import get_package_share_directory
# # from burger.onrobot import RG
# # from order_interfaces.msg import Order

# # package_path = get_package_share_directory("burger")

# # # --- [수정된 부분 1] ---
# # # [주의] 아래 목록은 학습된 YOLO 모델의 클래스 이름과 정확히 일치해야 합니다.
# # # 모델에 있는 실제 클래스 이름으로 수정해주세요. 예: 'patty', 'lettuce', 'tomato' 등
# # ingredient_dict = {1: "bun_bottom", 2: "bun_top", 3: "cheese", 4: "lettuce", 5: "onion", 6: "patty", 7: "shrimp", 8: "tomato"}
# # # ----------------------

# # # for single robot
# # ROBOT_ID = "dsr01"
# # ROBOT_MODEL = "m0609"
# # VELOCITY, ACC = 60, 60
# # BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]

# # DR_init.__dsr__id = ROBOT_ID
# # DR_init.__dsr__model = ROBOT_MODEL

# # rclpy.init()
# # dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
# # DR_init.__dsr__node = dsr_node

# # try:
# #     from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
# # except ImportError as e:
# #     print(f"Error importing DSR_ROBOT2: {e}")
# #     sys.exit()

# # ########### Gripper Setup. Do not modify this area ############

# # GRIPPER_NAME = "rg2"
# # TOOLCHANGER_IP = "192.168.1.1"
# # TOOLCHANGER_PORT = "502"
# # gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)


# # ########### Robot Controller ############


# # class RobotController(Node):
# #     def __init__(self):
# #         super().__init__("pick_and_place_integrated")

# #         # 메뉴 DB와 재료 이름 매핑
# #         self.menu_db = {
# #             "불고기버거": ["bun_bottom", "patty", "lettuce", "tomato", "bun_top"],
# #             "치즈버거": ["bun_bottom", "patty", "cheese", "bun_top"],
# #             "새우버거": ["bun_bottom", "shrimp", "lettuce", "bun_top"],
# #         }
# #         self.ingredient_map_korean_to_yolo = {
# #             "빵": "bun_bottom", "불고기": "patty", "치즈": "cheese", "상추": "lettuce",
# #             "토마토": "tomato", "새우": "shrimp", "번": "bun_bottom"
# #         }
        
# #         # ROS 토픽에서 받은 주문을 저장할 큐
# #         self.order_queue = deque(maxlen=1)

# #         self.init_robot()
# #         self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
# #         while not self.depth_client.wait_for_service(timeout_sec=3.0):
# #             self.get_logger().info("Waiting for depth position service...")
# #         self.depth_request = SrvDepthPosition.Request()

# #         # /cmd 토픽 구독자 설정
# #         self.order_subscription = self.create_subscription(
# #             Order, '/cmd', self.order_callback, QoSProfile(depth=10)
# #         )
# #         self.get_logger().info("Integrated Robot Controller is running.")

# #     def order_callback(self, msg):
# #         """ /cmd 토픽 메시지를 받으면 큐에 추가 """
# #         self.get_logger().info("New order received, adding to queue.")
# #         self.order_queue.append(msg)

# #     def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
# #         R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
# #         T = np.eye(4)
# #         T[:3, :3] = R
# #         T[:3, 3] = [x, y, z]
# #         return T

# #     def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
# #         gripper2cam = np.load(gripper2cam_path)
# #         coord = np.append(np.array(camera_coords), 1)
# #         x, y, z, rx, ry, rz = robot_pos
# #         base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
# #         base2cam = base2gripper @ gripper2cam
# #         td_coord = np.dot(base2cam, coord)
# #         return td_coord[:3]

# #     def robot_control(self):
# #         # 큐에 처리할 주문이 없으면 아무것도 하지 않고 반환
# #         if not self.order_queue:
# #             return

# #         # 큐에서 주문을 하나 꺼냄
# #         order = self.order_queue.popleft()
# #         self.get_logger().info(f"Processing order: {order.notes}")

# #         # 주문에 포함된 모든 버거를 순서대로 조립
# #         for burger in order.burgers:
# #             self.get_logger().info(f"--- Making a '{burger.menu_name}' ---")
            
# #             # --- 수정된 옵션 처리 로직 ---
# #             # 1. 기본 재료 목록으로 리스트를 시작합니다.
# #             base_ingredients = self.menu_db.get(burger.menu_name, [])
# #             final_assembly_list = list(base_ingredients)
            
# #             # 2. 옵션을 순회하며 재료를 추가하거나 제거합니다.
# #             for option in burger.options:
# #                 yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)
# #                 if not yolo_item:
# #                     self.get_logger().warn(f"Option item '{option.item}' not found in map. Skipping.")
# #                     continue

# #                 if option.type == 'add':
# #                     # 'add'의 경우, 수량만큼 리스트에 추가합니다.
# #                     for _ in range(option.amount):
# #                         # 조립 순서를 고려하여 위쪽 빵(bun_top) 바로 앞에 추가합니다.
# #                         try:
# #                             top_bun_index = final_assembly_list.index('bun_top')
# #                             final_assembly_list.insert(top_bun_index, yolo_item)
# #                         except ValueError:
# #                             # 위쪽 빵이 없으면 그냥 맨 뒤에 추가합니다.
# #                             final_assembly_list.append(yolo_item)
                
# #                 elif option.type == 'remove':
# #                     # 'remove'의 경우, 수량만큼 리스트에서 제거합니다.
# #                     for _ in range(option.amount):
# #                         try:
# #                             final_assembly_list.remove(yolo_item)
# #                         except ValueError:
# #                             self.get_logger().warn(f"Tried to remove '{yolo_item}' which was not in the assembly list.")
# #             # --- 로직 수정 끝 ---

# #             self.get_logger().info(f"Final Assembly list: {final_assembly_list}")

# #             # 각 재료를 순서대로 집어서 놓기
# #             for ingredient_name in final_assembly_list:
# #                 self.get_logger().info(f"--- Picking ingredient: {ingredient_name} ---")
                
# #                 # 원본 코드의 로직을 재사용하여 재료 하나를 처리
# #                 self.depth_request.target = ingredient_name
                
# #                 # 서비스 호출 및 결과 대기 (spin_until_future_complete는 블로킹 방식)
# #                 self.get_logger().info(f"Calling depth service for '{ingredient_name}'")
# #                 depth_future = self.depth_client.call_async(self.depth_request)
# #                 rclpy.spin_until_future_complete(self, depth_future)

# #                 if not (depth_future.result() and sum(depth_future.result().depth_position) != 0):
# #                     self.get_logger().error(f"Could not find '{ingredient_name}'. Skipping.")
# #                     continue

# #                 result = depth_future.result().depth_position.tolist()
# #                 self.get_logger().info(f"Received depth position: {result}")

# #                 gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
# #                 robot_posx = get_current_posx()[0]
# #                 td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

# #                 td_coord[2] += 50  # DEPTH_OFFSET
# #                 td_coord[2] = max(td_coord[2], 2)  # MIN_DEPTH

# #                 target_pos = list(td_coord[:3]) + robot_posx[3:]

# #                 self.get_logger().info(f"Target position: {target_pos}")
# #                 self.pick_and_place_target(target_pos)
# #                 self.init_robot()
            
# #             self.get_logger().info(f"--- Finished '{burger.menu_name}' ---")


# #     def init_robot(self):
# #         JReady = [0, 0, 90, 0, 90, 0]
# #         movej(JReady, vel=VELOCITY, acc=ACC)
# #         gripper.open_gripper()
# #         mwait()

# #     def pick_and_place_target(self, target_pos):
# #         # 이 함수는 하나의 재료를 집어서 고정된 위치(BUCKET_POS)에 놓는 동작을 가정합니다.
# #         # 간단한 시연을 위해, 집은 후 바로 BUCKET_POS로 이동합니다.
        
# #         # 집기
# #         pick_pos_above = trans(target_pos, [0, 0, 100, 0, 0, 0])
# #         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
# #         mwait()
# #         movel(target_pos, vel=VELOCITY/2, acc=ACC/2)
# #         mwait()
# #         gripper.close_gripper()
# #         time.sleep(1)
# #         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
# #         mwait()

# #         # 놓기 (고정된 위치)
# #         self.get_logger().info(f"Placing ingredient at BUCKET_POS: {BUCKET_POS}")
# #         movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
# #         mwait()
# #         gripper.open_gripper()
# #         time.sleep(1)


# # def main(args=None):
# #     node = RobotController()
# #     # 메인 루프: rclpy.ok()가 참인 동안 계속 실행
# #     while rclpy.ok():
# #         # spin_once를 호출하여 콜백 처리 (논블로킹)
# #         rclpy.spin_once(node, timeout_sec=0.1)
# #         # 로봇 제어 로직 실행
# #         node.robot_control()

# #     node.destroy_node()
# #     rclpy.shutdown()


# # if __name__ == "__main__":
# #     main()
# # import os
# # import time
# # import sys
# # from scipy.spatial.transform import Rotation
# # import numpy as np
# # import rclpy
# # from rclpy.node import Node
# # from rclpy.qos import QoSProfile
# # import DR_init
# # from collections import Counter, deque

# # from od_msg.srv import SrvDepthPosition
# # from ament_index_python.packages import get_package_share_directory
# # from burger.onrobot import RG
# # from order_interfaces.msg import Order

# # package_path = get_package_share_directory("burger")

# # # --- [수정된 부분 1] ---
# # # [주의] 아래 목록은 학습된 YOLO 모델의 클래스 이름과 정확히 일치해야 합니다.
# # # 모델에 있는 실제 클래스 이름으로 수정해주세요. 예: 'patty', 'lettuce', 'tomato' 등
# # ingredient_dict = {1: "bun_bottom", 2: "bun_top", 3: "cheese", 4: "lettuce", 5: "onion", 6: "patty", 7: "shrimp", 8: "tomato"}
# # # ----------------------

# # # for single robot
# # ROBOT_ID = "dsr01"
# # ROBOT_MODEL = "m0609"
# # VELOCITY, ACC = 60, 60
# # BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]

# # DR_init.__dsr__id = ROBOT_ID
# # DR_init.__dsr__model = ROBOT_MODEL

# # rclpy.init()
# # dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
# # DR_init.__dsr__node = dsr_node

# # try:
# #     from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
# # except ImportError as e:
# #     print(f"Error importing DSR_ROBOT2: {e}")
# #     sys.exit()

# # ########### Gripper Setup. Do not modify this area ############

# # GRIPPER_NAME = "rg2"
# # TOOLCHANGER_IP = "192.168.1.1"
# # TOOLCHANGER_PORT = "502"
# # gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)


# # ########### Robot Controller ############


# # class RobotController(Node):
# #     def __init__(self):
# #         super().__init__("pick_and_place_integrated")

# #         # 메뉴 DB와 재료 이름 매핑
# #         self.menu_db = {
# #             "불고기버거": ["bun_bottom", "patty", "lettuce", "tomato", "bun_top"],
# #             "치즈버거": ["bun_bottom", "patty", "cheese", "bun_top"],
# #             "새우버거": ["bun_bottom", "shrimp", "lettuce", "bun_top"],
# #         }
# #         self.ingredient_map_korean_to_yolo = {
# #             "빵": "bun_bottom", "불고기": "patty", "치즈": "cheese", "상추": "lettuce",
# #             "토마토": "tomato", "새우": "shrimp", "번": "bun_bottom"
# #         }
        
# #         # ROS 토픽에서 받은 주문을 저장할 큐
# #         self.order_queue = deque(maxlen=1)

# #         self.init_robot()
# #         self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
# #         while not self.depth_client.wait_for_service(timeout_sec=3.0):
# #             self.get_logger().info("Waiting for depth position service...")
# #         self.depth_request = SrvDepthPosition.Request()

# #         # /cmd 토픽 구독자 설정
# #         self.order_subscription = self.create_subscription(
# #             Order, '/cmd', self.order_callback, QoSProfile(depth=10)
# #         )
# #         self.get_logger().info("Integrated Robot Controller is running.")

# #     def order_callback(self, msg):
# #         """ /cmd 토픽 메시지를 받으면 큐에 추가 """
# #         self.get_logger().info("New order received, adding to queue.")
# #         self.order_queue.append(msg)

# #     def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
# #         R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
# #         T = np.eye(4)
# #         T[:3, :3] = R
# #         T[:3, 3] = [x, y, z]
# #         return T

# #     def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
# #         gripper2cam = np.load(gripper2cam_path)
# #         coord = np.append(np.array(camera_coords), 1)
# #         x, y, z, rx, ry, rz = robot_pos
# #         base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
# #         base2cam = base2gripper @ gripper2cam
# #         td_coord = np.dot(base2cam, coord)
# #         return td_coord[:3]

# #     def robot_control(self):
# #         # 큐에 처리할 주문이 없으면 아무것도 하지 않고 반환
# #         if not self.order_queue:
# #             return

# #         # 큐에서 주문을 하나 꺼냄
# #         order = self.order_queue.popleft()
# #         self.get_logger().info(f"Processing order: {order.notes}")

# #         # 주문에 포함된 모든 버거를 순서대로 조립
# #         for burger in order.burgers:
# #             self.get_logger().info(f"--- Making a '{burger.menu_name}' ---")
            
# #             # 조립할 재료 목록 생성
# #             base_ingredients = self.menu_db.get(burger.menu_name, [])
# #             ingredient_counts = Counter(base_ingredients)
# #             for option in burger.options:
# #                 yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)
# #                 if yolo_item:
# #                     if option.type == 'add':
# #                         ingredient_counts[yolo_item] += option.amount
# #                     elif option.type == 'remove':
# #                         ingredient_counts[yolo_item] = max(0, ingredient_counts[yolo_item] - option.amount)
            
# #             final_assembly_list = []
# #             for item in base_ingredients:
# #                  count = ingredient_counts.pop(item, 0)
# #                  if count > 0:
# #                     final_assembly_list.extend([item] * count)
# #             for item, count in ingredient_counts.items():
# #                 if count > 0:
# #                     final_assembly_list.extend([item] * count)

# #             self.get_logger().info(f"Assembly list: {final_assembly_list}")

# #             # 각 재료를 순서대로 집어서 놓기
# #             for ingredient_name in final_assembly_list:
# #                 self.get_logger().info(f"--- Picking ingredient: {ingredient_name} ---")
                
# #                 # 원본 코드의 로직을 재사용하여 재료 하나를 처리
# #                 self.depth_request.target = ingredient_name
                
# #                 # 서비스 호출 및 결과 대기 (spin_until_future_complete는 블로킹 방식)
# #                 self.get_logger().info(f"Calling depth service for '{ingredient_name}'")
# #                 depth_future = self.depth_client.call_async(self.depth_request)
# #                 rclpy.spin_until_future_complete(self, depth_future)

# #                 if not (depth_future.result() and sum(depth_future.result().depth_position) != 0):
# #                     self.get_logger().error(f"Could not find '{ingredient_name}'. Skipping.")
# #                     continue

# #                 result = depth_future.result().depth_position.tolist()
# #                 self.get_logger().info(f"Received depth position: {result}")

# #                 gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
# #                 robot_posx = get_current_posx()[0]
# #                 td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

# #                 td_coord[2] += 50  # DEPTH_OFFSET
# #                 td_coord[2] = max(td_coord[2], 2)  # MIN_DEPTH

# #                 target_pos = list(td_coord[:3]) + robot_posx[3:]

# #                 self.get_logger().info(f"Target position: {target_pos}")
# #                 self.pick_and_place_target(target_pos)
# #                 self.init_robot()
            
# #             self.get_logger().info(f"--- Finished '{burger.menu_name}' ---")


# #     def init_robot(self):
# #         JReady = [0, 0, 90, 0, 90, 0]
# #         movej(JReady, vel=VELOCITY, acc=ACC)
# #         gripper.open_gripper()
# #         mwait()

# #     def pick_and_place_target(self, target_pos):
# #         # 이 함수는 하나의 재료를 집어서 고정된 위치(BUCKET_POS)에 놓는 동작을 가정합니다.
# #         # 간단한 시연을 위해, 집은 후 바로 BUCKET_POS로 이동합니다.
        
# #         # 집기
# #         pick_pos_above = trans(target_pos, [0, 0, 100, 0, 0, 0])
# #         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
# #         mwait()
# #         movel(target_pos, vel=VELOCITY/2, acc=ACC/2)
# #         mwait()
# #         gripper.close_gripper()
# #         time.sleep(1)
# #         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
# #         mwait()

# #         # 놓기 (고정된 위치)
# #         self.get_logger().info(f"Placing ingredient at BUCKET_POS: {BUCKET_POS}")
# #         movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
# #         mwait()
# #         gripper.open_gripper()
# #         time.sleep(1)


# # def main(args=None):
# #     node = RobotController()
# #     # 메인 루프: rclpy.ok()가 참인 동안 계속 실행
# #     while rclpy.ok():
# #         # spin_once를 호출하여 콜백 처리 (논블로킹)
# #         rclpy.spin_once(node, timeout_sec=0.1)
# #         # 로봇 제어 로직 실행
# #         node.robot_control()

# #     node.destroy_node()
# #     rclpy.shutdown()


# # if __name__ == "__main__":
# #     main()
# import os
# import time
# import sys
# from scipy.spatial.transform import Rotation
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile
# import DR_init
# from collections import Counter, deque

# from od_msg.srv import SrvDepthPosition
# from ament_index_python.packages import get_package_share_directory
# from burger.onrobot import RG
# from order_interfaces.msg import Order

# package_path = get_package_share_directory("burger")

# # --- [수정된 부분 1] ---
# # [주의] 아래 목록은 학습된 YOLO 모델의 클래스 이름과 정확히 일치해야 합니다.
# # 모델에 있는 실제 클래스 이름으로 수정해주세요. 예: 'patty', 'lettuce', 'tomato' 등
# ingredient_dict = {1: "bun_bottom", 2: "bun_top", 3: "cheese", 4: "lettuce", 5: "onion", 6: "patty", 7: "shrimp", 8: "tomato"}
# # ----------------------

# # for single robot
# ROBOT_ID = "dsr01"
# ROBOT_MODEL = "m0609"
# VELOCITY, ACC = 60, 60
# BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]

# DR_init.__dsr__id = ROBOT_ID
# DR_init.__dsr__model = ROBOT_MODEL

# rclpy.init()
# dsr_node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
# DR_init.__dsr__node = dsr_node

# try:
#     from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans
# except ImportError as e:
#     print(f"Error importing DSR_ROBOT2: {e}")
#     sys.exit()

# ########### Gripper Setup. Do not modify this area ############

# GRIPPER_NAME = "rg2"
# TOOLCHANGER_IP = "192.168.1.1"
# TOOLCHANGER_PORT = "502"
# gripper = RG(GRIPPER_NAME, TOOLCHANGER_IP, TOOLCHANGER_PORT)


# ########### Robot Controller ############


# class RobotController(Node):
#     def __init__(self):
#         super().__init__("pick_and_place_integrated")

#         # 메뉴 DB와 재료 이름 매핑
#         self.menu_db = {
#             "불고기버거": ["bun_bottom", "patty", "lettuce", "tomato", "bun_top"],
#             "치즈버거": ["bun_bottom", "patty", "cheese", "bun_top"],
#             "새우버거": ["bun_bottom", "shrimp", "lettuce", "bun_top"],
#         }
#         self.ingredient_map_korean_to_yolo = {
#             "빵": "bun_bottom", "불고기": "patty", "치즈": "cheese", "상추": "lettuce",
#             "토마토": "tomato", "새우": "shrimp", "번": "bun_bottom"
#         }
        
#         # ROS 토픽에서 받은 주문을 저장할 큐
#         self.order_queue = deque(maxlen=1)

#         self.init_robot()
#         self.depth_client = self.create_client(SrvDepthPosition, "/get_3d_position")
#         while not self.depth_client.wait_for_service(timeout_sec=3.0):
#             self.get_logger().info("Waiting for depth position service...")
#         self.depth_request = SrvDepthPosition.Request()

#         # /cmd 토픽 구독자 설정
#         self.order_subscription = self.create_subscription(
#             Order, '/cmd', self.order_callback, QoSProfile(depth=10)
#         )
#         self.get_logger().info("Integrated Robot Controller is running.")

#     def order_callback(self, msg):
#         """ /cmd 토픽 메시지를 받으면 큐에 추가 """
#         self.get_logger().info("New order received, adding to queue.")
#         self.order_queue.append(msg)

#     def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
#         R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = [x, y, z]
#         return T

#     def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
#         gripper2cam = np.load(gripper2cam_path)
#         coord = np.append(np.array(camera_coords), 1)
#         x, y, z, rx, ry, rz = robot_pos
#         base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
#         base2cam = base2gripper @ gripper2cam
#         td_coord = np.dot(base2cam, coord)
#         return td_coord[:3]

#     def robot_control(self):
#         # 큐에 처리할 주문이 없으면 아무것도 하지 않고 반환
#         if not self.order_queue:
#             return

#         # 큐에서 주문을 하나 꺼냄
#         order = self.order_queue.popleft()
#         self.get_logger().info(f"Processing order: {order.notes}")

#         # 주문에 포함된 모든 버거를 순서대로 조립
#         for burger in order.burgers:
#             self.get_logger().info(f"--- Making a '{burger.menu_name}' ---")
            
#             # --- 수정된 옵션 처리 로직 ---
#             # -----------------------------
#             # 조립 리스트 생성 로직 (완전 수정본)
#             # -----------------------------

#             # 기본 재료 구성 생성 (menu_name 기반)
#             base_ingredients = list(self.menu_db.get(burger.menu_name, []))
#             final_assembly_list = base_ingredients.copy()

#             # 옵션 처리
#             ingredient_counts = Counter(final_assembly_list)

#             for option in burger.options:
#                 self.get_logger().info(f"[DEBUG] option.item={option.item}, option.type={option.type}, option.amount={option.amount}")
#                 yolo_item = self.ingredient_map_korean_to_yolo.get(option.item)

#                 if not yolo_item:
#                     continue

#                 if option.type == 'add':
#                     ingredient_counts[yolo_item] += option.amount

#                 elif option.type == 'remove':
#                     ingredient_counts[yolo_item] = max(0, ingredient_counts[yolo_item] - option.amount)

#             # 재조합
#             final_assembly_list = []
#             for item, count in ingredient_counts.items():
#                 final_assembly_list.extend([item] * count)

#             self.get_logger().info(f"Final Assembly list: {final_assembly_list}")

#             # 각 재료를 순서대로 집어서 놓기
#             for ingredient_name in final_assembly_list:
#                 self.get_logger().info(f"--- Picking ingredient: {ingredient_name} ---")
                
#                 # 원본 코드의 로직을 재사용하여 재료 하나를 처리
#                 self.depth_request.target = ingredient_name
                
#                 # 서비스 호출 및 결과 대기 (spin_until_future_complete는 블로킹 방식)
#                 self.get_logger().info(f"Calling depth service for '{ingredient_name}'")
#                 depth_future = self.depth_client.call_async(self.depth_request)
#                 rclpy.spin_until_future_complete(self, depth_future)

#                 if not (depth_future.result() and sum(depth_future.result().depth_position) != 0):
#                     self.get_logger().error(f"Could not find '{ingredient_name}'. Skipping.")
#                     continue

#                 result = depth_future.result().depth_position.tolist()
#                 self.get_logger().info(f"Received depth position: {result}")

#                 gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
#                 robot_posx = get_current_posx()[0]
#                 td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

#                 td_coord[2] += 50  # DEPTH_OFFSET
#                 td_coord[2] = max(td_coord[2], 2)  # MIN_DEPTH

#                 target_pos = list(td_coord[:3]) + robot_posx[3:]

#                 self.get_logger().info(f"Target position: {target_pos}")
#                 self.pick_and_place_target(target_pos)
#                 self.init_robot()
            
#             self.get_logger().info(f"--- Finished '{burger.menu_name}' ---")


#     def init_robot(self):
#         JReady = [0, 0, 90, 0, 90, 0]
#         movej(JReady, vel=VELOCITY, acc=ACC)
#         gripper.open_gripper()
#         mwait()

#     def pick_and_place_target(self, target_pos):
#         # 이 함수는 하나의 재료를 집어서 고정된 위치(BUCKET_POS)에 놓는 동작을 가정합니다.
#         # 간단한 시연을 위해, 집은 후 바로 BUCKET_POS로 이동합니다.
        
#         # 집기
#         pick_pos_above = trans(target_pos, [0, 0, 100, 0, 0, 0])
#         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
#         mwait()
#         movel(target_pos, vel=VELOCITY/2, acc=ACC/2)
#         mwait()
#         gripper.close_gripper()
#         time.sleep(1)
#         movel(pick_pos_above, vel=VELOCITY, acc=ACC)
#         mwait()

#         # 놓기 (고정된 위치)
#         self.get_logger().info(f"Placing ingredient at BUCKET_POS: {BUCKET_POS}")
#         movel(BUCKET_POS, vel=VELOCITY, acc=ACC)
#         mwait()
#         gripper.open_gripper()
#         time.sleep(1)


# def main(args=None):
#     node = RobotController()
#     # 메인 루프: rclpy.ok()가 참인 동안 계속 실행
#     while rclpy.ok():
#         # spin_once를 호출하여 콜백 처리 (논블로킹)
#         rclpy.spin_once(node, timeout_sec=0.1)
#         # 로봇 제어 로직 실행
#         node.robot_control()

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
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
            package_share_directory = os.path.expanduser("/home/changbeom/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/burger/burger") 
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
