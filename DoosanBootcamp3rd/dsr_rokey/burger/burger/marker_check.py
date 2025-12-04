#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import cv2.aruco as aruco
import numpy as np

from burger.realsense import ImgNode


def main(args=None):
    rclpy.init(args=args)

    # RealSense 이미지 노드 (이미 만들어둔 ImgNode 재사용)
    img_node = ImgNode()

    # ArUco 설정 (프로젝트에서 쓰는 것과 동일하게)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    aruco_params = aruco.DetectorParameters()

    print("[INFO] ArUco 체크 시작 (q 키를 누르면 종료)")

    try:
        while rclpy.ok():
            # ImgNode 콜백 한 번 돌리기
            rclpy.spin_once(img_node, timeout_sec=0.1)

            # 컬러 프레임 가져오기
            color = img_node.get_color_frame()
            if color is None:
                continue

            # GRAY 변환
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

            # 마커 검출
            corners, ids, _ = aruco.detectMarkers(
                gray,
                aruco_dict,
                parameters=aruco_params
            )

            # 결과 출력 및 화면에 그리기
            if ids is not None:
                # 박스 & ID 그리기
                aruco.drawDetectedMarkers(color, corners, ids)
                for c, i in zip(corners, ids):
                    pts = c[0]        # shape = (4, 2)
                    cx = int(np.mean(pts[:, 0]))
                    cy = int(np.mean(pts[:, 1]))
                    print(f"[ARUCO] id={int(i[0])}, center=({cx}, {cy})")

                    # 중심점 표시
                    cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)
            else:
                print("[ARUCO] No markers detected")

            # 화면 표시
            cv2.imshow("ArUco Check", color)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        img_node.destroy_node()
        rclpy.shutdown()
        print("[INFO] 종료")


if __name__ == "__main__":
    main()
