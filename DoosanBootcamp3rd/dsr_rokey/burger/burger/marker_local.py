import cv2
import cv2.aruco as aruco
import numpy as np

def main():
    # 0번 카메라 열기 (노트북 기본 웹캠)
    cap = cv2.VideoCapture(6)

    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다. (VideoCapture(0) 실패)")
        return

    # Doosan 코드에서 쓰던 것과 동일한 딕셔너리 사용
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    aruco_params = aruco.DetectorParameters()

    print("✅ 웹캠 열림. 'q' 키를 누르면 종료됩니다.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 읽을 수 없습니다.")
            break

        # 그레이 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 아루코 마커 검출
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            # 마커 테두리 그리기
            aruco.drawDetectedMarkers(frame, corners, ids)

            for c, i in zip(corners, ids):
                marker_id = int(i[0])
                pts = c[0]           # shape: (4, 2)

                # 중심 좌표 계산
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                # 콘솔 로그
                print(f"Marker {marker_id} center: ({cx}, {cy})")

                # 영상 위에 중심점 & 텍스트 표시
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"ID:{marker_id} ({cx},{cy})",
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        # 결과 영상 화면에 표시
        cv2.imshow("ArUco Webcam", frame)

        # q 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("📦 종료")

if __name__ == "__main__":
    main()
