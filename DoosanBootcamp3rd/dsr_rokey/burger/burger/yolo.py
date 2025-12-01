########## YoloModel ##########
import os
import json
import time
from collections import Counter

import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np


# --- [수정된 부분 1] ---
# 사용자의 커스텀 모델 절대 경로를 직접 지정합니다.
# 경로에 사용자 이름(changbeom)이 포함되어 있으니, 다른 환경에서는 수정이 필요할 수 있습니다.
YOLO_MODEL_PATH = "/home/hyochan/ros2_ws/src/AAA_proj2/DoosanBootcamp3rd/dsr_rokey/burger/burger/cvs/best_v04.pt"
# ----------------------


class YoloModel:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)
        
        # --- [수정된 부분 2] ---
        # JSON 파일에서 클래스 이름을 읽는 대신, 모델 파일에서 직접 클래스 이름을 가져옵니다.
        # self.model.names는 {id: 'name'} 형태의 딕셔너리입니다. (예: {0: 'patty', 1: 'lettuce'})
        # 이를 {'name': id} 형태로 변환하여 기존 코드와 호환성을 유지합니다.
        self.reversed_class_dict = {v: k for k, v in self.model.names.items()}
        print("Model loaded. Available classes:", self.reversed_class_dict)
        # ----------------------

    def get_frames(self, img_node, duration=1.0):
        """get frames while target_time"""
        end_time = time.time() + duration
        frames = {}

        while time.time() < end_time:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            stamp = img_node.get_color_frame_stamp()
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)

        if not frames:
            print("No frames captured in %.2f seconds" % duration)

        print("%d frames captured" % len(frames))
        return list(frames.values())

    def get_best_detection(self, img_node, target):
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:  # Check if frames are empty
            return None, None

        results = self.model(frames, verbose=False)
        detections = self._aggregate_detections(results)

        # --- [수정된 부분 3] ---
        # 요청된 target이 self.reversed_class_dict에 있는지 확인합니다.
        if target not in self.reversed_class_dict:
            print(f"Error: Target '{target}' not found in the model's classes.")
            return None, None
        # ----------------------

        label_id = self.reversed_class_dict[target]
        print("Looking for label_id: ", label_id)

        print("Detections: ", detections)

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("No matches found for the target label.")
            return None, None
        best_det = max(matches, key=lambda x: x["score"])
        return best_det["box"], best_det["score"]

    def _aggregate_detections(self, results, confidence_threshold=0.5, iou_threshold=0.5):
        """
        Fuse raw detection boxes across frames using IoU-based grouping
        and majority voting for robust final detections.
        """
        raw = []
        for res in results:
            for box, score, label in zip(
                res.boxes.xyxy.tolist(),
                res.boxes.conf.tolist(),
                res.boxes.cls.tolist(),
            ):
                if score >= confidence_threshold:
                    raw.append({"box": box, "score": score, "label": int(label)})

        final = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            boxes = np.array([g["box"] for g in group])
            scores = np.array([g["score"] for g in group])
            labels = [g["label"] for g in group]

            final.append(
                {
                    "box": boxes.mean(axis=0).tolist(),
                    "score": float(scores.mean()),
                    "label": Counter(labels).most_common(1)[0][0],
                }
            )

        return final

    def _iou(self, box1, box2):
        """
        Compute Intersection over Union (IoU) between two boxes [x1, y1, x2, y2].
        """
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0