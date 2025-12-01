import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'burger'
MODEL_PATH = f'/home/rokey/nj_ws/src/AAA_proj2/DoosanBootcamp3rd/dsr_rokey/burger/burger/cvs/face_detection_yunet_2023mar.onnx'

class YuNetModel:
    def __init__(self, model_path=MODEL_PATH, conf_threshold=0.9, nms_threshold=0.3, top_k=5000):
        self.model = cv2.FaceDetectorYN.create(
            model=model_path,
            config="",
            input_size=(320, 320),
            score_threshold=conf_threshold,
            nms_threshold=nms_threshold,
            top_k=top_k
        )

    def get_detections(self, image):
        self.model.setInputSize((image.shape[1], image.shape[0]))
        return self.model.detect(image)
