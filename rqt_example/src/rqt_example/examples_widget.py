# examples_widget.py 파일 전체를 아래와 같이 수정합니다.

import os

from ament_index_python.resources import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel

import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class ExamplesWidget(QWidget):

    def __init__(self, node):
        super(ExamplesWidget, self).__init__()
        self.setObjectName('ExamplesWidget')

        self.node = node

        pkg_name = 'rqt_example'
        ui_filename = 'rqt_example.ui'

        # UI 파일 로드 (기존과 동일)
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        qos = QoSProfile(depth=10)

        # --------------------------
        # Publishers 제거 (모든 버튼 기능 제거)
        # --------------------------
        # 기존: self.pub_order_start, self.pub_reorder, self.pub_finish_work 관련 코드 모두 제거

        # --------------------------
        # Subscriber (주문 내역 표시용)
        # --------------------------
        self.sub_order_text = self.node.create_subscription(
            String,
            '/order_text',
            self.cb_order_text,
            qos
        )

        # --------------------------
        # Button Connect 제거 (모든 버튼 연결 제거)
        # --------------------------
        # 기존: push_button_startOrder, push_button_reOrder, push_button_finishWork 연결 제거

        # --------------------------
        # QLabel 생성 후 ScrollArea에 넣기
        # --------------------------
        self.label = QLabel()                  # ScrollArea에 들어갈 QLabel
        self.label.setWordWrap(True)           # 줄 바꿈 활성화
        self.scrollArea.setWidget(self.label)  # 기존 scrollArea에 QLabel 배치

        # 초기 텍스트
        self.label.setText("주문을 시작하려면 카메라에 얼굴을 인식 시켜주세요")

    # ==========================================================
    # Callback: order_text 구독해서 label 업데이트 (주문 내역 표시)
    # ==========================================================
    def cb_order_text(self, msg: String):
        self.label.setText(msg.data)

    # ==========================================================
    # Button functions 제거 (모든 send_* 함수 제거)
    # ==========================================================
    # 기존: send_start_order, send_reorder, send_finish_work 함수 모두 제거

    # ==========================================================
    # 종료 시 정리
    # ==========================================================
    def shutdown_widget(self):
        self.node.destroy_subscription(self.sub_order_text)
        # 기존 Publisher 정리 코드 제거