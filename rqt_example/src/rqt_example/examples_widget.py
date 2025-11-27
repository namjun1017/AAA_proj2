import os

from ament_index_python.resources import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel

import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String


class ExamplesWidget(QWidget):

    def __init__(self, node):
        super(ExamplesWidget, self).__init__()
        self.setObjectName('ExamplesWidget')

        self.node = node

        pkg_name = 'rqt_example'
        ui_filename = 'rqt_example.ui'

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        qos = QoSProfile(depth=10)

        # --------------------------
        # Publishers
        # --------------------------
        self.pub_order_start = self.node.create_publisher(Bool, '/order_start', qos)
        self.pub_finish_work = self.node.create_publisher(Bool, '/finish_work', qos)

        # --------------------------
        # Subscriber (label)
        # --------------------------
        self.sub_order_text = self.node.create_subscription(
            String,
            '/order_text',
            self.cb_order_text,
            qos
        )

        # --------------------------
        # Button Connect
        # --------------------------
        self.push_button_startOrder.clicked.connect(self.send_start_order)
        self.push_button_finishOrder.clicked.connect(self.send_finish_order)
        self.push_button_finishWork.clicked.connect(self.send_finish_work)

        # --------------------------
        # QLabel 생성 후 ScrollArea에 넣기
        # --------------------------
        self.label = QLabel()                  # ScrollArea에 들어갈 QLabel
        self.label.setWordWrap(True)           # 줄 바꿈 활성화

        self.scrollArea.setWidget(self.label)  # 기존 scrollArea에 QLabel 배치

        # 초기 텍스트
        self.label.setText("")

    # ==========================================================
    # Callback: order_text 구독해서 label 업데이트
    # ==========================================================
    def cb_order_text(self, msg: String):
        self.label.setText(msg.data)

    # ==========================================================
    # Button functions
    # ==========================================================
    def send_start_order(self):
        msg = Bool()
        msg.data = True
        self.pub_order_start.publish(msg)

    def send_finish_order(self):
        msg = Bool()
        msg.data = False
        self.pub_order_start.publish(msg)

    def send_finish_work(self):
        msg = Bool()
        msg.data = True
        self.pub_finish_work.publish(msg)

    # ==========================================================
    # 종료 시 정리
    # ==========================================================
    def shutdown_widget(self):
        self.node.destroy_subscription(self.sub_order_text)
        self.node.destroy_publisher(self.pub_order_start)
        self.node.destroy_publisher(self.pub_finish_work)
