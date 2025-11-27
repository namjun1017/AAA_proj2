import rclpy
from rclpy.node import Node

from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile
import os
from dotenv import load_dotenv

from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String, Float64MultiArray, Bool  # [수정] String 임포트 추가

class GetOrder(Node):
    def __init__(self):
        super().__init__('get_order')

        # .env 로드
        load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '.env'))
        openai_api_key = os.getenv("OPENAI_API_KEY")

        if not openai_api_key:
            self.get_logger().error("OPENAI_API_KEY not found in .env")
            exit(1)

        # OpenAI STT Client
        self.client = OpenAI(api_key=openai_api_key)

        # 파라미터
        self.duration = 10  # seconds
        self.samplerate = 16000  # Whisper는 16kHz를 선호
        self.order = "주문 내역 없음"

        qos = QoSProfile(depth=1)

        self.order_pub = self.create_publisher(String, '/order', qos)

        self.word_sub = self.create_subscription(
            Bool, '/order_start', self.getword_callback, qos
        )

        self.get_logger().info("GetOrder Node Ready")

    def getword_callback(self, msg):
        if msg.data:
            self.get_logger().info("Recording for 5 seconds...")

            audio = sd.rec(
                int(self.duration * self.samplerate),
                samplerate=self.samplerate,
                channels=1,
                dtype="int16",
            )
            sd.wait()

            # 임시 WAV 파일 저장
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
                wav.write(temp_wav.name, self.samplerate, audio)

                with open(temp_wav.name, "rb") as f:
                    transcript = self.client.audio.transcriptions.create(
                        model="whisper-1", file=f
                    )

            os.remove(temp_wav.name)

            self.get_logger().info(f"GetOrder Result: {transcript.text}")
            self.order = transcript.text

            self.order_pub.publish(String(data=self.order))


def main(args=None):
    rclpy.init(args=args)
    node = GetOrder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()