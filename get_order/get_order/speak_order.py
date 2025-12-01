import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Bool, String # String 타입 추가 (TTS할 텍스트를 받을 수도 있으므로)

from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
import io
import os
import tempfile

class SpeakOrder(Node):
    def __init__(self):
        super().__init__('speak_order')

        # TTS에 사용할 기본 텍스트
        self.default_text = "주문 내역을 다시 한 번 말씀해 주십시오."
        self.lang = 'ko' # 한국어 설정

        qos = QoSProfile(depth=1)

        # /face 토픽 구독: Bool 타입 신호를 받으면 TTS 동작
        self.face_sub = self.create_subscription(
            Bool, 
            '/face', 
            self.face_callback, 
            qos
        )
        
        # /order_text 토픽 구독 (선택적): TTS 할 텍스트를 받아올 수 있음
        self.text_sub = self.create_subscription(
            String,
            '/order_text',
            self.text_callback,
            qos
        )
        
        self.tts_text = self.default_text # 현재 TTS 할 텍스트를 저장할 변수
        
        self.get_logger().info("SpeakOrder Node Ready (gTTS Ready)")

    def text_callback(self, msg):
        """
        /order_text 토픽에서 새로운 텍스트를 받아옵니다.
        """
        self.tts_text = msg.data
        self.get_logger().info(f"Updated TTS text: {self.tts_text}")

    def face_callback(self, msg):
        """
        /face 토픽에서 True 신호를 받으면 TTS를 실행합니다.
        """
        if msg.data:
            self.get_logger().info(f"Face detected (True). Speaking: '{self.tts_text}'")
            
            try:
                # 1. gTTS 객체 생성 및 MP3 데이터 메모리에 저장
                tts = gTTS(text=self.tts_text, lang=self.lang)
                mp3_fp = io.BytesIO()
                tts.write_to_fp(mp3_fp)
                mp3_fp.seek(0)

                # 2. pydub으로 오디오 데이터 로드
                # pydub는 ffplay를 사용하여 오디오를 재생합니다 (ffmpeg가 시스템에 설치되어 있어야 함)
                audio = AudioSegment.from_file(mp3_fp, format="mp3")
                
                # 3. 오디오 재생
                play(audio)
                
                self.get_logger().info("TTS playback finished.")
                
            except Exception as e:
                self.get_logger().error(f"TTS or Playback Error: {e}")
                # TTS 실패 시 기본 텍스트로 복원
                self.tts_text = self.default_text
                
        # 신호가 False인 경우 (얼굴 인식 실패 등)에는 아무것도 하지 않습니다.


def main(args=None):
    rclpy.init(args=args)
    node = SpeakOrder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()