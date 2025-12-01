import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# STT (OpenAI) 및 오디오 관련
from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile
import os
from dotenv import load_dotenv

# TTS (gTTS, pydub) 관련
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
import io
import time # 지연 시간 추가용

from std_msgs.msg import String, Bool

class GetOrder(Node):
    # 대화 상태 정의
    STATE_IDLE = 'IDLE' # 대기 중 (초기 상태)
    STATE_AWAITING_ORDER = 'AWAITING_ORDER' # 주문 메뉴를 기다리는 중
    STATE_AWAITING_CONFIRMATION = 'AWAITING_CONFIRMATION' # 재주문/완료 응답을 기다리는 중

    def __init__(self):
        super().__init__('get_order')

        # .env 로드 및 OpenAI 클라이언트 초기화
        load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '.env'))
        openai_api_key = os.getenv("OPENAI_API_KEY")

        if not openai_api_key:
            self.get_logger().error("OPENAI_API_KEY not found in .env")
            exit(1)

        self.client = OpenAI(api_key=openai_api_key)

        # STT 및 오디오 파라미터
        self.duration = 10  # seconds
        self.samplerate = 16000  # Whisper 권장
        self.current_order = "주문 내역 없음"
        self.conversation_state = self.STATE_IDLE
        
        # TTS 파라미터
        self.tts_lang = 'ko' 

        qos = QoSProfile(depth=1)

        # /order 토픽 발행자 (최종 주문 텍스트)
        self.order_pub = self.create_publisher(String, '/order', qos)
        self.finish_work_pub = self.create_publisher(Bool, '/finish_work', qos)

        # /face 토픽 구독자 (대화 시작 트리거)
        self.face_sub = self.create_subscription(
            Bool, '/face', self.face_callback, qos
        )

        self.get_logger().info("GetOrder Node Ready with integrated TTS/STT.")

    def speak(self, text):
        """gTTS를 사용하여 텍스트를 음성으로 변환하고 재생합니다."""
        self.get_logger().info(f"TTS Speaking: '{text}'")
        try:
            tts = gTTS(text=text, lang=self.tts_lang)
            mp3_fp = io.BytesIO()
            tts.write_to_fp(mp3_fp)
            mp3_fp.seek(0)

            # pydub으로 오디오 로드 및 재생 (FFmpeg 필요)
            audio = AudioSegment.from_file(mp3_fp, format="mp3")
            play(audio)
            
        except Exception as e:
            self.get_logger().error(f"TTS or Playback Error: {e}")

    def record_and_transcribe(self, prompt_text=""):
        """음성을 녹음하고 Whisper API를 통해 텍스트로 변환합니다."""
        
        # 녹음 전에 프롬프트가 있다면 말하기
        if prompt_text:
            self.speak(prompt_text)
            # TTS가 끝난 후 사용자가 말할 수 있도록 잠시 대기
            time.sleep(1) 

        self.get_logger().info(f"Recording for {self.duration} seconds...")

        audio = sd.rec(
            int(self.duration * self.samplerate),
            samplerate=self.samplerate,
            channels=1,
            dtype="int16",
        )
        sd.wait()

        transcript_text = "음성 인식 실패"

        # 임시 WAV 파일 저장 및 STT 요청
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)

            try:
                with open(temp_wav.name, "rb") as f:
                    transcript = self.client.audio.transcriptions.create(
                        model="whisper-1", file=f
                    )
                transcript_text = transcript.text
            except Exception as e:
                self.get_logger().error(f"OpenAI API Error: {e}")
            
        os.remove(temp_wav.name)

        self.get_logger().info(f"STT Result: {transcript_text}")
        return transcript_text


    def face_callback(self, msg):
        """/face 토픽을 받아 대화를 시작하거나 재개합니다."""
        if msg.data and self.conversation_state == self.STATE_IDLE:
            # 첫 시작 트리거
            self.conversation_state = self.STATE_AWAITING_ORDER
            self.get_logger().info("Conversation Started.")
            self.start_order_process()

    def start_order_process(self):
        """
        주문 메뉴를 받기 위한 STT를 수행하고 다음 단계로 넘어가는 메인 대화 흐름입니다.
        이 함수는 Blocking 방식으로 순차적인 대화를 처리합니다.
        """
        while self.conversation_state != self.STATE_IDLE:
            if self.conversation_state == self.STATE_AWAITING_ORDER:
                # 1. TTS 1: 주문 메뉴 요청
                greeting_text = "안녕하세요. 햄부기입니다. 원하시는 메뉴를 주문해주세요"
                self.current_order = self.record_and_transcribe(greeting_text)
                                
                # 🚨 STT 결과를 바로 /order로 발행합니다. 
                # -> OrderDetails가 이를 파싱하고 RQT UI(/order_text)를 업데이트합니다.
                self.order_pub.publish(String(data=self.current_order))
                self.get_logger().info("Published raw order to /order for immediate parsing and display.")
                
                # 🚨 RQT가 업데이트되고 사용자가 주문 내역을 확인할 시간을 확보합니다.
                time.sleep(1.5)

                # 2. 상태 전환: 확인 단계로 이동
                self.conversation_state = self.STATE_AWAITING_CONFIRMATION
                
            elif self.conversation_state == self.STATE_AWAITING_CONFIRMATION:
                # 3. TTS 2: 확인 및 재주문/완료 요청
                
                # 사용자의 주문 내역을 TTS에 포함하여 더 자연스럽게 만듭니다.
                # 사용자의 주문 내역을 TTS에 포함하여 더 자연스럽게 만듭니다.
                # (이 시점에 RQT에는 OrderDetails가 파싱한 내용이 표시됩니다.)
                confirmation_prompt = (
                    f"주문 내역을 확인해주시기 바랍니다. 재주문을 원하시면 재주문, "
                    f"그렇지 않을 경우 완료 라고 말해주세요"
                )
                
                confirmation_response = self.record_and_transcribe(confirmation_prompt)
                                
                # 4. STT 2 결과 분석 및 상태 전환
                if "재주문" in confirmation_response:
                    self.get_logger().info("User requested re-order. Restarting menu process.")
                    # 재주문 요청 시 AWAITING_ORDER로 돌아가 다시 메뉴를 말하게 함.
                    self.conversation_state = self.STATE_AWAITING_ORDER
                
                elif "완료" in confirmation_response:
                    self.get_logger().info("User completed order. Publishing final order signal.")
                    
                    # 🚨 /finish_work 토픽을 True로 발행하여 OrderDetails에게 최종 /cmd 발행을 지시합니다.
                    finish_msg = Bool(data=True)
                    self.finish_work_pub.publish(finish_msg) 
                    
                    # 🚨 이 시점에서는 /order를 다시 발행할 필요가 없습니다.

                    # 5. 대화 종료
                    self.conversation_state = self.STATE_IDLE # 대화 종료 상태
                    self.speak("주문이 완료되었습니다. 감사합니다.")
                    
                else:
                    self.get_logger().warn(f"Unrecognized response: '{confirmation_response}'. Assuming re-order for safety.")
                    # 인식 실패나 모호한 응답의 경우 재주문으로 간주하고 다시 메뉴 요청
                    self.speak("죄송합니다. '재주문' 또는 '완료'를 명확하게 말씀해 주시겠어요?")
                    self.conversation_state = self.STATE_AWAITING_ORDER


def main(args=None):
    rclpy.init(args=args)
    node = GetOrder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()