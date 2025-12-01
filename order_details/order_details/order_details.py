import os
import json
import re
import warnings
from dotenv import load_dotenv
from openai import OpenAI
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from order_interfaces.msg import Order, OrderItem, Option

class OrderDetails(Node):
    def __init__(self):
        super().__init__('order_details')
        # load_dotenv(dotenv_path=".env")

        # 주소는 수정 필요
        dotenv_path = os.path.expanduser("/home/nj/test_ws/src/order_details/order_details/.env") 
        load_dotenv(dotenv_path=dotenv_path)

        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise RuntimeError("OPENAI_API_KEY not set in .env")

        # LLM 설정
        self.llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.2,
            openai_api_key=openai_api_key
        )

        # 메뉴 DB
        self.menu_db = {
            "불고기버거": ["빵", "불고기", "상추", "토마토", "빵"],
            "치즈버거": ["빵", "불고기", "치즈", "빵"],
            "새우버거": ["빵", "새우", "상추", "빵"],
        }

        # 옵션 정규화(동의어 → 공식명)
        self.item_alias = {
            "패티": "불고기",
            "고기": "불고기",
            "불고기패티": "불고기",
            "미트": "불고기",
            "불고기": "불고기",

            "양상추": "상추",
            "토마토소스": "토마토",
            "치즈슬라이스": "치즈",
        }

        # 메뉴 목록을 문자열로 변환하여 프롬프트에 삽입
        menu_lines = "\n".join(
            f"- {name} : {', '.join(items)}" for name, items in self.menu_db.items()
        )

        qos = QoSProfile(depth=1)

        self.cmd_pub = self.create_publisher(
            Order,
            '/cmd', # 새로운 토픽 이름
            qos
        )

        self.text_pub = self.create_publisher(
            String,
            '/order_text', # 새로운 토픽 이름
            qos
        )

        self.order_sub = self.create_subscription(
            String,
            '/order',
            self.order_callback,
            qos
        )

        # ----- Prompt -----
# ----- Prompt -----
        prompt_content = f"""
        당신은 한국어 주문 문장을 구조화된 JSON 주문 데이터로 변환하는 역할을 합니다.
        반드시 아래에 제시된 JSON 스키마를 **엄격히** 따르세요. (추가 텍스트 금지)

        메뉴(기본구성):
        {menu_lines}

        <JSON 출력 형식>
        {{{{  
        "orders": [
            {{{{
            "menu": "메뉴명",
            "quantity": 정수,
            "options": [
                {{{{ "item": "옵션명", "type": "add"|"remove", "amount": 정수 }}}}
            ]
            }}}}
        ],
        "notes": ""
        }}}}  

        <규칙>
        - 메뉴 이름은 메뉴 DB에 가능한 한 맞춰 정정합니다.
        - 수량 없으면 1로 간주.
        - '빼', '빼줘' 등은 remove.
        - '더', '추가', '추가했어', '추가해줘', '더 넣어줘', '더 줘' 등은 모두 add.
        - amount 명시 없으면 1.
        - 출력은 반드시 JSON만.

        <예시 입력/출력>
        입력: "불고기 버거 하나에 토마토 빼고 패티 한장 추가요"
        출력:
        {{{{
        "orders": [
            {{{{
            "menu": "불고기버거",
            "quantity": 1,
            "options": [
                {{{{ "item": "토마토", "type": "remove", "amount": 1 }}}},
                {{{{ "item": "불고기", "type": "add", "amount": 1 }}}}
            ]
            }}}}
        ],
        "notes": ""
        }}}} 

        <사용자 입력>
        "{{user_input}}"
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"],
            template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm

# -------------------------
    # JSON Extractor 수정
    # -------------------------
    def _clean_json_text(self, text: str) -> str:
        # 이중 중괄호 {{ 또는 단일 중괄호 {로 시작하고 끝나는 가장 바깥쪽 객체를 찾습니다.
        # LLM이 {{...}} 또는 {...} 형태로 출력할 수 있으므로, 가장 바깥쪽 객체만 추출합니다.
        
        # 1. 일단 정규식으로 {} 또는 {{}} 내부의 텍스트를 찾습니다.
        m = re.search(r"\{.*\}", text, flags=re.DOTALL)
        if not m:
            return text

        json_text = m.group(0).strip()
        
        # 2. 추출된 문자열이 이중 중괄호로 시작/끝나면 이를 제거합니다.
        # (프롬프트 이스케이프 때문에 LLM이 {{...}} 형태로 출력하는 경우)
        if json_text.startswith('{{') and json_text.endswith('}}'):
            return json_text[1:-1].strip() # 바깥쪽 {{ 와 }} 중 하나씩 제거
        
        return json_text

    # -------------------------
    # 메뉴 정규화
    # -------------------------
    def _match_menu(self, text: str) -> str:
        if not text:
            return ""
        t = text.replace(" ", "").lower()

        for name in self.menu_db.keys():
            if name.replace(" ", "").lower() in t or t in name.replace(" ", "").lower():
                return name

        for name in self.menu_db.keys():
            if name.replace(" ", "").lower() == t:
                return name

        return text

    def _format_order_to_text(self, parsed_dict: dict) -> str:
            output_lines = ["==============================", "주문 내역", "=============================="]
            
            orders = parsed_dict.get("orders", [])
            
            # Helper for formatting option item amount (장 vs 개)
            def format_option_amount(item, amount):
                # '불고기'(패티), '치즈'일 경우 '장' 사용, 그 외는 '개' 사용
                unit = "장" if item in ["불고기", "치즈"] else "개"
                return f"{item} {amount}{unit}"

            # 주문 항목별 반복
            for i, order_item in enumerate(orders):
                menu = order_item.get("menu", "알 수 없는 메뉴")
                quantity = order_item.get("quantity", 0)
                options = order_item.get("options", [])
                
                output_lines.append(f"\n{i+1}. 메뉴명       : {menu}")
                output_lines.append(f"   수량         : {quantity}")
                
                # 옵션 분류 및 포맷팅
                add_options_str = ""
                remove_options_str = ""
                
                add_options = [opt for opt in options if opt.get("type") == "add"]
                remove_options = [opt for opt in options if opt.get("type") == "remove"]
                
                if add_options:
                    add_options_str = ", ".join([format_option_amount(opt['item'], opt['amount']) for opt in add_options])
                else:
                    add_options_str = "없음"
                    
                if remove_options:
                    remove_options_str = ", ".join([format_option_amount(opt['item'], opt['amount']) for opt in remove_options])
                else:
                    remove_options_str = "없음"
                    
                output_lines.append("   옵션:")
                output_lines.append(f"     - 추가: {add_options_str}")
                output_lines.append(f"     - 제거: {remove_options_str}")
                
                # 메모 (JSON은 최상위에 'notes'가 있지만, 예시처럼 항목별로 없음을 표시)
                notes_line = "없음"
                if i == len(orders) - 1 and parsed_dict.get("notes"):
                    # 예시처럼 마지막 항목에만 전체 메모를 표시
                    notes_line = parsed_dict["notes"]
                    
                output_lines.append(f"   메모         : {notes_line}")

            
            # 총 주문 항목 수
            output_lines.append("\n==============================")
            output_lines.append(f"총 주문 항목 수: {len(orders)}")
            output_lines.append("==============================")
            
            return "\n".join(output_lines)

    # -------------------------
    # 메인 파싱 함수 (수정된 부분 포함)
    # -------------------------
    def parse_order(self, user_text: str):
        # LLM 호출
        response = self.lang_chain.invoke({"user_input": user_text})
        raw = response.content.strip()

        # JSON 추출
        json_text = self._clean_json_text(raw)

        # JSON 파싱
        try:
            parsed = json.loads(json_text)
        except json.JSONDecodeError:
            # 복구 시도
            s = json_text.replace("'", '"')
            s = re.sub(r",\s*}", "}", s)
            s = re.sub(r",\s*]", "]", s)
            try:
                parsed = json.loads(s)
            except Exception as e:
                warnings.warn(f"Failed to parse JSON. raw:\n{raw}\nerror:{e}", stacklevel=2)
                return None

        # -----------------------------
        # 정규화/기본값 처리 및 유효성 검증
        # -----------------------------
        for order in parsed.get("orders", []):
            # 메뉴 정규화
            order["menu"] = self._match_menu(order.get("menu", ""))

            # 수량 기본값
            if "quantity" not in order or not isinstance(order["quantity"], int):
                order["quantity"] = 1

            # 옵션 기본 구조
            if "options" not in order or not isinstance(order["options"], list):
                order["options"] = []
            
            # 해당 메뉴의 기본 구성품 목록 가져오기 (없으면 빈 리스트)
            base_items = self.menu_db.get(order["menu"], [])
            
            valid_options = []
            
            # 옵션 세부 정규화 및 유효성 검증
            for opt in order["options"]:
                # amount 기본값
                if "amount" not in opt or not isinstance(opt["amount"], int):
                    opt["amount"] = 1

                # type normalize
                if opt.get("type") not in ("add", "remove"):
                    opt["type"] = "add"

                # 옵션명 정규화 (패티=불고기 등)
                item = opt.get("item", "")
                if item in self.item_alias:
                    opt["item"] = self.item_alias[item]
                
                
                valid_options.append(opt)

            # 유효한 옵션 리스트로 갱신
            order["options"] = valid_options


        # notes 기본값
        if "notes" not in parsed:
            parsed["notes"] = ""

        return parsed

    def order_callback(self, msg):
        raw_text = msg.data
        self.get_logger().info(f"[DEBUG] STEP 1 - Raw voice input: {raw_text}")

        parsed_dict = self.parse_order(raw_text)

        if parsed_dict is None:
            self.get_logger().error("[ERROR] Failed to parse order → parsed_dict is None")
            return

        if not parsed_dict.get("orders"):
            self.get_logger().error(f"[ERROR] No orders detected in parsed_dict={parsed_dict}")
            return

        self.get_logger().info("Parsed raw order and starting mapping.")


        if parsed_dict:
            self.get_logger().info(f"Parsed raw order and starting mapping.")

            # 🚨 1. 최상위 Order 메시지 객체 생성
            order_msg = Order()
            order_msg.notes = parsed_dict.get("notes", "")
            
            # Level 2 (개별 인스턴스)를 담을 리스트
            burger_instances = [] 
            
            # 🚨 2. 수량(Quantity)만큼 반복하여 인스턴스 풀기 (Unrolling)
            for item_dict in parsed_dict.get("orders", []):
                menu_name = item_dict.get("menu", "")
                quantity = item_dict.get("quantity", 0)
                
                # 수량 (quantity) 만큼 반복하여 개별 인스턴스 생성
                for i in range(quantity):
                    # 🚨 Level 2: 개별 버거 인스턴스 생성 (불고기버거_1)
                    burger_instance_msg = OrderItem() 
                    burger_instance_msg.menu_name = menu_name
                    
                    options_list = []
                    
                    # 🚨 Level 3: 해당 인스턴스에 적용할 옵션 리스트 생성
                    for opt_dict in item_dict.get("options", []):
                        option_msg = Option()
                        option_msg.item = opt_dict.get("item", "")
                        option_msg.type = opt_dict.get("type", "")
                        
                        # 인스턴스별 옵션에서는 amount를 1로 처리하는 것이 일반적입니다.
                        # (하나의 버거에 치즈 2개를 넣고 싶다면, LLM이 옵션을 두 개로 분리하는 것이 더 명확합니다.)
                        option_msg.amount = opt_dict.get("amount", 1) 
                        options_list.append(option_msg)
                    
                    burger_instance_msg.options = options_list
                    burger_instances.append(burger_instance_msg)
                    
            # 3. 최상위 Order 메시지에 인스턴스 리스트 설정
            order_msg.burgers = burger_instances 
            # 3. 최상위 Order 메시지에 인스턴스 리스트 설정
            # order_msg.burgers = burger_instances 

            # ===========================
            # 🔥 DEBUG: 파싱된 주문 구조 확인
            # ===========================
            self.get_logger().info("===== DEBUG ORDER_DETAILS OUTPUT =====")
            for idx, burger in enumerate(order_msg.burgers):
                self.get_logger().info(f"[Burger {idx}] menu_name = {burger.menu_name}")
                
                if not burger.options:
                    self.get_logger().warn(f"[Burger {idx}] ⚠ options is EMPTY")
                else:
                    for op in burger.options:
                        self.get_logger().info(
                            f"   OPTION → item={op.item}, type={op.type}, amount={op.amount}"
                        )
            self.get_logger().info(f"notes = {order_msg.notes}")
            self.get_logger().info("======================================")

            # 4. 발행
            # self.cmd_pub.publish(order_msg)
            # self.get_logger().info(f"Published {len(burger_instances)} individual burger instances to /parsed_order topic.")

            # 4. 발행
            self.cmd_pub.publish(order_msg)
            self.get_logger().info(f"Published {len(burger_instances)} individual burger instances to /parsed_order topic.")
            
            formatted_text = self._format_order_to_text(parsed_dict)
            text_msg = String()
            text_msg.data = formatted_text

            self.text_pub.publish(text_msg)
            self.get_logger().info("Published formatted order text to /order_text topic.")
            
        else:
            self.get_logger().warn("Failed to parse order or result was None.")


# -------------------------
# 실행 테스트
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OrderDetails()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

