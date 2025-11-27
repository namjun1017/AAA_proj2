import os
import json
import re
import warnings
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate

class OrderDetails:
    def __init__(self):
        load_dotenv(dotenv_path=".env")
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
        - '빼', '빼줘' 등은 remove / '더', '추가' 등은 add.
        - amount 명시 없으면 1.
        - 기본 구성에 없는 항목 add는 무시.
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
                
                # --- [수정] 핵심 유효성 검증: 기본 구성에 없는 항목 'add'는 무시 ---
                if opt["type"] == "add" and opt["item"] not in base_items:
                    # 유효하지 않은 'add' 옵션은 경고 후 건너뛰기
                    warnings.warn(f"Ignoring 'add' option: '{opt['item']}' not in base menu: '{order['menu']}'", stacklevel=2)
                    continue

                valid_options.append(opt)

            # 유효한 옵션 리스트로 갱신
            order["options"] = valid_options


        # notes 기본값
        if "notes" not in parsed:
            parsed["notes"] = ""

        return parsed


# -------------------------
# 실행 테스트
# -------------------------
if __name__ == "__main__":
    # 패티 정규화가 '불고기'로 변경되었으므로, 예시 출력도 이에 맞춰 수정했습니다.
    parser = OrderDetails()

    test_sentences = [
        "불고기 버거 하나에 토마토 빼고 패티 한장 추가요",
        "불고기버거 2개에 패티 하나 더, 토마토는 빼줘",
        "치즈버거 하나랑 새우버거 하나, 새우버거에는 고기 추가", # (수정된 검증 로직 테스트) 새우버거에는 고기(불고기)가 기본으로 없으므로, 이 'add' 옵션은 **무시되어야** 합니다.
        "불고기 버거 1개랑 치즈버거 1개, 고기 두장 더",
    ]

    for s in test_sentences:
        print("입력 :", s)
        result = parser.parse_order(s)
        print("파싱 결과:")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print("-" * 50)