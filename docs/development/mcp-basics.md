# MCP 기초 — 직접 실행하며 이해하기

이 문서는 SkyBuddy orchestrator가 왜, 어떻게 MCP(Model Context Protocol)를 쓰는지
처음 접하는 팀원을 위한 문서입니다. `sandbox/mcp-101/`에 있는 예제를 순서대로
직접 실행하면서 읽는 걸 추천합니다 — 이 문서도 그렇게 한 단계씩 직접 만들어보면서
정리한 내용입니다.

## MCP가 뭔가

MCP는 **"AI 애플리케이션이 외부 도구/데이터에 접근하는 방식을 표준화한 프로토콜"**입니다.
먼저 풀어야 할 오해가 하나 있는데, **MCP 자체는 AI가 아닙니다.** LLM이 등장하지 않아도
MCP는 완벽하게 동작합니다 — "어떤 도구가 있는지 알려주고(list_tools), 그 도구를
실행해달라는 요청을 표준 형식으로 받는(call_tool)" 통신 규약일 뿐입니다.

USB에 비유하면 이해가 쉽습니다. USB가 있기 전엔 마우스마다, 프린터마다 전용 포트와
드라이버가 필요했습니다. USB는 "이런 규격으로 꽂으면 어떤 기기든 인식된다"는 표준을
만들었죠. MCP도 마찬가지로, **어떤 LLM(Gemini, Claude, GPT...)을 쓰든 같은 방식으로
도구에 연결**할 수 있게 해주는 규격입니다. 그래서 도구(MCP 서버)를 만드는 팀과
AI 로직을 만드는 팀이 서로의 내부 구현에 신경 쓰지 않고 독립적으로 개발할 수 있습니다.

세 가지 역할로 나뉩니다.

| 역할 | 하는 일 | SkyBuddy 예시 |
|---|---|---|
| **MCP Server** | 도구를 갖고 있고, 표준 방식으로 노출만 함. 누가 호출하는지 모름 | `mock_drone_server.py` (나중엔 middleware가 이 역할) |
| **MCP Client** | 서버에 접속해서 "뭐 있어?"/"이거 실행해줘" 통신 담당 | orchestrator 안의 MCP 연결 코드 |
| **Host** | Client를 들고 있으면서 LLM도 호출하는 전체 진행자 | orchestrator 서비스 자체 |

LLM(Gemini 등)은 이 세 역할 어디에도 속하지 않습니다. Host가 "이런 도구들 있는데,
사용자가 이렇게 말했어. 뭘 호출해야 할까?"라고 물어봤을 때 **판단만** 해주는 두뇌
역할입니다. 실제 실행은 여전히 Host가 MCP Client로 서버에 요청을 보내서 이루어집니다.

## 실습 준비

```bash
cd sandbox/mcp-101
python3 -m venv venv
venv/bin/pip install -r requirements.txt
```

이후 이 폴더의 모든 예제는 시스템 기본 `python`이 아니라 방금 만든
`venv/bin/python`으로 실행해야 합니다. 기본 파이썬으로 실행하면
`ModuleNotFoundError: No module named 'fastmcp'`가 납니다 — 예제 스크립트가
서버를 서브프로세스로 띄울 때도 지금 실행 중인 파이썬(`sys.executable`)을 그대로
쓰기 때문에, 처음부터 올바른 venv로 실행하는 게 중요합니다.

## 1단계 — MCP만 실행해보기 (LLM 없이)

`demo_server.py`는 도구가 `takeoff` 하나뿐인 제일 단순한 MCP 서버입니다.
```python
@mcp.tool()
def takeoff(altitude: float = 3.0) -> str:
    """드론을 지정한 고도까지 이륙시킵니다"""
    return f"드론이 고도 {altitude}m까지 이륙했습니다."
```
`@mcp.tool()` 데코레이터 하나로 일반 함수가 MCP 도구가 됩니다. FastMCP가 타입힌트와
docstring을 읽어서 JSON Schema를 자동 생성해줍니다.

`demo_client.py`는 이 서버에 접속해서, **사람이 직접 정한 값**으로 도구를 실행합니다.
```python
await session.initialize()                       # 핸드셰이크
tools = await session.list_tools()                # "뭐 있어?"
result = await session.call_tool("takeoff", {"altitude": 3})  # "이거 실행해줘" (하드코딩)
```

실행:
```bash
venv/bin/python demo_client.py
```

출력에 `list_tools`로 조회된 `takeoff` 스키마와, `call_tool` 실행 결과(
`드론이 고도 3.0m까지 이륙했습니다.`)가 순서대로 찍히면 성공입니다. 여기엔 AI가
전혀 없습니다 — `"takeoff"`, `{"altitude": 3}`을 코드에 직접 박아넣었으니까요. 이
단계의 목적은 "AI를 빼고 MCP 통신 자체가 뭘 하는지"부터 확실히 보는 것입니다.

## 2단계 — LLM 판단 추가해서 실행해보기

`llm_practice.py`에서부터 Gemini가 "판단"을 대신합니다. 실행 전에 API 키가
필요합니다 (`=` 앞뒤 공백 없이 붙여 써야 합니다):
```bash
export GOOGLE_API_KEY="본인의_Gemini_API_키"
venv/bin/python llm_practice.py
```

핵심은 **판단(decide)과 실행(execute)이 분리되어 있다는 것**입니다. 코드가 네
단계로 이어집니다.

**① MCP 도구 스키마 → Gemini 도구 형식 변환**
```python
def mcp_tool_to_gemini_tool(tool):
    return types.Tool(function_declarations=[types.FunctionDeclaration(
        name=tool.name,
        description=tool.description,
        parameters=types.Schema.from_json_schema(json_schema=types.JSONSchema(**tool.inputSchema)),
    )])
```
MCP 서버가 준 JSON Schema를, Gemini가 요구하는 `FunctionDeclaration` 형식으로
번역하는 것뿐입니다. 실행 결과는 없고 데이터 모양만 바뀝니다.

**② Gemini에게 판단만 시키기 (아직 아무것도 실행 안 됨)**
```python
response = await client.aio.models.generate_content(
    model="models/gemini-3.6-flash",
    contents=[user_content],
    config=types.GenerateContentConfig(tools=gemini_tools),
)
# response.function_calls -> [FunctionCall(name='takeoff', args={'altitude': 5}, ...)]
```
"드론 5미터로 띄워줘"라는 문장만 보고 Gemini가 스스로 도구 이름과 인자를 뽑아냅니다.
**이 시점에 mock 드론은 아직 움직이지 않습니다.**

**③ 판단대로 진짜 실행 (여기서 처음 실제 상태가 바뀜)**
```python
call = response.function_calls[0]
result = await session.call_tool(call.name, call.args)   # 진짜 MCP 실행
```
`call.name`, `call.args`는 방금 Gemini가 만들어낸 값입니다. 1단계에서 사람이
하드코딩했던 `session.call_tool("takeoff", {"altitude": 3})`와 형태는 똑같고,
값을 누가 정했는지만 다릅니다.

**④ 실행 결과를 다시 Gemini에게 보여줘서 자연어 답변 완성**
```python
follow_up = await client.aio.models.generate_content(
    model="models/gemini-3.6-flash",
    contents=[user_content, model_content, types.Content(role="user", parts=[function_response_part])],
    config=types.GenerateContentConfig(tools=gemini_tools),
)
# follow_up.text -> "드론을 고도 5m까지 이륙시켰습니다."
```
`판단 결과`, `실행 결과`, `최종 답변` 세 줄이 순서대로 출력되면 전체 루프가 완성된
것입니다. Gemini API를 두 번(판단용, 최종 답변용) 순서대로 호출하기 때문에 1단계보다
응답이 조금 더 걸립니다.

## 왜 판단/실행을 분리하나 — 환각 방지

"LLM이 자연어로 뭘 하겠다고 답하는 것"과 "실제로 그 일이 일어나는 것"은 다릅니다.
판단과 실행을 분리해두면:
- LLM이 존재하지 않는 도구를 지어내려 해도, 실제 MCP 서버에 없는 이름이면 실행 단계에서
  걸러집니다.
- "이륙했습니다"라는 답변이 나왔다면, 그건 실제로 ③번(`call_tool`)이 실행됐기 때문이지
  LLM이 그냥 지어낸 문장이 아닙니다.
- MCP 서버가 제공하는 도구 스키마(파라미터 타입, 필수 여부)가 LLM의 판단 범위를
  제한하는 **근거(context)** 역할을 합니다.

## SkyBuddy 실제 아키텍처와의 대응

```
사용자 자연어
   → orchestrator (Host + MCP Client + LLM 판단 호출)
   → [지금] mock_drone_server.py — 가짜 상태(drone_state 딕셔너리)로 응답
   → [나중] middleware — 진짜 MCP 서버로 진화, 내부에서 airsim.MultirotorClient() 호출
   → AirSim (실제 시뮬레이터)
```
`mock_drone_server.py`는 지금 middleware가 완성되기 전까지 그 자리를 임시로 대신하고
있는 것입니다. 도구의 이름/스키마는 그대로 두고 내부 구현만 가짜 → 진짜로 바뀌는 구조라,
orchestrator의 판단 로직은 나중에 거의 손댈 필요가 없습니다.

## `mock_drone_server.py`가 지금 제공하는 도구

`sandbox/mock-orchestrator/mock_drone_server.py`는 `drone_state` 딕셔너리 하나로
드론 한 대의 상태를 흉내 내며, 다음 6개 도구를 노출합니다.

| 도구 | 파라미터 | 하는 일 |
|---|---|---|
| `takeoff` | `altitude: float = 3.0` | 지정 고도까지 이륙 |
| `land` | (없음) | 현재 위치에서 착륙 |
| `move_to` | `latitude, longitude, altitude_m` | 지정 좌표로 이동 |
| `rotate` | `degrees: float` | 기수 방향 회전 |
| `return_home` | (없음) | 이륙 지점으로 복귀 |
| `get_status` | (없음) | 현재 상태 조회 |

`move_to`의 파라미터는 `services/middleware/app/schemas/mission.py`의 `GeoPosition`과
같은 이름(`latitude`, `longitude`, `altitude_m`)을 씁니다. 이륙 지점 기준 상대 좌표가
아니라 WGS84 기준 절대 좌표라는 뜻입니다.

`move_to`는 아직 **저수준 도구**입니다 — LLM이 좌표를 직접 골라서 호출합니다. 이후
`MissionAssignment`(drone_id + area_id)를 받아 내부에서 좌표 변환과 `move_to` 호출을
대신 처리하는 고수준 도구(`assign_mission`)가 추가되면, `move_to`는 LLM이 직접 부르는
도구 목록에서 빠지고 그 내부 구현으로 숨겨질 예정입니다.
