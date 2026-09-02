"""3단계: demo_client.py의 하드코딩된 call_tool(...)을,
Gemini가 자연어를 보고 스스로 판단하게 바꿔보는 연습 파일.

완료:
- 3-1: mcp_tool_to_gemini_tool() - MCP 도구 스키마 -> Gemini 도구 형식 변환
- 3-2: Gemini에게 자연어를 보여주고 "판단"만 받아보기 (function_calls)
- 3-3: 그 판단을 실제로 session.call_tool()로 실행
- 3-4: 실행 결과를 Gemini에게 다시 보여줘서 최종 자연어 답변 만들기
"""

import asyncio
import sys
from pathlib import Path
import os
from google import genai
from google.genai import types
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

server_params = StdioServerParameters(
    command=sys.executable,
    args=[str(Path(__file__).parent / "demo_server.py")],
)

client = genai.Client(api_key=os.environ["GOOGLE_API_KEY"])

def mcp_tool_to_gemini_tool(tool):
    return types.Tool(
        function_declarations=[
            types.FunctionDeclaration(
                name=tool.name,
                description=tool.description,
                parameters=types.Schema.from_json_schema(
                    json_schema=types.JSONSchema(**tool.inputSchema)
                ),
            )
        ]
    )


async def main():
    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            # 3-1: MCP 서버가 가진 도구들을 Gemini가 이해하는 형식으로 변환
            tools = await session.list_tools()
            gemini_tools = [mcp_tool_to_gemini_tool(t) for t in tools.tools]

            # 나중에(3-4) 대화 기록을 다시 만들어야 하니, 사용자 메시지를
            # 그냥 문자열이 아니라 Content 객체로 미리 만들어둔다.
            user_content = types.Content(
                role="user",
                parts=[types.Part.from_text(text="드론 5미터로 띄워줘")],
            )

            # 3-2: Gemini에게 "이 문장 + 이런 도구들 있어"를 보여주고 판단만 받기.
            # 이 시점엔 아직 아무 도구도 실행되지 않았다 - response.function_calls는
            # "이걸 부르고 싶다"는 Gemini의 의사표시일 뿐이다.
            response = await client.aio.models.generate_content(
                model='models/gemini-3.6-flash',
                contents=[user_content],
                config=types.GenerateContentConfig(tools=gemini_tools),
            )
            print("판단 결과:", response.function_calls)

            # Gemini의 이 턴(판단이 담긴 응답) 자체를 저장해둔다.
            # 3-4에서 대화 맥락을 이어가려면 이게 필요하다.
            model_content = response.candidates[0].content

            # 3-3: 판단(call.name, call.args)을 가지고 진짜로 MCP 서버에 실행 요청.
            # 여기서 처음으로 mock 드론(demo_server.py 프로세스 안)이 실제로 움직인다.
            call = response.function_calls[0]
            result = await session.call_tool(call.name, call.args)
            result_text = result.content[0].text
            print("실행 결과:", result_text)

            # 3-4: 방금 나온 실행 결과를 "도구 실행 결과 보고" 형식으로 포장한다.
            function_response_part = types.Part.from_function_response(
                name=call.name, response={"result": result_text}
            )

            # Gemini에게 대화 전체 맥락(① 내가 한 말 ② Gemini의 판단 ③ 실행 결과)을
            # 다시 보여주고, 그걸 바탕으로 사람에게 들려줄 자연어 답변을 만들게 한다.
            follow_up = await client.aio.models.generate_content(
                model='models/gemini-3.6-flash',
                contents=[
                    user_content,
                    model_content,
                    types.Content(role="user", parts=[function_response_part]),
                ],
                config=types.GenerateContentConfig(tools=gemini_tools),
            )
            print("최종 답변:", follow_up.text)

asyncio.run(main())
