"""제일 단순한 MCP 서버. 도구가 딱 하나(add)뿐이다."""

from fastmcp import FastMCP

mcp = FastMCP("demo-server")


@mcp.tool()
def takeoff(altitude: float = 3.0) -> str:
    """드론을 지정한 고도까지 이륙시킵니다"""
    return f"""드론이 고도 {altitude}m까지 이륙했습니다."""


if __name__ == "__main__":
    mcp.run()
