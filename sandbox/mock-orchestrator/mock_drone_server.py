"""
Mock Drone MCP Server (v2)
============================
takeoff, land, get_status에 move_to / rotate / return_home 추가.
좌표 필드는 services/middleware/app/schemas/mission.py의 GeoPosition과
같은 이름(latitude, longitude, altitude_m)을 사용한다.
실제 AirSim 연동 시 각 함수 안의 상태 관리 코드를
airsim.MultirotorClient() 호출로 교체하면 됨.
"""

import sys

from fastmcp import FastMCP

mcp = FastMCP("mock-drone-server")

# 드론의 현재 상태. latitude/longitude/altitude_m은 GeoPosition과 같은
# 실제 지구 좌표계(WGS84) 기준 절대 위치. heading은 기수 방향(도, 0~359).
drone_state = {
    "airborne": False,
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude_m": 0.0,       # 고도. 양수 = 위로 올라간 높이
    "heading": 0.0, # 0=북쪽 기준, 시계방향 각도
}


def _debug_state(tool_name: str) -> None:
    """도구 실행 후 drone_state를 stderr로 출력한다.

    stdout은 MCP 프로토콜 통신에 쓰이므로, 여기서 print()를 쓰면
    프로토콜 메시지에 잡음이 섞여 통신이 깨진다. 반드시 stderr로 보낸다.
    """
    print(f"[DEBUG] after {tool_name}: {drone_state}", file=sys.stderr)


@mcp.tool()
def takeoff(altitude: float = 3.0) -> str:
    """드론을 이륙시켜 지정한 고도까지 상승시킵니다."""
    if drone_state["airborne"]:
        return f"이미 이륙한 상태입니다 (현재 고도 {drone_state['altitude_m']}m)."

    drone_state["airborne"] = True
    drone_state["altitude_m"] = altitude
    _debug_state("takeoff")

    return f"[MOCK] 드론이 이륙하여 고도 {altitude}m 까지 상승했습니다."


@mcp.tool()
def land() -> str:
    """드론을 현재 위치에서 착륙시킵니다."""
    if not drone_state["airborne"]:
        return "드론이 이미 지상에 있습니다."

    drone_state["airborne"] = False
    drone_state["altitude_m"] = 0.0
    _debug_state("land")

    return f"[MOCK] 드론이 (latitude={drone_state['latitude']}, longitude={drone_state['longitude']}) 위치에 착륙했습니다."


@mcp.tool()
def move_to(latitude: float, longitude: float, altitude_m: float = None) -> str:
    """
    드론을 지정한 좌표(latitude, longitude[, altitude_m])로 이동시킵니다.
    실제 지구 좌표계 기준 절대 위치

    Args:
        latitude: -90 ~ 90
        longitude: -180 ~ 180
        altitude_m: 목표 고도(미터). 생략하면 현재 고도를 유지합니다.
    """
    if not drone_state["airborne"]:
        return "드론이 이륙 상태가 아닙니다. 먼저 takeoff를 호출하세요."

    target_z = altitude_m if altitude_m is not None else drone_state["altitude_m"]

    drone_state["latitude"] = latitude
    drone_state["longitude"] = longitude
    drone_state["altitude_m"] = target_z
    _debug_state("move_to")

    return f"[MOCK] 드론이 (latitude={latitude}, longitude={longitude}, altitude={target_z}) 위치로 이동했습니다."


@mcp.tool()
def rotate(degrees: float) -> str:
    """
    드론의 기수 방향을 지정한 각도(도, 0~359)로 회전시킵니다.

    Args:
        degrees: 목표 방향(도). 0=북, 90=동, 180=남, 270=서.
    """
    if not drone_state["airborne"]:
        return "드론이 이륙 상태가 아닙니다. 먼저 takeoff를 호출하세요."

    normalized = degrees % 360

    drone_state["heading"] = normalized
    _debug_state("rotate")

    return f"[MOCK] 드론이 {normalized}도 방향으로 회전했습니다."


@mcp.tool()
def return_home() -> str:
    """드론을 이륙 지점(0,0)으로 복귀시킨 뒤 현재 고도를 유지한 채 대기시킵니다."""
    if not drone_state["airborne"]:
        return "드론이 이륙 상태가 아닙니다. 복귀할 필요가 없습니다."

    drone_state["latitude"] = 0.0
    drone_state["longitude"] = 0.0
    _debug_state("return_home")

    return f"[MOCK] 드론이 홈 위치(0, 0)로 복귀했습니다 (고도 {drone_state['altitude_m']}m 유지)."


@mcp.tool()
def get_status() -> str:
    """드론의 현재 상태(위치, 고도, 방향, 이륙 여부)를 조회합니다."""
    state = "비행 중" if drone_state["airborne"] else "지상 대기 중"
    return (
        f"[MOCK] 상태: {state} | "
        f"위치: (latitude={drone_state['latitude']}, longitude={drone_state['longitude']}) | "
        f"고도: {drone_state['altitude_m']}m | "
        f"방향: {drone_state['heading']}도"
    )


if __name__ == "__main__":
    mcp.run()
