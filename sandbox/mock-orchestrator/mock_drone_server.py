"""
Mock Drone MCP Server (v3)
============================
v2(takeoff/land/move_to/rotate/return_home/get_status)에
propose_mission_plan을 추가한다.

propose_mission_plan은 MissionPlan 스키마로 계획을 검증한 뒤,
"accepted"인 경우 각 assignment(drone_id + area_id)에 대해
내부적으로 이륙/이동을 실제로 실행한다 — LLM이 move_to를 직접
호출하지 않아도, 검증된 계획이 곧바로 물리적 동작으로 이어진다.

여러 드론을 동시에 다루기 위해 drone_state를 drone_id별 dict로 관리한다.
좌표 필드는 services/middleware/app/schemas/mission.py의 GeoPosition과
같은 이름(latitude, longitude, altitude_m)을 사용한다.
실제 AirSim 연동 시 각 함수 안의 상태 관리 코드를
airsim.MultirotorClient() 호출로 교체하면 됨.
"""

import sys
from pathlib import Path

from fastmcp import FastMCP
from pydantic import ValidationError
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

# repo root: sandbox/mock-orchestrator/mock_drone_server.py -> ../../
_MIDDLEWARE_ROOT = Path(__file__).resolve().parents[2] / "services" / "middleware"
sys.path.insert(0, str(_MIDDLEWARE_ROOT))

from app.schemas.mission import MissionPlan  # noqa: E402  (sys.path 설정 이후에 import해야 함)

mcp = FastMCP("mock-drone-server")

DEFAULT_DRONE_ID = "drone-1"

# 모든 탐색 구역은 다각형 경계로 정의한다. middleware가 완성되기 전까지
# mock 단계에서만 쓰는 하드코딩된 테스트 데이터이고, 실제로는 AirSim이
# 내보내는 SearchArea 경계(강 구역, 산 구역처럼 미리 나뉜 지형)로
# 대체될 예정이다. Shapely는 좌표를 (경도, 위도) = (longitude, latitude)
# 순서로 받는다는 점에 주의.
#
# area-a, area-b는 대칭 사각형으로 만들어서 중심이 기존에 쓰던 좌표
# (37.45/127.12, 37.46/127.13)와 정확히 같다 — 기존 테스트가 참조하는
# 값을 깨지 않기 위함이다.
# area-c는 이슈 #16 검증용 L자 모양(예: 강이나 능선 때문에 한쪽 모서리가
# 접근 불가능한 지형을 흉내낸 것)이다.
AREA_POLYGONS: dict[str, list[tuple[float, float]]] = {
    "area-a": [
        (127.115, 37.445),
        (127.115, 37.455),
        (127.125, 37.455),
        (127.125, 37.445),
    ],
    "area-b": [
        (127.125, 37.455),
        (127.125, 37.465),
        (127.135, 37.465),
        (127.135, 37.455),
    ],
    "area-c": [
        (127.10, 37.44),
        (127.10, 37.45),
        (127.105, 37.45),
        (127.105, 37.445),
        (127.11, 37.445),
        (127.11, 37.44),
    ],
}


def _area_center(polygon_coords: list[tuple[float, float]]) -> dict[str, float]:
    """다각형의 중심점을 {"latitude", "longitude"} 딕셔너리로 반환한다.

    Shapely의 centroid 계산은 내부적으로 삼각형 분해를 거치기 때문에,
    대칭 사각형처럼 "정확히 37.45"가 나와야 할 자리에도
    37.449999999999996 같은 부동소수점 오차가 섞여 나온다. GPS 좌표는
    소수점 6자리(약 11cm 정밀도)면 충분하므로 반올림해서 이 오차를 없앤다.
    """
    center = Polygon(polygon_coords).centroid
    return {"latitude": round(center.y, 6), "longitude": round(center.x, 6)}


# area_id -> 대표 좌표(중심점) 매핑. propose_mission_plan은 드론을 이
# 좌표로 이동시킨다. 구역을 통째로 한 드론에게 맡길 때는 이 중심점을
# 쓰고, 나눠서 맡기고 싶을 때는 먼저 split_search_area를 호출해 얻은
# 하위 area_id(예: "area-a-1")를 대신 쓰면 된다. AREA_POLYGONS로부터
# 자동 계산되므로, 구역을 추가/수정할 때는 AREA_POLYGONS만 고치면 된다.
#
# 표기를 하나로 고정한다 — LLM이 "A구역", "area_A" 등 매번 다른 표기로
# area_id를 지어내면 여기서 못 찾아 조용히 실행이 스킵되는 문제가 있었다.
# area_id를 새로 추가할 때는 propose_mission_plan()의 docstring에 적힌
# "사용 가능한 area_id" 목록도 반드시 같이 갱신해야 한다.
AREA_COORDINATES: dict[str, dict[str, float]] = {
    area_id: _area_center(coords) for area_id, coords in AREA_POLYGONS.items()
}

# 드론별 현재 상태. latitude/longitude/altitude_m은 GeoPosition과 같은
# 실제 지구 좌표계(WGS84) 기준 절대 위치. heading은 기수 방향(도, 0~359).
drone_states: dict[str, dict] = {}


def _new_state() -> dict:
    return {
        "airborne": False,
        "latitude": 0.0,
        "longitude": 0.0,
        "altitude_m": 0.0,       # 고도. 양수 = 위로 올라간 높이
        "heading": 0.0,          # 0=북쪽 기준, 시계방향 각도
    }


def _get_state(drone_id: str) -> dict:
    """drone_id에 해당하는 상태를 반환한다. 처음 보는 drone_id면 새로 만든다."""
    return drone_states.setdefault(drone_id, _new_state())


def _debug_state(tool_name: str) -> None:
    """도구 실행 후 전체 drone_states를 stderr로 출력한다.

    stdout은 MCP 프로토콜 통신에 쓰이므로, 여기서 print()를 쓰면
    프로토콜 메시지에 잡음이 섞여 통신이 깨진다. 반드시 stderr로 보낸다.
    """
    print(f"[DEBUG] after {tool_name}: {drone_states}", file=sys.stderr)


def _do_takeoff(drone_id: str, altitude: float) -> str:
    state = _get_state(drone_id)
    if state["airborne"]:
        return f"[{drone_id}] 이미 이륙한 상태입니다 (현재 고도 {state['altitude_m']}m)."

    state["airborne"] = True
    state["altitude_m"] = altitude
    _debug_state(f"takeoff({drone_id})")

    return f"[MOCK] {drone_id} 드론이 이륙하여 고도 {altitude}m 까지 상승했습니다."


def _do_move_to(
    drone_id: str, latitude: float, longitude: float, altitude_m: float | None
) -> str:
    state = _get_state(drone_id)
    if not state["airborne"]:
        return f"[{drone_id}] 드론이 이륙 상태가 아닙니다. 먼저 takeoff를 호출하세요."

    target_z = altitude_m if altitude_m is not None else state["altitude_m"]

    state["latitude"] = latitude
    state["longitude"] = longitude
    state["altitude_m"] = target_z
    _debug_state(f"move_to({drone_id})")

    return (
        f"[MOCK] {drone_id} 드론이 (latitude={latitude}, longitude={longitude}, "
        f"altitude={target_z}) 위치로 이동했습니다."
    )


@mcp.tool()
def takeoff(altitude: float = 3.0, drone_id: str = DEFAULT_DRONE_ID) -> str:
    """드론을 이륙시켜 지정한 고도까지 상승시킵니다.

    Args:
        altitude: 목표 고도(미터)
        drone_id: 대상 드론 식별자. 생략하면 기본 드론(drone-1)에 적용됩니다.
    """
    return _do_takeoff(drone_id, altitude)


@mcp.tool()
def land(drone_id: str = DEFAULT_DRONE_ID) -> str:
    """드론을 현재 위치에서 착륙시킵니다.

    Args:
        drone_id: 대상 드론 식별자. 생략하면 기본 드론(drone-1)에 적용됩니다.
    """
    state = _get_state(drone_id)
    if not state["airborne"]:
        return f"[{drone_id}] 드론이 이미 지상에 있습니다."

    state["airborne"] = False
    state["altitude_m"] = 0.0
    _debug_state(f"land({drone_id})")

    return (
        f"[MOCK] {drone_id} 드론이 (latitude={state['latitude']}, "
        f"longitude={state['longitude']}) 위치에 착륙했습니다."
    )


@mcp.tool()
def move_to(
    latitude: float,
    longitude: float,
    altitude_m: float = None,
    drone_id: str = DEFAULT_DRONE_ID,
) -> str:
    """
    드론을 지정한 좌표(latitude, longitude[, altitude_m])로 이동시킵니다.
    실제 지구 좌표계 기준 절대 위치

    Args:
        latitude: -90 ~ 90
        longitude: -180 ~ 180
        altitude_m: 목표 고도(미터). 생략하면 현재 고도를 유지합니다.
        drone_id: 대상 드론 식별자. 생략하면 기본 드론(drone-1)에 적용됩니다.
    """
    return _do_move_to(drone_id, latitude, longitude, altitude_m)


@mcp.tool()
def rotate(degrees: float, drone_id: str = DEFAULT_DRONE_ID) -> str:
    """
    드론의 기수 방향을 지정한 각도(도, 0~359)로 회전시킵니다.

    Args:
        degrees: 목표 방향(도). 0=북, 90=동, 180=남, 270=서.
        drone_id: 대상 드론 식별자. 생략하면 기본 드론(drone-1)에 적용됩니다.
    """
    state = _get_state(drone_id)
    if not state["airborne"]:
        return f"[{drone_id}] 드론이 이륙 상태가 아닙니다. 먼저 takeoff를 호출하세요."

    normalized = degrees % 360

    state["heading"] = normalized
    _debug_state(f"rotate({drone_id})")

    return f"[MOCK] {drone_id} 드론이 {normalized}도 방향으로 회전했습니다."


def _do_return_home(drone_id: str) -> str:
    state = _get_state(drone_id)
    if not state["airborne"]:
        return f"[{drone_id}] 드론이 이륙 상태가 아닙니다. 복귀할 필요가 없습니다."

    # 이륙 지점(0, 0)으로 현재 고도를 유지한 채 직선 복귀한다. 시뮬레이터 쪽
    # (kpi_mission.py의 goto(home))과 같은 방식이다. 탐색 경로가 지그재그라
    # 지나온 길을 역순으로 되짚으면 비행 시간과 배터리가 크게 늘어나고,
    # 지형 충돌은 최소 안전 고도(30m) 유지로 막는다.
    _do_move_to(drone_id, 0.0, 0.0, state["altitude_m"])
    _debug_state(f"return_home({drone_id})")

    return (
        f"[MOCK] {drone_id} 드론이 홈 위치(0, 0)로 직선 복귀했습니다 "
        f"(고도 {state['altitude_m']}m 유지)."
    )


@mcp.tool()
def return_home(drone_id: str = DEFAULT_DRONE_ID) -> str:
    """드론을 이륙 지점(0,0)으로 직선 복귀시킨 뒤 현재 고도를 유지한 채
    대기시킵니다.

    Args:
        drone_id: 대상 드론 식별자. 생략하면 기본 드론(drone-1)에 적용됩니다.
    """
    return _do_return_home(drone_id)


@mcp.tool()
def get_status(drone_id: str = DEFAULT_DRONE_ID) -> str:
    """드론의 현재 상태(위치, 고도, 방향, 이륙 여부)를 조회합니다.

    Args:
        drone_id: 대상 드론 식별자. 생략하면 기본 드론(drone-1)을 조회합니다.
    """
    state = _get_state(drone_id)
    flying = "비행 중" if state["airborne"] else "지상 대기 중"
    return (
        f"[MOCK] {drone_id} 상태: {flying} | "
        f"위치: (latitude={state['latitude']}, longitude={state['longitude']}) | "
        f"고도: {state['altitude_m']}m | "
        f"방향: {state['heading']}도"
    )


def _only_polygons(geom):
    """intersection 결과에서 면적이 있는 부분(Polygon/MultiPolygon)만 남긴다.

    컷 라인이 다각형의 변과 겹치면 결과가 GeometryCollection이 되는데
    (sandbox/shapely-101/split_test.py에서 확인한 현상), 그 안에 섞인
    LineString(면적 0)은 버리고 도형만 골라낸다. 아무 도형도 없으면
    None을 반환한다.
    """
    if geom.geom_type in ("Polygon", "MultiPolygon"):
        return geom

    polys = [g for g in geom.geoms if g.geom_type in ("Polygon", "MultiPolygon")]

    if len(polys) == 0:
        return None
    if len(polys) == 1:
        return polys[0]
    return unary_union(polys)


def _area_left_of(polygon, x):
    """다각형에서 x(경도) 왼쪽에 있는 부분의 면적."""
    minx, miny, _, maxy = polygon.bounds
    left = box(minx, miny, x, maxy)
    return polygon.intersection(left).area


def _find_cut_x(polygon, target_area, iterations=50):
    """왼쪽 누적 면적이 target_area가 되는 x(경도)를 이진 탐색으로 찾는다.

    _area_left_of(polygon, x)는 x가 커질수록 단조 증가하므로, 매 반복마다
    목표보다 작으면 오른쪽 절반을, 크거나 같으면 왼쪽 절반을 남기는
    방식으로 구간을 절반씩 좁혀 나간다.
    """
    minx, _, maxx, _ = polygon.bounds
    lo, hi = minx, maxx
    for _ in range(iterations):
        mid = (lo + hi) / 2
        if _area_left_of(polygon, mid) < target_area:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def _split_polygon_equal_area(polygon, n_pieces):
    """각 조각의 면적이 (거의) 같아지도록 폴리곤을 세로로 n등분한다."""
    minx, miny, maxx, maxy = polygon.bounds
    total_area = polygon.area

    cuts = [minx]
    for k in range(1, n_pieces):
        cuts.append(_find_cut_x(polygon, total_area * k / n_pieces))
    cuts.append(maxx)

    pieces = []
    for i in range(n_pieces):
        strip = box(cuts[i], miny, cuts[i + 1], maxy)
        piece = _only_polygons(polygon.intersection(strip))
        if piece is not None:
            pieces.append(piece)
    return pieces


@mcp.tool()
def split_search_area(area_id: str, n_pieces: int = 2) -> dict:
    """탐색 구역 하나를 면적이 같은 n개의 하위 구역으로 나누고,
    각 조각을 새 area_id로 등록합니다. 드론이 여러 대라 구역을
    나눠 맡겨야 할 때, 이 도구로 먼저 구역을 쪼갠 뒤 반환된
    새 area_id들을 propose_mission_plan의 assignments에 사용하세요.

    나눌 수 있는 area_id 목록: "area-a", "area-b", "area-c"
    (AREA_POLYGONS에 등록된 모든 구역)

    Args:
        area_id: 나눌 구역의 식별자. AREA_POLYGONS에 등록된 값이어야 합니다.
        n_pieces: 나눌 조각 수 (2 이상).
    """
    polygon_coords = AREA_POLYGONS.get(area_id)
    if polygon_coords is None:
        return {
            "status": "rejected",
            "error_code": "UNKNOWN_AREA_ID",
            "message": (
                f"'{area_id}'는 분할 가능한 구역이 아닙니다. "
                f"{sorted(AREA_POLYGONS)} 중 하나여야 합니다."
            ),
        }
    if n_pieces < 2:
        return {
            "status": "rejected",
            "error_code": "INVALID_N_PIECES",
            "message": "n_pieces는 2 이상이어야 합니다.",
        }

    polygon = Polygon(polygon_coords)
    pieces = _split_polygon_equal_area(polygon, n_pieces)

    new_area_ids = []
    for i, piece in enumerate(pieces, start=1):
        new_id = f"{area_id}-{i}"
        # Shapely 좌표는 (경도, 위도) 순서이므로 뒤집어서 저장한다.
        # round()로 부동소수점 오차를 정리한다 (_area_center 참고).
        center = piece.centroid
        latitude, longitude = round(center.y, 6), round(center.x, 6)
        AREA_COORDINATES[new_id] = {"latitude": latitude, "longitude": longitude}
        new_area_ids.append(
            {
                "area_id": new_id,
                "latitude": latitude,
                "longitude": longitude,
                "area_ratio": round(piece.area / polygon.area, 3),
            }
        )

    _debug_state(f"split_search_area({area_id})")
    return {
        "status": "ok",
        "base_area_id": area_id,
        "new_area_ids": new_area_ids,
    }


@mcp.tool()
def propose_mission_plan(
    mission_id: str, assignments: list[dict], generated_at: str
) -> dict:
    """
    드론별 임무 배정 계획을 제안합니다.
    LLM은 좌표 계산 없이 어떤 드론을 어떤 구역에 배정할지만 결정합니다.
    계획이 검증을 통과하면(accepted), 각 배정에 대해 이 도구가 내부적으로
    이륙과 이동을 실제로 실행합니다 — move_to는 LLM이 직접 호출하지 않아도
    검증된 계획으로부터 자동으로 실행됩니다.

    중요: area_id는 반드시 아래 목록에 있는 값 그대로 사용하세요.
    "A구역", "area_A"처럼 다른 표기를 쓰면 실제 좌표를 찾지 못해 계획은
    accepted로 응답하더라도 드론이 실제로 이동하지 않습니다.
      - "area-a"
      - "area-b"
      - split_search_area 도구로 새로 생성한 area_id (예: "area-c-1", "area-c-2")

    Args:
        mission_id: 임무 식별자
        assignments: [{"drone_id": str, "area_id": str, "priority": int}, ...]
            area_id는 반드시 "area-a" 또는 "area-b" 중 하나여야 합니다.
        generated_at: ISO8601 타임존 포함 시각 (예: "2026-09-06T10:00:00+09:00")
    """
    try:
        plan = MissionPlan(
            mission_id=mission_id,
            assignments=assignments,
            generated_at=generated_at,
        )
    except ValidationError as exc:
        result = {
            "status": "rejected",
            "error_code": "VALIDATION_ERROR",
            "message": str(exc),
            "mission_id": mission_id,
        }
        _debug_state("propose_mission_plan(rejected)")
        return result

    execution_log = []
    skipped_area_ids = []
    for assignment in plan.assignments:
        area = AREA_COORDINATES.get(assignment.area_id)
        if area is None:
            skipped_area_ids.append(assignment.area_id)
            execution_log.append(
                f"{assignment.drone_id}: area_id '{assignment.area_id}'는 알 수 없는 "
                f"구역입니다 ({sorted(AREA_COORDINATES)} 중 하나여야 함). 실행하지 "
                "않았습니다."
            )
            continue

        execution_log.append(_do_takeoff(assignment.drone_id, altitude=30.0))
        execution_log.append(
            _do_move_to(
                assignment.drone_id,
                area["latitude"],
                area["longitude"],
                30.0,
            )
        )

    result = {
        # 계획 자체(스키마)는 유효해서 accepted지만, area_id를 몰라 하나라도
        # 실행을 못 했으면 warning으로 명시한다. 이걸 execution_log 안에만
        # 묻어두면 LLM이 최종 요약에서 빼먹기 쉬워서(실제로 그랬음) 별도
        # 필드로 분리했다.
        "status": "accepted",
        "warning": (
            f"알 수 없는 area_id로 인해 실행되지 않은 배정이 있습니다: {skipped_area_ids}. "
            "사용자에게 반드시 알리세요."
            if skipped_area_ids
            else None
        ),
        "mission_id": plan.mission_id,
        "assignments": [a.model_dump() for a in plan.assignments],
        "execution_log": execution_log,
    }
    _debug_state("propose_mission_plan(accepted)")
    return result


if __name__ == "__main__":
    mcp.run()
