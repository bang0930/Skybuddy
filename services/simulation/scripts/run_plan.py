"""미들웨어 MCP 서버에 임무 계획을 직접 제출하고, 끝날 때까지 드론 상태를 보여 준다 (LLM 없이 리허설).

  python3 run_plan.py mission-101 drone-01=sec-sw drone-02=sec-se
  python3 run_plan.py mission-101 drone-02=sec-ne:90 drone-01=sec-sw:10    # :숫자 = 우선순위(클수록 먼저)

미들웨어 HTTP 서버(MCP_TRANSPORT=http)가 떠 있어야 한다. MCP 연결 코드는 팀 저장소의
services/middleware/scripts/check_mvp_pipeline.py 의 connect·call 을 그대로 가져다 쓴다.
상태 줄은 바뀔 때만 출력한다 (배터리·연결 상태가 미들웨어에 어떻게 보이는지 확인용).
"""
import argparse
import asyncio
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
# 팀 저장소(services/simulation/scripts)면 같은 저장소의 미들웨어, 개발 환경이면 ~/skybuddy-team 의 것을 쓴다.
_CANDIDATES = [os.environ.get("SKYBUDDY_TEAM_SCRIPTS"),
               os.path.join(os.path.dirname(os.path.dirname(_HERE)), "middleware", "scripts"),
               os.path.expanduser("~/skybuddy-team/services/middleware/scripts")]
TEAM_SCRIPTS = next((c for c in _CANDIDATES
                     if c and os.path.exists(os.path.join(c, "check_mvp_pipeline.py"))),
                    _CANDIDATES[-1])
sys.path.insert(0, TEAM_SCRIPTS)
try:
    from check_mvp_pipeline import call, connect
    from mcp import ClientSession
except ImportError as e:
    sys.exit(f"미들웨어 스크립트를 불러오지 못했다 ({e}). {TEAM_SCRIPTS} 와 mcp 설치를 확인할 것")


def assignment(text):
    drone, _, rest = text.partition("=")
    area, _, priority = rest.partition(":")
    if not drone or not area:
        raise argparse.ArgumentTypeError(f"드론=구역[:우선순위] 형식이어야 한다: {text}")
    item = {"drone_id": drone, "area_id": area}
    if priority:
        item["priority"] = int(priority)
    return item


def drone_text(d):
    pos = d.get("position") or {}
    alt = pos.get("altitude_m")
    battery = d.get("battery_percent")
    return (f"{d['drone_id']} {d['status']}/{d['connection_status']} "
            f"배터리 {'-' if battery is None else f'{battery:g}%'} "
            f"고도 {'-' if alt is None else f'{alt:.0f}m'}")


async def run(url, mission_id, assignments, timeout_s):
    async with connect(url) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            context = await call(session, "get_mission_context", {"mission_id": mission_id})
            print(f"[1] {mission_id} 구역: {[a['area_id'] for a in context['search_areas']]}")
            for d in context["drones"]:
                print(f"    {drone_text(d)}")

            result = await call(session, "submit_mission_plan",
                                {"mission_id": mission_id, "assignments": assignments})
            print(f"[2] 계획 {result['status']} (시도 {result['attempt']})")
            if result["status"] != "accepted":
                for error in result.get("errors") or []:
                    print(f"    {error['code']}: {error['message']}")
                return 1
            task_ids = {task["task_id"] for task in result["tasks"]}
            for task in result["tasks"]:
                print(f"    {task['task_id']}: {task['area_id']}, 명령 {task['command_count']}개")

            started, last = time.monotonic(), None
            while True:
                status = await call(session, "get_mission_status", {"mission_id": mission_id})
                tasks = [t for t in status["tasks"] if t["task_id"] in task_ids]
                drones = {d["drone_id"]: d for d in status["drones"]}
                line = " | ".join(
                    f"{t['drone_id']} {sum(c['status'] == 'succeeded' for c in t['commands'])}"
                    f"/{len(t['commands'])} {t['state']} ({drone_text(drones[t['drone_id']])})"
                    for t in tasks)
                if line != last:
                    print(f"[3] {time.monotonic() - started:6.1f}s {line}")
                    last = line
                if tasks and all(t["state"] != "running" for t in tasks):
                    break
                if time.monotonic() - started > timeout_s:
                    print("    시간 초과로 지켜보기를 멈춘다 (비행은 서버에서 계속된다)")
                    return 1
                await asyncio.sleep(2)

            for task in tasks:
                for command in task["commands"]:
                    if command["status"] != "succeeded":
                        error = command["history"][-1].get("error") or {}
                        print(f"    {command['command_id']} {command['command_type']} {command['status']} "
                              f"{error.get('code')}: {error.get('message')}")
            ok = all(task["state"] == "succeeded" for task in tasks)
            print(f"[4] 이번 작업 {'모두 성공' if ok else '실패 있음'}")
            return 0 if ok else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="임무 계획 제출 (LLM 없이)")
    parser.add_argument("mission_id")
    parser.add_argument("assignments", nargs="+", type=assignment, help="드론=구역[:우선순위]")
    parser.add_argument("--url", default=os.environ.get("MCP_SERVER_URL") or "http://127.0.0.1:9000/mcp")
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args(argv)
    return asyncio.run(run(args.url, args.mission_id, args.assignments, args.timeout))


if __name__ == "__main__":
    sys.exit(main())
