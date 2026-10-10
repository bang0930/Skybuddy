"""End-to-end check of the middleware MCP server without an LLM.

Starts ``python -m app.mcp_server`` as a stdio subprocess, exactly as the orchestrator
would, and calls the three tools with hand-written arguments:

    context -> invalid plan (rejected) -> valid plan (accepted) -> status until done

Run from the repository root::

    .venv/bin/python services/middleware/scripts/check_mvp_pipeline.py

Exit code 0 means every command of every task succeeded. The default registry config
uses FakeDrone; set ``MIDDLEWARE_DRONES_CONFIG`` to try other configurations.
"""

import asyncio
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

MIDDLEWARE_ROOT = Path(__file__).resolve().parents[1]
MISSION_ID = "mission-001"
TIMEOUT_S = 180


def server_parameters() -> StdioServerParameters:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        filter(None, [str(MIDDLEWARE_ROOT), env.get("PYTHONPATH")])
    )
    return StdioServerParameters(
        command=sys.executable, args=["-m", "app.mcp_server"], env=env
    )


async def call(session: ClientSession, name: str, arguments: dict) -> dict:
    result = await session.call_tool(name, arguments)
    if result.isError:
        raise RuntimeError(f"{name} failed: {result.content[0].text}")
    return result.structuredContent or json.loads(result.content[0].text)


def plan(assignments: list[dict]) -> dict:
    return {
        "mission_id": MISSION_ID,
        "assignments": assignments,
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }


async def main() -> int:
    async with stdio_client(server_parameters()) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            tools = [tool.name for tool in (await session.list_tools()).tools]
            print(f"[1] tools: {tools}")

            context = await call(session, "get_mission_context", {"mission_id": MISSION_ID})
            drones = [d["drone_id"] for d in context["drones"]]
            areas = [a["area_id"] for a in context["search_areas"]]
            for drone in context["drones"]:
                print(
                    f"[2] {drone['drone_id']} {drone['protocol']:7s} "
                    f"{drone['status']}/{drone['connection_status']} "
                    f"commands={drone['capabilities']['commands']}"
                )
            print(f"    areas: {areas}")

            rejected = await call(
                session,
                "submit_mission_plan",
                plan([{"drone_id": drones[0], "area_id": "area-does-not-exist"}]),
            )
            codes = [error["code"] for error in rejected["errors"]]
            print(f"[3] invalid plan -> {rejected['status']} {codes}")
            if rejected["status"] != "rejected":
                return 1

            accepted = await call(
                session,
                "submit_mission_plan",
                plan(
                    [
                        {"drone_id": drone_id, "area_id": area_id}
                        for drone_id, area_id in zip(drones, areas)
                    ]
                ),
            )
            print(f"[4] valid plan -> {accepted['status']} (attempt {accepted['attempt']})")
            for task in accepted["tasks"]:
                print(f"    {task['task_id']}: {task['area_id']}, {task['command_count']} commands")
            if accepted["status"] != "accepted":
                print(f"    errors: {accepted['errors']}")
                return 1

            started = time.monotonic()
            while True:
                status = await call(session, "get_mission_status", {"mission_id": MISSION_ID})
                progress = ", ".join(
                    f"{task['drone_id']} "
                    f"{sum(c['status'] == 'succeeded' for c in task['commands'])}"
                    f"/{len(task['commands'])}"
                    for task in status["tasks"]
                )
                print(f"[5] {time.monotonic() - started:5.1f}s {status['state']:9s} {progress}")
                if status["state"] not in ("running", "idle"):
                    break
                if time.monotonic() - started > TIMEOUT_S:
                    print("    timed out waiting for the mission")
                    return 1
                await asyncio.sleep(2)

            for task in status["tasks"]:
                for command in task["commands"]:
                    if command["status"] != "succeeded":
                        error = command["history"][-1].get("error")
                        print(f"    {command['command_id']} {command['status']} {error}")
            print(f"[6] mission {status['state']}")
            return 0 if status["state"] == "succeeded" else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
