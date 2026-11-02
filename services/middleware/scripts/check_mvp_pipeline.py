"""End-to-end check of the middleware MCP server without an LLM.

Calls the three tools with hand-written arguments:

    context -> invalid plan (rejected) -> valid plan (accepted) -> status until done

Two ways to reach the server:

- default: start ``python -m app.mcp_server`` as a stdio subprocess, exactly as an
  orchestrator without ``MCP_SERVER_URL`` would.
- ``--url`` (or ``MCP_SERVER_URL``): connect to a server already running with
  ``MCP_TRANSPORT=http``. Nothing is started; the drones are the ones that server holds.

Run from the repository root::

    .venv/bin/python services/middleware/scripts/check_mvp_pipeline.py
    python3 services/middleware/scripts/check_mvp_pipeline.py --url http://127.0.0.1:9000/mcp

Exit code 0 means every command of the tasks accepted in this run succeeded. Only those
tasks are judged: an HTTP server keeps earlier tasks of the same mission, and the
mission-level ``state`` stays ``failed`` once any of them failed. The default registry
config uses FakeDrone; set ``MIDDLEWARE_DRONES_CONFIG`` to try other configurations
(stdio only; an HTTP server uses the config it was started with).
"""

import argparse
import asyncio
import json
import os
import sys
import time
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
from mcp.client.streamable_http import streamable_http_client

MIDDLEWARE_ROOT = Path(__file__).resolve().parents[1]
MISSION_ID = "mission-001"
TIMEOUT_S = 180


def server_parameters() -> StdioServerParameters:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        filter(None, [str(MIDDLEWARE_ROOT), env.get("PYTHONPATH")])
    )
    # An MCP_TRANSPORT=http exported for the HTTP server must not leak into this child,
    # or it would listen on a port while the client waits on its stdout.
    env["MCP_TRANSPORT"] = "stdio"
    return StdioServerParameters(
        command=sys.executable, args=["-m", "app.mcp_server"], env=env
    )


@asynccontextmanager
async def connect(url: str | None) -> AsyncIterator[tuple[Any, Any]]:
    """Yield ``(read, write)`` streams for ``url`` over HTTP, or a stdio subprocess."""
    if url is None:
        async with stdio_client(server_parameters()) as (read, write):
            yield read, write
        return
    # mcp 1.x yields (read, write, get_session_id); mcp 2.x yields (read, write).
    async with streamable_http_client(url) as streams:
        yield streams[0], streams[1]


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


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--url",
        default=os.environ.get("MCP_SERVER_URL") or None,
        help="streamable HTTP endpoint of a running server, e.g. "
        "http://127.0.0.1:9000/mcp (default: $MCP_SERVER_URL; unset = stdio subprocess)",
    )
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S)
    return parser.parse_args(argv)


async def run(url: str | None, timeout_s: float) -> int:
    print(f"[0] transport: {'http ' + url if url else 'stdio (subprocess)'}")
    async with connect(url) as (read, write):
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
            task_ids = {task["task_id"] for task in accepted["tasks"]}

            started = time.monotonic()
            while True:
                status = await call(session, "get_mission_status", {"mission_id": MISSION_ID})
                tasks = [task for task in status["tasks"] if task["task_id"] in task_ids]
                progress = ", ".join(
                    f"{task['drone_id']} "
                    f"{sum(c['status'] == 'succeeded' for c in task['commands'])}"
                    f"/{len(task['commands'])}"
                    for task in tasks
                )
                print(f"[5] {time.monotonic() - started:5.1f}s {status['state']:9s} {progress}")
                if tasks and all(task["state"] != "running" for task in tasks):
                    break
                if time.monotonic() - started > timeout_s:
                    print("    timed out waiting for the mission")
                    return 1
                await asyncio.sleep(2)

            for task in tasks:
                for command in task["commands"]:
                    if command["status"] != "succeeded":
                        error = command["history"][-1].get("error")
                        print(f"    {command['command_id']} {command['status']} {error}")
            succeeded = all(task["state"] == "succeeded" for task in tasks)
            print(f"[6] this run's tasks {'succeeded' if succeeded else 'failed'} "
                  f"(mission state {status['state']})")
            return 0 if succeeded else 1


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    return asyncio.run(run(args.url, args.timeout))


if __name__ == "__main__":
    sys.exit(main())
