"""Tests for the MCP tool surface, using fastmcp's in-memory client."""

import asyncio
import time

from conftest import plan
from fastmcp import Client

from app.mcp_server import create_mcp_server


def run(coroutine):
    return asyncio.run(coroutine)


def test_exposes_three_tools_with_input_schemas(service) -> None:
    async def list_tools():
        async with Client(create_mcp_server(service)) as client:
            return await client.list_tools()

    tools = {tool.name: tool for tool in run(list_tools())}

    assert set(tools) == {"get_mission_context", "submit_mission_plan", "get_mission_status"}
    submit = tools["submit_mission_plan"].inputSchema
    assert submit["required"] == ["mission_id", "assignments", "generated_at"]
    item = submit["properties"]["assignments"]["items"]
    item = submit.get("$defs", {}).get(item.get("$ref", "").split("/")[-1], item)
    assert set(item["properties"]) == {"drone_id", "area_id", "priority"}
    assert set(item["required"]) == {"drone_id", "area_id"}


def test_full_pipeline_through_mcp_tools(service) -> None:
    async def scenario():
        async with Client(create_mcp_server(service)) as client:
            context = (await client.call_tool(
                "get_mission_context", {"mission_id": "mission-001"}
            )).data
            rejected = (await client.call_tool(
                "submit_mission_plan", plan(("drone-01", "area-west"))
            )).data
            accepted = (await client.call_tool(
                "submit_mission_plan",
                plan(("drone-01", "area-north"), ("drone-02", "area-south")),
            )).data
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                status = (await client.call_tool(
                    "get_mission_status", {"mission_id": "mission-001"}
                )).data
                if status["state"] not in ("running", "idle"):
                    break
                await asyncio.sleep(0.1)
            return context, rejected, accepted, status

    context, rejected, accepted, status = run(scenario())

    assert [d["drone_id"] for d in context["drones"]] == ["drone-01", "drone-02"]
    assert [a["area_id"] for a in context["search_areas"]] == ["area-north", "area-south"]
    assert rejected["status"] == "rejected"
    assert rejected["errors"][0]["code"] == "UNKNOWN_AREA"
    assert accepted["status"] == "accepted"
    assert accepted["attempt"] == 2
    assert status["state"] == "succeeded"
    assert all(task["state"] == "succeeded" for task in status["tasks"])


def test_unknown_mission_is_a_tool_error(service) -> None:
    async def call():
        async with Client(create_mcp_server(service)) as client:
            return await client.call_tool(
                "get_mission_status", {"mission_id": "mission-404"}, raise_on_error=False
            )

    result = run(call())
    assert result.is_error
    assert "mission-404" in result.content[0].text
