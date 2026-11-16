"""HTTP transport: the server outlives client sessions, and its start/stop are recorded.

These tests start the real entry point (``python -m app.mcp_server``) as a subprocess
with ``MCP_TRANSPORT=http`` on a free port, using fake drones.
"""

import asyncio
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
from conftest import fast_registry_config, scenario
from fastmcp import Client

from app.mcp_server import TransportSettings, transport_from_env

MIDDLEWARE = Path(__file__).resolve().parents[1]
CHECK_SCRIPT = MIDDLEWARE / "scripts" / "check_mvp_pipeline.py"
NO_SIGINT = pytest.mark.skipif(os.name == "nt", reason="sends SIGINT to the server")


def run(coroutine):
    return asyncio.run(coroutine)


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def read_events(path: Path) -> list[dict]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


class HttpServer:
    def __init__(self, tmp_path: Path, *, horizontal_speed_m_s: float = 50.0) -> None:
        self.port = free_port()
        self.url = f"http://127.0.0.1:{self.port}/mcp"
        self.event_log = tmp_path / "events.jsonl"
        self.stderr_path = tmp_path / "server.stderr.log"

        config = fast_registry_config().model_dump(mode="json")
        for drone in config["drones"]:
            drone["fake"]["horizontal_speed_m_s"] = horizontal_speed_m_s
        config_path = tmp_path / "drones.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        scenarios = tmp_path / "scenarios"
        scenarios.mkdir()
        (scenarios / "mission-001.json").write_text(scenario().model_dump_json(), "utf-8")

        self.env = {
            **os.environ,
            "PYTHONPATH": os.pathsep.join(
                filter(None, [str(MIDDLEWARE), os.environ.get("PYTHONPATH")])
            ),
            "MCP_TRANSPORT": "http",
            "MCP_HOST": "127.0.0.1",
            "MCP_PORT": str(self.port),
            "MIDDLEWARE_DRONES_CONFIG": str(config_path),
            "MIDDLEWARE_SCENARIOS_DIR": str(scenarios),
            "MIDDLEWARE_EVENT_LOG": str(self.event_log),
        }
        self._stderr = self.stderr_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            [sys.executable, "-m", "app.mcp_server"],
            env=self.env,
            stdout=subprocess.DEVNULL,
            stderr=self._stderr,
        )
        self._wait_until_listening()

    def _wait_until_listening(self, timeout_s: float = 30) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise AssertionError(f"server exited early:\n{self.stderr()}")
            with socket.socket() as sock:
                if sock.connect_ex(("127.0.0.1", self.port)) == 0:
                    return
            time.sleep(0.1)
        raise AssertionError(f"server did not listen on {self.port}:\n{self.stderr()}")

    def stop(self) -> int:
        """Stop as Ctrl+C would; fall back to kill so a test never hangs."""
        if self.process.poll() is None:
            if os.name == "nt":
                self.process.terminate()
            else:
                self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self._stderr.close()
        return self.process.returncode

    def stderr(self) -> str:
        if not self._stderr.closed:
            self._stderr.flush()
        return self.stderr_path.read_text(encoding="utf-8")

    def events(self, name: str) -> list[dict]:
        return [event for event in read_events(self.event_log) if event["event"] == name]


@pytest.fixture
def http_server(tmp_path):
    server = HttpServer(tmp_path)
    yield server
    server.stop()


@pytest.fixture
def slow_http_server(tmp_path):
    # Climbs stay fast but legs take seconds, so the server stops between polls of a
    # goto. (close_all() waits for a blocking takeoff or land that holds a client.)
    server = HttpServer(tmp_path, horizontal_speed_m_s=2.0)
    yield server
    server.stop()


def submit(url: str) -> dict:
    async def call():
        async with Client(url) as client:
            return (await client.call_tool(
                "submit_mission_plan",
                {
                    "mission_id": "mission-001",
                    "assignments": [
                        {"drone_id": "drone-01", "area_id": "area-north"},
                        {"drone_id": "drone-02", "area_id": "area-south"},
                    ],
                },
            )).data

    return run(call())


# ------------------------------------------------------------------ MCP_TRANSPORT


def test_transport_defaults_to_stdio() -> None:
    assert transport_from_env({}) == TransportSettings("stdio")
    assert transport_from_env({"MCP_TRANSPORT": "stdio", "MCP_PORT": "x"}).port is None


def test_http_transport_uses_mock_server_defaults() -> None:
    settings = transport_from_env({"MCP_TRANSPORT": "http"})

    assert settings == TransportSettings("http", "127.0.0.1", 9000)
    assert settings.url == "http://127.0.0.1:9000/mcp"
    assert transport_from_env(
        {"MCP_TRANSPORT": "http", "MCP_HOST": "0.0.0.0", "MCP_PORT": "9100"}
    ) == TransportSettings("http", "0.0.0.0", 9100)


@pytest.mark.parametrize("value", ["HTTP", "sse", "streamable-http", "tcp"])
def test_unknown_transport_is_rejected(value: str) -> None:
    with pytest.raises(ValueError, match="MCP_TRANSPORT must be one of stdio, http"):
        transport_from_env({"MCP_TRANSPORT": value})


@pytest.mark.parametrize("port", ["abc", "0", "70000"])
def test_bad_http_port_is_rejected(port: str) -> None:
    with pytest.raises(ValueError, match="MCP_PORT"):
        transport_from_env({"MCP_TRANSPORT": "http", "MCP_PORT": port})


def test_unknown_transport_stops_before_connecting_drones(tmp_path) -> None:
    event_log = tmp_path / "events.jsonl"
    result = subprocess.run(
        [sys.executable, "-m", "app.mcp_server"],
        env={
            **os.environ,
            "PYTHONPATH": str(MIDDLEWARE),
            "MCP_TRANSPORT": "websocket",
            "MIDDLEWARE_EVENT_LOG": str(event_log),
        },
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 2
    assert "MCP_TRANSPORT must be one of stdio, http; got 'websocket'" in result.stderr
    assert read_events(event_log) == []  # no server_started: no drone was touched


# -------------------------------------------------------------------- HTTP server


def test_mission_continues_after_the_client_disconnects(http_server) -> None:
    async def first_session():
        async with Client(http_server.url) as client:
            tools = {tool.name for tool in await client.list_tools()}
            accepted = (await client.call_tool(
                "submit_mission_plan",
                {
                    "mission_id": "mission-001",
                    "assignments": [
                        {"drone_id": "drone-01", "area_id": "area-north"},
                        {"drone_id": "drone-02", "area_id": "area-south"},
                    ],
                },
            )).data
            return tools, accepted

    async def later_session():
        async with Client(http_server.url) as client:
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                status = (await client.call_tool(
                    "get_mission_status", {"mission_id": "mission-001"}
                )).data
                if status["state"] != "running":
                    return status
                await asyncio.sleep(0.2)
            return status

    tools, accepted = run(first_session())
    # The first session is closed here; over stdio the server would be gone with it.
    status = run(later_session())

    assert tools == {"get_mission_context", "submit_mission_plan", "get_mission_status"}
    assert accepted["status"] == "accepted"
    assert status["state"] == "succeeded"
    assert {task["task_id"] for task in status["tasks"]} == {
        task["task_id"] for task in accepted["tasks"]
    }
    assert all(task["state"] == "succeeded" for task in status["tasks"])

    started = http_server.events("server_started")
    assert len(started) == 1
    assert (started[0]["transport"], started[0]["host"], started[0]["port"]) == (
        "http", "127.0.0.1", http_server.port
    )


@NO_SIGINT
def test_ctrl_c_records_nothing_left_running_after_completion(http_server) -> None:
    accepted = submit(http_server.url)
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and len(http_server.events("task_finished")) < len(
        accepted["tasks"]
    ):
        time.sleep(0.2)

    assert http_server.stop() == 0
    stopping = http_server.events("server_stopping")
    assert stopping[-1]["transport"] == "http"
    assert stopping[-1]["running_tasks"] == []
    assert "경고" not in http_server.stderr()


@NO_SIGINT
def test_ctrl_c_during_a_flight_records_running_tasks_and_warns(slow_http_server) -> None:
    accepted = submit(slow_http_server.url)
    time.sleep(2)  # past takeoff, inside the first area leg

    assert slow_http_server.stop() == 0
    stopping = slow_http_server.events("server_stopping")[-1]
    running = {task["task_id"]: task for task in stopping["running_tasks"]}
    assert set(running) == {task["task_id"] for task in accepted["tasks"]}
    assert all(task["current_command"]["status"] for task in running.values())
    stderr = slow_http_server.stderr()
    assert "자동 착륙은 하지 않으므로" in stderr
    assert all(drone in stderr for drone in ("drone-01", "drone-02"))


def test_check_script_runs_against_a_running_http_server(http_server) -> None:
    result = subprocess.run(
        [sys.executable, str(CHECK_SCRIPT), "--url", http_server.url, "--timeout", "60"],
        env={**http_server.env, "MCP_TRANSPORT": ""},
        capture_output=True,
        text=True,
        timeout=120,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert f"[0] transport: http {http_server.url}" in result.stdout
    assert "invalid plan -> rejected ['UNKNOWN_AREA']" in result.stdout
    assert "this run's tasks succeeded" in result.stdout
    # The script did not start a second server: only one server_started was logged.
    assert len(http_server.events("server_started")) == 1
