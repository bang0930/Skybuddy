"""Run the MVP pipeline over stdio MCP and keep everything needed as integration evidence.

Same flow as ``check_mvp_pipeline.py``:

    context -> invalid plan (rejected) -> valid plan (accepted) -> status until done

but only for the drones chosen with ``--drones``, and every request, response, timing,
and the run's event log are saved under
``services/middleware/evidence/<timestamp>_<scenario>/`` with a ``summary.md``.

Run from the repository root::

    python3 services/middleware/scripts/collect_integration_evidence.py --drones drone-01
    python3 services/middleware/scripts/collect_integration_evidence.py --drones drone-02
    python3 services/middleware/scripts/collect_integration_evidence.py --drones drone-01,drone-02

``MIDDLEWARE_DRONES_CONFIG``, ``MIDDLEWARE_SCENARIOS_DIR``, and ``MIDDLEWARE_EVENT_LOG``
work as for the server. Only the selected drones are registered: the server receives a
copy of the drones config that lists just those drones (``drones.selected.json``).

Exit code 0 means the mission succeeded and the identifier isolation check passed.
"""

import argparse
import asyncio
import csv
import json
import os
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path
from typing import Any

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

MIDDLEWARE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = MIDDLEWARE_ROOT.parents[1]
sys.path.insert(0, str(MIDDLEWARE_ROOT))

from app.integration_evidence import (  # noqa: E402
    IsolationCheck,
    check_identifier_isolation,
    command_summaries,
    connection_gaps,
    read_events,
    registered_drones,
    transition_rows,
)
from app.mcp_server import (  # noqa: E402
    DEFAULT_DRONES_CONFIG,
    DEFAULT_EVENT_LOG,
    DEFAULT_SCENARIOS_DIR,
)
from app.runtime import RegistryConfig  # noqa: E402

DEFAULT_OUTPUT_ROOT = MIDDLEWARE_ROOT / "evidence"
INVALID_AREA_ID = "area-does-not-exist"
PACKAGES = ("mcp", "fastmcp", "pydantic", "pymavlink", "shapely")


class ToolCallError(RuntimeError):
    """An MCP tool returned an error result (recorded before raising)."""


class Recorder:
    """Call MCP tools and save each request, response, and duration."""

    def __init__(self, session: ClientSession, run_dir: Path, started: float) -> None:
        self.session = session
        self.calls_dir = run_dir / "tool_calls"
        self.calls_dir.mkdir()
        self.started = started
        self.calls: list[dict[str, Any]] = []

    async def call(self, name: str, arguments: dict, *, purpose: str) -> dict:
        at = datetime.now(timezone.utc)
        begin = time.perf_counter()
        result = await self.session.call_tool(name, arguments)
        duration_ms = (time.perf_counter() - begin) * 1000
        if result.isError:
            response: Any = {"error": result.content[0].text if result.content else ""}
        else:
            response = result.structuredContent or json.loads(result.content[0].text)
        record = {
            "seq": len(self.calls) + 1,
            "tool": name,
            "purpose": purpose,
            "at": at.isoformat(),
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "duration_ms": round(duration_ms, 1),
            "is_error": bool(result.isError),
            "request": arguments,
            "response": response,
        }
        self.calls.append(record)
        path = self.calls_dir / f"{record['seq']:03d}_{name}.json"
        path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        if result.isError:
            raise ToolCallError(f"{name} failed: {response['error']}")
        return response


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--drones",
        help="comma-separated drone IDs to fly (default: every drone in the config)",
    )
    parser.add_argument("--mission-id", default="mission-001")
    parser.add_argument(
        "--timeout", type=float, default=900, help="seconds to wait for the mission"
    )
    parser.add_argument(
        "--poll-interval", type=float, default=1.0, help="seconds between status calls"
    )
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--label", help="scenario name in the folder (default: from drones)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config_path = Path(os.environ.get("MIDDLEWARE_DRONES_CONFIG", DEFAULT_DRONES_CONFIG))
    raw_config = json.loads(config_path.read_text(encoding="utf-8"))
    config = RegistryConfig.model_validate(raw_config)
    known = [drone.drone_id for drone in config.drones]
    selected = args.drones.split(",") if args.drones else known
    selected = [drone_id.strip() for drone_id in selected if drone_id.strip()]
    unknown = [drone_id for drone_id in selected if drone_id not in known]
    if not selected or unknown or len(set(selected)) != len(selected):
        print(f"--drones must be distinct IDs from {known}; got {selected}", file=sys.stderr)
        return 2
    protocols = {drone.drone_id: drone.protocol.value for drone in config.drones}
    modes = sorted({drone.mode.value for drone in config.drones if drone.drone_id in selected})

    label = args.label or scenario_label(selected, protocols)
    run_dir = make_run_dir(args.output_root, label)
    selected_config = run_dir / "drones.selected.json"
    selected_config.write_text(
        json.dumps(
            {**raw_config, "drones": [d for d in raw_config["drones"] if d["drone_id"] in selected]},
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    event_log_path = Path(os.environ.get("MIDDLEWARE_EVENT_LOG", DEFAULT_EVENT_LOG))
    log_offset = event_log_path.stat().st_size if event_log_path.exists() else 0

    print(f"[evidence] {run_dir}")
    started_at = datetime.now(timezone.utc)
    started = time.monotonic()
    recorder_box: list[Recorder] = []
    outcome = {"mission_state": None, "error": None, "server_error": None}
    stderr_path = run_dir / "server.stderr.log"
    with stderr_path.open("w", encoding="utf-8") as server_stderr:
        try:
            outcome["mission_state"] = asyncio.run(
                run_flow(args, selected, selected_config, run_dir, started, recorder_box,
                         server_stderr)
            )
        except Exception as exc:  # keep whatever was collected
            outcome["error"] = describe_error(exc)
            print(f"[evidence] flow stopped: {outcome['error']}", file=sys.stderr)
    finished_at = datetime.now(timezone.utc)

    events_path = run_dir / "events.jsonl"
    copy_new_events(event_log_path, log_offset, events_path)
    events = read_events(events_path)
    if not registered_drones(events):
        # The server exited before registering drones; its traceback ends in stderr.
        outcome["server_error"] = last_line(stderr_path)
    calls = recorder_box[0].calls if recorder_box else []

    environment = {
        "started_at": started_at.isoformat(),
        "finished_at": finished_at.isoformat(),
        "duration_s": round((finished_at - started_at).total_seconds(), 1),
        "python": sys.version.split()[0],
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "wsl": "microsoft" in platform.release().lower(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "packages": package_versions(),
        "git": git_revision(),
        "drones_config": str(config_path),
        "drones_config_selected": str(selected_config),
        "scenarios_dir": os.environ.get("MIDDLEWARE_SCENARIOS_DIR", str(DEFAULT_SCENARIOS_DIR)),
        "event_log": str(event_log_path),
        "mode": "+".join(modes),
        "selected_drones": {drone_id: protocols[drone_id] for drone_id in selected},
        "mission_id": args.mission_id,
        "poll_interval_s": args.poll_interval,
        "timeout_s": args.timeout,
    }
    write_json(run_dir / "environment.json", environment)

    bindings = binding_rows(events)
    write_json(run_dir / "bindings.json", bindings)
    write_csv(run_dir / "tool_calls.csv", [call_row(call) for call in calls])
    write_csv(run_dir / "command_transitions.csv", transition_rows(events))
    summaries = command_summaries(events)
    write_csv(run_dir / "command_summary.csv", summaries)
    gaps = connection_gaps(calls_as_observations(calls), selected)
    write_json(run_dir / "connection_gaps.json", gaps)
    isolation = check_identifier_isolation(events, selected_drones=selected)
    write_json(run_dir / "identifier_check.json", isolation_dict(isolation))

    summary = render_summary(
        label=label,
        outcome=outcome,
        environment=environment,
        bindings=bindings,
        calls=calls,
        summaries=summaries,
        gaps=gaps,
        isolation=isolation,
        run_dir=run_dir,
    )
    (run_dir / "summary.md").write_text(summary, encoding="utf-8")

    succeeded = outcome["mission_state"] == "succeeded" and isolation.verdict == "PASS"
    print(
        f"[evidence] mission={outcome['mission_state']} "
        f"identifier_check={isolation.verdict} -> {run_dir / 'summary.md'}"
    )
    return 0 if succeeded else 1


async def run_flow(
    args: argparse.Namespace,
    selected: list[str],
    selected_config: Path,
    run_dir: Path,
    started: float,
    recorder_box: list[Recorder],
    server_stderr: Any,
) -> str | None:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        filter(None, [str(MIDDLEWARE_ROOT), env.get("PYTHONPATH")])
    )
    env["MIDDLEWARE_DRONES_CONFIG"] = str(selected_config)
    server = StdioServerParameters(
        command=sys.executable, args=["-m", "app.mcp_server"], env=env
    )
    async with stdio_client(server, errlog=server_stderr) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            recorder = Recorder(session, run_dir, started)
            recorder_box.append(recorder)
            mission_id = args.mission_id

            context = await recorder.call(
                "get_mission_context", {"mission_id": mission_id}, purpose="context"
            )
            for drone in context["drones"]:
                print(
                    f"[1] {drone['drone_id']} {drone['protocol']:7s} "
                    f"{drone['status']}/{drone['connection_status']}"
                )
            areas = [area["area_id"] for area in context["search_areas"]]
            if len(areas) < len(selected):
                raise RuntimeError(f"{len(selected)} drones but only {len(areas)} areas")

            rejected = await recorder.call(
                "submit_mission_plan",
                plan(mission_id, [(selected[0], INVALID_AREA_ID)]),
                purpose="invalid plan",
            )
            print(f"[2] invalid plan -> {rejected['status']} "
                  f"{[error['code'] for error in rejected.get('errors', [])]}")

            accepted = await recorder.call(
                "submit_mission_plan",
                plan(mission_id, list(zip(selected, areas))),
                purpose="valid plan",
            )
            print(f"[3] valid plan -> {accepted['status']} (attempt {accepted['attempt']})")
            if accepted["status"] != "accepted":
                print(f"    errors: {accepted.get('errors')}")

            deadline = time.monotonic() + args.timeout
            while True:
                status = await recorder.call(
                    "get_mission_status", {"mission_id": mission_id}, purpose="status"
                )
                print(f"[4] {time.monotonic() - started:6.1f}s {status['state']:9s} "
                      + ", ".join(progress(status, drone_id) for drone_id in selected))
                if status["state"] not in ("running",):
                    return status["state"]
                if time.monotonic() > deadline:
                    print("    timed out waiting for the mission", file=sys.stderr)
                    return "timeout"
                await asyncio.sleep(args.poll_interval)


def plan(mission_id: str, assignments: list[tuple[str, str]]) -> dict:
    return {
        "mission_id": mission_id,
        "assignments": [
            {"drone_id": drone_id, "area_id": area_id} for drone_id, area_id in assignments
        ],
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }


def progress(status: dict, drone_id: str) -> str:
    drone = next((d for d in status["drones"] if d["drone_id"] == drone_id), {})
    tasks = [task for task in status["tasks"] if task["drone_id"] == drone_id]
    done = total = 0
    if tasks:
        commands = tasks[-1]["commands"]
        done = sum(command["status"] == "succeeded" for command in commands)
        total = len(commands)
    return f"{drone_id} {done}/{total} {drone.get('connection_status', '?')}"


def scenario_label(selected: list[str], protocols: dict[str, str]) -> str:
    if len(selected) == 1:
        return f"single-{selected[0]}-{protocols[selected[0]]}"
    return "multi-" + "-".join(selected)


def make_run_dir(root: Path, label: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    root.mkdir(parents=True, exist_ok=True)
    for suffix in [""] + [f"-{n}" for n in range(2, 100)]:
        run_dir = root / f"{stamp}_{label}{suffix}"
        try:
            run_dir.mkdir()
            return run_dir
        except FileExistsError:
            continue
    raise RuntimeError(f"could not create a run directory under {root}")


def describe_error(exc: BaseException) -> str:
    """Name the underlying error; anyio wraps failures in (nested) exception groups."""
    while len(getattr(exc, "exceptions", ())) == 1:
        exc = exc.exceptions[0]
    inner = getattr(exc, "exceptions", None)
    if inner:
        return "; ".join(describe_error(item) for item in inner)
    return f"{type(exc).__name__}: {exc}"


def last_line(path: Path) -> str | None:
    lines = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    return lines[-1].strip() if lines else None


def copy_new_events(source: Path, offset: int, target: Path) -> None:
    """Copy only the lines this run appended to the (shared) event log."""
    data = b""
    if source.exists():
        with source.open("rb") as file:
            file.seek(offset)
            data = file.read()
    target.write_bytes(data)


def calls_as_observations(calls: list[dict]) -> list[dict]:
    return [
        {"at": call["at"], "elapsed_s": call["elapsed_s"], "response": call["response"]}
        for call in calls
        if not call["is_error"]
    ]


def binding_rows(events: list[dict]) -> list[dict]:
    rows = []
    for drone in registered_drones(events).values():
        binding = drone.get("binding") or {}
        rows.append(
            {
                "drone_id": drone["drone_id"],
                "protocol": drone.get("protocol"),
                "mode": drone.get("mode"),
                "connection_string": drone.get("connection_string"),
                "system_id": binding.get("system_id"),
                "component_id": binding.get("component_id"),
                "namespace": binding.get("namespace"),
                "connected": drone.get("connected"),
                "connect_error": drone.get("connect_error"),
            }
        )
    return rows


def call_row(call: dict) -> dict:
    response = call["response"] if isinstance(call["response"], dict) else {}
    return {
        "seq": call["seq"],
        "tool": call["tool"],
        "purpose": call["purpose"],
        "at": call["at"],
        "elapsed_s": call["elapsed_s"],
        "duration_ms": call["duration_ms"],
        "is_error": call["is_error"],
        "result": response.get("status") or response.get("state") or response.get("error"),
    }


def isolation_dict(check: IsolationCheck) -> dict:
    return {
        "verdict": check.verdict,
        "sent_commands": check.sent_commands,
        "drones": [vars(drone) for drone in check.drones],
        "failures": check.failures,
    }


def package_versions() -> dict[str, str | None]:
    versions = {}
    for name in PACKAGES:
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def git_revision() -> dict[str, Any] | None:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT,
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        branch = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=REPO_ROOT,
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"], cwd=REPO_ROOT,
            capture_output=True, text=True, check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    return {"commit": commit, "branch": branch, "uncommitted_changes": bool(dirty)}


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as file:
        if not rows:
            return
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------- summary


def render_summary(
    *,
    label: str,
    outcome: dict,
    environment: dict,
    bindings: list[dict],
    calls: list[dict],
    summaries: list[dict],
    gaps: list[dict],
    isolation: IsolationCheck,
    run_dir: Path,
) -> str:
    git = environment["git"] or {}
    git_text = (
        f"{git.get('commit')} ({git.get('branch')}"
        f"{', 커밋 안 된 변경 있음' if git.get('uncommitted_changes') else ''})"
        if git else "확인 불가"
    )
    selected = ", ".join(
        f"{drone_id} ({protocol})" for drone_id, protocol in environment["selected_drones"].items()
    )
    lines = [
        f"# 통합 검증 결과: {label}",
        "",
        "| 항목 | 값 |",
        "|---|---|",
        f"| 임무 결과 | {outcome['mission_state'] or '확인 못 함'} |",
        f"| 식별자 혼선 검사 | {isolation.verdict} |",
        f"| disconnected 보고 구간 | {len(gaps)}개 |",
        f"| 실행 시각 (UTC) | {environment['started_at']} ~ {environment['finished_at']} "
        f"({environment['duration_s']}초) |",
        f"| 모드 | {environment['mode']} |",
        f"| 선택한 드론 | {selected} |",
        f"| Python | {environment['python']} (`{environment['python_executable']}`) |",
        f"| OS | {environment['platform']}{' (WSL)' if environment['wsl'] else ''} |",
        f"| ROS_DISTRO | {environment['ros_distro'] or '-'} |",
        f"| 드론 구성 파일 | `{environment['drones_config']}` → 선택 드론만 `drones.selected.json` |",
        f"| 시나리오 디렉터리 | `{environment['scenarios_dir']}` |",
        f"| 코드 버전 | {git_text} |",
    ]
    if outcome["error"]:
        lines.append(f"| 중단 사유 | {outcome['error']} |")
    if outcome["server_error"]:
        lines.append(f"| 서버 오류 (stderr 마지막 줄) | `{outcome['server_error']}` |")

    lines += ["", "## 드론 등록과 binding (`server_started`)", ""]
    if bindings:
        lines += [
            "| drone_id | protocol | mode | 연결 문자열 | SYSID/COMPID | namespace | 연결 | 연결 오류 |",
            "|---|---|---|---|---|---|---|---|",
        ]
        for row in bindings:
            sysid = (
                f"{row['system_id']}/{row['component_id']}"
                if row["system_id"] is not None else "-"
            )
            lines.append(
                f"| {row['drone_id']} | {row['protocol']} | {row['mode']} "
                f"| {row['connection_string'] or '-'} | {sysid} "
                f"| {('/' + row['namespace']) if row['namespace'] else '-'} "
                f"| {'성공' if row['connected'] else '실패'} | {row['connect_error'] or '-'} |"
            )
    else:
        lines.append("`server_started` 이벤트가 없습니다. `server.stderr.log`를 확인하세요.")

    lines += ["", "## 도구 호출 흐름", "", "| # | 도구 | 목적 | 소요(ms) | 결과 |",
              "|---|---|---|---|---|"]
    status_calls = [call for call in calls if call["purpose"] == "status"]
    for call in calls:
        if call["purpose"] == "status":
            continue
        lines.append(
            f"| {call['seq']} | {call['tool']} | {call['purpose']} "
            f"| {call['duration_ms']:.1f} | {describe_response(call)} |"
        )
    if status_calls:
        durations = [call["duration_ms"] for call in status_calls]
        lines.append(
            f"| {status_calls[0]['seq']}~{status_calls[-1]['seq']} | get_mission_status "
            f"| status × {len(status_calls)} "
            f"| 평균 {sum(durations) / len(durations):.1f} / 최대 {max(durations):.1f} "
            f"| 최종 {describe_response(status_calls[-1])} |"
        )

    lines += ["", "## 명령별 결과", ""]
    if summaries:
        lines += [
            "| command_id | drone | 종류 | 상태 전이 | sent 후 경과(s) | 오류 |",
            "|---|---|---|---|---|---|",
        ]
        for row in summaries:
            elapsed = "-" if row["elapsed_s"] is None else f"{row['elapsed_s']:.2f}"
            lines.append(
                f"| {row['command_id']} | {row['drone_id']} | {row['command_type']} "
                f"| {row['transitions']} | {elapsed} | {row['error_code'] or '-'} |"
            )
    else:
        lines.append("실행된 명령이 없습니다.")

    lines += ["", "## disconnected 보고 구간", ""]
    if gaps:
        lines += [
            "| drone | 시작 (UTC) | 끝 (UTC) | 경과 기준(s) | 관측 수 | 명령 단계 | 회복 |",
            "|---|---|---|---|---|---|---|",
        ]
        for gap in gaps:
            lines.append(
                f"| {gap['drone_id']} | {gap['start_at']} | {gap['end_at']} "
                f"| {gap['start_elapsed_s']:.1f}~{gap['end_elapsed_s']:.1f} "
                f"| {gap['observations']} | {'<br>'.join(gap['stages'])} "
                f"| {'예' if gap['recovered'] else '아니오'} |"
            )
    else:
        lines.append("상태 조회 중 `disconnected`로 보고된 적이 없습니다.")
    lines.append(
        f"\n상태 조회 간격({environment['poll_interval_s']}초)마다 관측한 값이라 그보다 짧은 "
        "구간은 빠질 수 있습니다. 시각은 도구 호출 시작 시각입니다."
    )

    lines += [
        "",
        f"## 식별자 혼선 검사: {isolation.verdict}",
        "",
        "각 명령이 `server_started`에 등록된 drone_id, protocol, binding의 클라이언트로만 "
        "전송되었는지 EventLog(`plan_accepted`, `command_transition`의 `target`)로 확인합니다.",
        "",
        "| drone_id | protocol | 등록 binding | 전송된 명령 | 불일치 |",
        "|---|---|---|---|---|",
    ]
    for drone in isolation.drones:
        lines.append(
            f"| {drone.drone_id} | {drone.protocol} | {drone.binding} "
            f"| {drone.sent_commands} | {drone.mismatches} |"
        )
    if isolation.failures:
        lines += ["", "실패 항목:", ""] + [f"- {failure}" for failure in isolation.failures]

    lines += [
        "",
        "## 파일",
        "",
        "| 파일 | 내용 |",
        "|---|---|",
        "| `tool_calls/NNN_<tool>.json` | 도구 호출별 요청, 응답, 소요 시간 |",
        "| `tool_calls.csv` | 도구 호출 목록 |",
        "| `command_transitions.csv` | 명령 상태 전이 전체 |",
        "| `command_summary.csv` | 명령별 전이 경로와 경과 시간 |",
        "| `events.jsonl` | 이번 실행의 EventLog 사본 |",
        "| `bindings.json` | 드론 등록과 binding |",
        "| `connection_gaps.json` | disconnected 보고 구간 |",
        "| `identifier_check.json` | 식별자 혼선 검사 결과 |",
        "| `environment.json` | 실행 환경 |",
        "| `drones.selected.json` | 서버에 넘긴 드론 구성 (선택 드론만) |",
        "| `server.stderr.log` | 서버와 어댑터 출력 |",
        "",
    ]
    return "\n".join(lines)


def describe_response(call: dict) -> str:
    response = call["response"]
    if call["is_error"]:
        return f"도구 오류: {response.get('error', '')[:120]}"
    if call["tool"] == "get_mission_context":
        drones = ", ".join(
            f"{d['drone_id']} {d['status']}/{d['connection_status']}" for d in response["drones"]
        )
        return f"{drones}; 구역 {len(response['search_areas'])}개"
    if call["tool"] == "submit_mission_plan":
        if response["status"] == "rejected":
            codes = ", ".join(error["code"] for error in response.get("errors", []))
            return f"rejected ({codes}), attempt {response['attempt']}"
        tasks = ", ".join(
            f"{task['drone_id']}→{task['area_id']} ({task['command_count']}개 명령)"
            for task in response.get("tasks", [])
        )
        return f"accepted, attempt {response['attempt']}: {tasks}"
    return str(response.get("state"))


if __name__ == "__main__":
    sys.exit(main())
