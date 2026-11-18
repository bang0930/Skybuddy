"""자연어 명령, 정상 운용, 장애 시나리오 3개가 제대로 됐는지 미들웨어 이벤트 기록으로 판정한다 (기체에 명령은 안 보낸다).

  python3 mission_check.py --report                      정상 운용·장애 시나리오 3개가 지금 설정으로 검증됐는지 한눈에
  python3 mission_check.py --follow --since <epoch 초>   그 뒤에 들어온 계획을 끝날 때까지 따라가며 보여 주고 판정
                                                          (run_live.sh say·demo 가 명령을 보낸 뒤 이것을 부른다)
  python3 mission_check.py                               가장 최근 비행 기록(~/skybuddy/logs/live_*)의 마지막 명령을 판정
  python3 mission_check.py ~/skybuddy/logs/live_20261117_134357
  python3 mission_check.py ~/skybuddy/logs/live_20261117_190000 --scenario battery   그 기록의 시나리오 다시 판정
  python3 mission_check.py --selftest

자연어 명령(say) 통과 조건 (모두 맞아야 '판정: 통과', 종료 코드 0)
  1) 계획이 수락됐다 (plan_accepted). 거부(plan_rejected)되면 이유를 보여 준다. LLM 이 고쳐 다시 내서 수락되면 통과로 본다
  2) 수락된 작업마다 이륙·이동·착륙 명령이 전부 succeeded
  3) 작업이 succeeded 로 끝났다 (task_finished)
따라가는 중 Ctrl+C 로 멈춰도 비행은 계속된다 (나중에 인자 없이 다시 돌리면 판정만 한다).

정상 운용과 장애 시나리오 3개 (11/9 팀 회의, run_live.sh demo). 모두 같은 임무(mission-101)·같은 섹터(남·북)·같은 첫 명령
("남쪽과 북쪽 섹터를 드론 두 대로 하나씩 맡아 수색해 줘. 어느 드론이 어느 섹터를 맡을지는 드론 상태를 보고 정해 줘")
  normal   0. 정상 운용(장애 없음, 기준): 배정 규칙대로(배터리가 같으면 drone-01 가까운 남, drone-02 먼 북) 두 드론이
              각자 섹터를 끝까지 수색하고 홈에 착륙. 통신 두절·미도달·비상 정지 없음
  link     1. 통신 두절:먼 섹터(북)로 간 드론의 링크가 거리에 따라 나빠지다(250m) 끊김(330m) → 기체 귀환(RTL) →
              미들웨어가 두절 감지(LINK_LOST) → 복구되면 귀환·착륙. 가까운 섹터 드론은 성공
  battery  2. 배터리 부족: drone-01 배터리 40% → 우선순위 배정(부족한 드론에 가까운 섹터) → 비행 중 20% 에서 기체 귀환 →
              미들웨어는 목표에 못 가는 것으로 보고 timeout 처럼 처리(WAYPOINT_TIMEOUT → 귀환 → 홈 착륙) →
              못 끝낸 섹터를 완충 드론에 재배정 → 성공
  timeout  3. 웨이포인트 미도달: drone-02 가 웨이포인트에 60초 안에 못 감(WAYPOINT_TIMEOUT) → 귀환 → 홈 착륙 →
              못 끝낸 섹터를 다른 드론에 재배정 → 성공
장애 드론의 작업 실패는 정상이다. 장애가 실제로 걸렸는지 → 미들웨어가 알아채 멈췄는지 → 장애 드론이 홈으로 돌아와
내렸는지 → 다른 드론은 성공했는지 → (2·3) 재배정이 성공했는지를 본다.

--follow 로 판정한 run_live 명령은 그 기록 폴더의 demos.jsonl 에 판정 구간이 남는다. --report 는 live_* 기록을 모두
다시 판정해, 시나리오마다 '지금 설정'(기록 폴더에 남은 속도·경로 설정이 지금 파일과 같음)으로 통과했는지 보여 준다.
정상 운용은 두 드론이 다 성공해 unavailable 이 없으므로, 끝난 뒤 다시 켜지 않고 장애 시나리오 하나를 이어서 할 수 있다.
"""
import argparse
import glob
import json
import math
import os
import sys
import tempfile
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = HERE if os.path.exists(os.path.join(HERE, "kpi_mission.py")) else os.path.dirname(HERE)
LOG_ROOT = os.path.expanduser("~/skybuddy/logs")
RETRY_WINDOW_S = 180.0     # 판정만 할 때: 마지막 수락 앞 이만큼 안의 제출·거부는 같은 명령의 재시도로 본다
PLAN_WAIT_S = 30.0         # 따라갈 때: 계획 이벤트를 기다리는 시간 (LLM 답이 온 뒤라 보통 이미 들어와 있다)
M_PER_DEG = 111320.0       # 미들웨어 coverage.py 와 같은 값
TYPE_KO = {"takeoff": "이륙", "goto": "이동", "land": "착륙", "return_home": "복귀"}

# 정상 운용 + 장애 시나리오 3개: (제목, 기대). run_live.sh demo 와 같아야 한다. 현황(--report)은 이 순서로 보여 준다.
SCENARIOS = {
    "normal": ("0. 정상 운용",
               "장애 없이 배정 규칙대로(배터리 같으면 drone-01 가까운 섹터) 두 드론이 각자 섹터를 끝까지 수색 → 홈 착륙"),
    "link": ("1. 통신 두절",
             "먼 섹터 드론의 링크가 거리에 따라 악화 → 두절 → 기체 귀환 → 복구 → 귀환·착륙, 가까운 섹터 드론은 성공"),
    "battery": ("2. 배터리 부족",
                "배터리 부족 드론에 가까운 섹터 → 비행 중 부족해 귀환(timeout 처럼 처리) → 홈 착륙 → 완충 드론에 재배정 → 성공"),
    "timeout": ("3. 웨이포인트 미도달",
                "웨이포인트에 시간 안에 못 감 → 귀환 → 홈 착륙 → 못 끝낸 섹터를 다른 드론에 재배정 → 성공"),
}
STAGES = {"battery-1": "battery", "timeout-1": "timeout"}   # 재배정 명령 전 1단계 (demos.jsonl·현황에 안 남긴다)
DEMO_COMMANDS = {name: f"bash {{d}}/run_live.sh demo {name}" for name in SCENARIOS}
LOW_BATTERY_DRONE = "drone-01"       # demo battery 가 배터리를 낮추는 드론
STALL_DRONE = "drone-02"             # demo timeout 이 웨이포인트를 못 가게 하는 드론
TIE_ORDER = ("drone-01", "drone-02")  # 배터리가 비슷할 때 가까운 섹터부터 받는 순서 (mission-101 설명의 규칙)
# 장애가 걸렸다는 기록 (정상 운용에 있으면 안 된다). fault_link_state 는 degraded·down 으로 바뀔 때만 센다
FAULT_EFFECTS = ("fault_battery_low", "fault_battery_empty", "fault_goto_dropped", "fault_failsafe",
                 "fault_takeoff_blocked")
SETTING_FILES = ("mavlink_drone.parm", "dds_drone.parm", "drones.sitl.json")


def num(x):
    return f"{x:g}" if isinstance(x, (int, float)) and not isinstance(x, bool) else "?"


def parse_time(text):
    try:
        return datetime.fromisoformat(str(text).replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def event_time(ev):
    return parse_time(ev.get("logged_at") or ev.get("updated_at") or "")


def hms(t):
    return datetime.fromtimestamp(t).strftime("%H:%M:%S") if t else "--:--:--"


def stamp(t):
    return datetime.fromtimestamp(t).strftime("%m/%d %H:%M") if t else "--/-- --:--"


def duration(seconds):
    seconds = int(round(seconds or 0))
    return f"{seconds // 60}분 {seconds % 60}초" if seconds >= 60 else f"{seconds}초"


def dist_m(a, b):
    """두 (위도, 경도) 사이 수평 거리 m (수백 m 안이라 평면 근사)."""
    lat = math.radians((a[0] + b[0]) / 2.0)
    return math.hypot((b[0] - a[0]) * M_PER_DEG, (b[1] - a[1]) * M_PER_DEG * math.cos(lat))


def area_names(mission_id):
    """{area_id: 북·남·동·서} (2x2 임무일 때. 미니맵과 같은 이름). 못 구하면 {}."""
    try:
        sys.path.insert(0, HERE)
        import map_probe as mp
        for folder in (os.environ.get("MIDDLEWARE_SCENARIOS_DIR"), os.path.join(BASE, "scenarios", "missions")):
            if not folder:
                continue
            path = os.path.join(os.path.expanduser(folder), f"{mission_id}.json")
            if os.path.isfile(path):
                with open(path, encoding="utf-8") as f:
                    mission = json.load(f)
                origin = mp.load_settings()[0]
                return mp.compass_names(mp.area_centers(mission, origin[0], origin[1]))
    except Exception:  # noqa: BLE001 - 이름은 보기 좋으라고 붙이는 것이라 없어도 된다
        pass
    return {}


def area_reach_m(task):
    """홈(귀환 goto 목표)에서 섹터 수색 웨이포인트 중심까지 수평 거리 m (모르면 None)."""
    if not task:
        return None
    home, points = task.get("home"), task.get("points") or []
    if not home or None in home or not points:
        return None
    lat = sum(p[0] for p in points) / len(points)
    lon = sum(p[1] for p in points) / len(points)
    return dist_m(home, (lat, lon))


class Run:
    """since~until 사이에 들어온 계획·작업의 진행 상태. feed() 에 이벤트를 차례로 넣는다."""

    def __init__(self, since=None, names=None, until=None):
        self.since = since
        self.until = until
        self.names = names            # {area_id: 이름}. None 이면 임무 ID 를 보고 처음 한 번 구한다
        self.submitted = 0
        self.rejections = []          # [(시도, [code: message])]
        self.accepted = []            # [시도]
        self.tasks = {}               # task_id -> 작업 상태 dict
        self.server_stopped = False
        self.emergency = []           # [(drone, landed)]
        self.link = {}                # drone -> {"lost", "restored", "recovery", "return_leg"}
        self.returned = {}            # drone -> (귀환 구간 결과, 착륙 결과): 미도달 뒤 귀환 (timeout_return_finished)

    def task_of(self, drone):
        """그 드론의 마지막 작업 (없으면 None)."""
        found = [task for task in self.tasks.values() if task["drone"] == drone]
        return found[-1] if found else None

    def first_tasks(self):
        """처음 수락된 계획의 작업들."""
        starts = [task["start"] for task in self.tasks.values() if task["start"] is not None]
        if not starts:
            return list(self.tasks.values())
        first = min(starts)
        return [task for task in self.tasks.values() if task["start"] == first]

    def first_task_of(self, drone):
        return next((task for task in self.first_tasks() if task["drone"] == drone), None)

    @staticmethod
    def root_failure(task):
        """작업을 멈춘 첫 실패 (무엇, 상태, code, message). 뒤따라 취소된 명령(TASK_ABORTED)은 건너뛴다."""
        fails = list((task or {}).get("failed", {}).values())
        return next((f for f in fails if f[2] != "TASK_ABORTED"), fails[0] if fails else None)

    def landed(self, drone):
        """그 드론이 내려앉았다는 미들웨어 기록 (설명 글) 또는 None."""
        if (self.returned.get(drone) or (None, None))[1] == "landed":
            return "귀환 후 착륙"
        if (self.link.get(drone) or {}).get("recovery") in ("landed", "already_on_ground"):
            return f"두절 처리 끝: {self.link[drone]['recovery']}"
        if any(d == drone and flag is True for d, flag in self.emergency):
            return "비상 정지 뒤 landed=true (그 자리 착륙)"
        task = self.task_of(drone)
        if task and any(task["kinds"].get(cid) == "land" for cid in task["done"]):
            return "착륙 명령 성공"
        return None

    def area_label(self, area_id):
        name = (self.names or {}).get(area_id)
        return f"{name}({area_id})" if name else str(area_id)

    def pairs_text(self, assignments):
        return ", ".join(f"{a.get('drone_id')} → {self.area_label(a.get('area_id'))}" for a in assignments) or "배정 없음"

    def feed(self, ev):
        """이벤트 하나를 반영하고, 사람이 읽을 진행 줄 [(시각, 글)] 을 돌려준다."""
        t = event_time(ev)
        if self.since is not None and (t is None or t < self.since):
            return []
        if self.until is not None and t is not None and t > self.until:
            return []
        kind = ev.get("event")
        if self.names is None and ev.get("mission_id") and kind in ("plan_submitted", "plan_accepted"):
            self.names = area_names(ev["mission_id"])
        if kind == "plan_submitted":
            self.submitted += 1
            plan = ev.get("plan") or {}
            return [(t, f"계획 제출 (시도 {ev.get('attempt')}): {self.pairs_text(plan.get('assignments') or [])}")]
        if kind == "plan_rejected":
            errors = [f"{e.get('code')}: {e.get('message')}" for e in ev.get("errors") or []]
            self.rejections.append((ev.get("attempt"), errors))
            return [(t, f"계획 거부 (시도 {ev.get('attempt')}): {'; '.join(errors)[:300]}")]
        if kind == "plan_accepted":
            self.accepted.append(ev.get("attempt"))
            for task in ev.get("tasks") or []:
                group = next((g for g in ev.get("commands") or [] if g and g[0].get("task_id") == task.get("task_id")), [])
                cmds = [(c.get("command_id"), c.get("payload") or {}) for c in group]
                kinds = {cid: p.get("command_type") for cid, p in cmds}
                gotos = [cid for cid, k in kinds.items() if k == "goto"]
                points = {cid: ((p.get("target") or {}).get("latitude"), (p.get("target") or {}).get("longitude"))
                          for cid, p in cmds if p.get("command_type") == "goto"}
                land_at = next((i for i, (_, p) in enumerate(cmds) if p.get("command_type") == "land"), None)
                home_cid = cmds[land_at - 1][0] if land_at and kinds.get(cmds[land_at - 1][0]) == "goto" else None
                self.tasks[task.get("task_id")] = {
                    "drone": task.get("drone_id"), "area": task.get("area_id"), "kinds": kinds,
                    "goto_order": gotos, "done": {}, "failed": {}, "state": None, "start": t, "end": None,
                    "home": points.get(home_cid),
                    "points": [points[cid] for cid in gotos if cid != home_cid and None not in points[cid]],
                    "altitude": next((p.get("altitude_m") for _, p in cmds if p.get("command_type") == "takeoff"), None)}
            return [(t, f"계획 수락 (시도 {ev.get('attempt')}): "
                        + self.pairs_text([{"drone_id": x.get("drone_id"), "area_id": x.get("area_id")}
                                           for x in ev.get("tasks") or []]))]
        if kind == "command_transition":
            task = self.tasks.get(ev.get("task_id"))
            if task is None:
                return []
            cid, status = ev.get("command_id"), ev.get("status")
            ctype = ev.get("command_type") or task["kinds"].get(cid)
            what = TYPE_KO.get(ctype, ctype)
            if ctype == "goto" and cid in task["goto_order"]:
                what = f"이동 {task['goto_order'].index(cid) + 1}/{len(task['goto_order'])}"
            took = ev.get("elapsed_since_sent_s")
            if status == "succeeded":
                task["done"][cid] = took
                return [(t, f"{task['drone']} {what} 완료" + (f" ({float(took):.1f}초)" if took is not None else ""))]
            if status in ("failed", "timed_out", "cancelled", "rejected"):
                err = ev.get("error") or {}
                task["failed"][cid] = (what, status, err.get("code"), err.get("message"))
                return [(t, f"{task['drone']} {what} {status}: {err.get('code')} {err.get('message') or ''}".rstrip())]
            return []
        if kind == "task_finished":
            task = self.tasks.get(ev.get("task_id"))
            if task is None:
                return []
            task["state"], task["end"] = ev.get("state"), t
            return [(t, f"{task['drone']} 작업 끝: {ev.get('state')} ({duration((t or 0) - (task['start'] or t or 0))})")]
        if kind == "emergency_stop":
            self.emergency.append((ev.get("drone_id"), ev.get("landed")))
            return [(t, f"{ev.get('drone_id')} 비상 정지 (landed={ev.get('landed')})")]
        if kind == "timeout_return_finished":
            drone = ev.get("drone_id")
            self.returned[drone] = (ev.get("return_leg"), ev.get("outcome"))
            return [(t, f"{drone} 미도달 뒤 귀환 끝: 귀환 {ev.get('return_leg')}, {ev.get('outcome')}")]
        if kind in ("link_lost", "link_restored", "link_not_restored", "link_recovery_finished"):
            drone = ev.get("drone_id")
            seen = self.link.setdefault(drone, {"lost": 0, "restored": [], "recovery": None, "return_leg": None})
            if kind == "link_lost":
                seen["lost"] += 1
                return [(t, f"{drone} 통신 두절 감지 (복구 기다림 {num(ev.get('wait_s'))}초)")]
            if kind == "link_restored":
                seen["restored"].append(ev.get("outage_s"))
                return [(t, f"{drone} 통신 복구 ({num(ev.get('outage_s'))}초 두절)")]
            if kind == "link_not_restored":
                return [(t, f"{drone} 통신 복구 안 됨")]
            seen["recovery"], seen["return_leg"] = ev.get("outcome"), ev.get("return_leg")
            return [(t, f"{drone} 두절 처리 끝: {ev.get('outcome')}"
                        + (f", 귀환 {ev.get('return_leg')}" if ev.get("return_leg") else ""))]
        if kind == "area_split":
            return [(t, f"구역 나눔: {ev.get('base_area_id')} → {', '.join(ev.get('area_ids') or [])}")]
        if kind == "server_stopping":
            self.server_stopped = True
            return [(t, "미들웨어 서버가 꺼졌다")]
        return []

    def finished(self):
        """판정할 수 있는 상태인가: 수락된 작업이 다 끝났거나 서버가 꺼졌다."""
        if self.server_stopped:
            return True
        return bool(self.tasks) and all(task["state"] is not None for task in self.tasks.values())

    def verdict(self):
        """(통과 여부, 요약 줄들)."""
        lines, problems = [], []
        if not self.accepted:
            if self.rejections:
                attempt, errors = self.rejections[-1]
                problems.append(f"계획이 수락되지 않았다 (마지막 시도 {attempt} 거부: {'; '.join(errors)[:400]})")
            elif self.submitted:
                problems.append("계획을 냈지만 수락·거부 기록이 없다")
            else:
                problems.append("계획이 제출되지 않았다 (LLM 답을 볼 것: 임무를 되묻거나 도구 호출에 실패했을 수 있다)")
        for task in self.tasks.values():
            gotos = len(task["goto_order"])
            reached = sum(1 for cid in task["goto_order"] if cid in task["done"])
            took = duration((task["end"] or time.time()) - (task["start"] or time.time()))
            alt = f", {task['altitude']:g}m" if isinstance(task.get("altitude"), (int, float)) else ""
            lines.append(f"  {task['drone']} {self.area_label(task['area'])}{alt}: 이동 {reached}/{gotos}, "
                         f"{'끝 ' + str(task['state']) if task['state'] else '진행 중'}, {took}")
            for what, status, code, message in task["failed"].values():
                problems.append(f"{task['drone']} {what} {status} ({code}: {message})")
            if task["state"] is None:
                problems.append(f"{task['drone']} 작업이 아직 끝나지 않았다")
            elif task["state"] != "succeeded" and not task["failed"]:
                problems.append(f"{task['drone']} 작업이 {task['state']} 로 끝났다")
        for drone, landed in self.emergency:
            problems.append(f"{drone} 비상 정지 (landed={landed})")
        ok = not problems
        head = "판정: 통과" if ok else "판정: 실패"
        if ok:
            retry = f", 거부 {len(self.rejections)}번 뒤" if self.rejections else ""
            head += f" - 자연어 명령 → 계획 수락(시도 {self.accepted[-1]}{retry}) → 작업 {len(self.tasks)}개 모두 착륙까지 성공"
        return ok, [head] + lines + [f"  - {p}" for p in problems]


def read_events(path, offset=0):
    """(이벤트 목록, 새 offset). 줄이 덜 쓰였으면 다음에 다시 읽는다."""
    events = []
    try:
        with open(path, "rb") as f:
            f.seek(offset)
            data = f.read()
    except (OSError, TypeError):
        return events, offset
    end = data.rfind(b"\n") + 1
    for line in data[:end].splitlines():
        try:
            events.append(json.loads(line.decode("utf-8", errors="replace")))
        except json.JSONDecodeError:
            continue
    return events, offset + end


def in_window(events, since=None, until=None, margin=1.0):
    """since~until (epoch 초) 사이 기록만. 시각이 없으면 뺀다."""
    picked = []
    for ev in events:
        t = event_time(ev)
        if t is None:
            continue
        if since is not None and t < since - margin:
            continue
        if until is not None and t > until + margin:
            continue
        picked.append(ev)
    return picked


def newest_events(root=LOG_ROOT):
    files = glob.glob(os.path.join(root, "live_*", "mission-events.jsonl"))
    return max(files, key=os.path.getmtime) if files else None


def faults_path(events_path):
    """같은 기록 폴더의 장애 기록 (mission-events*.jsonl -> fault-events*.jsonl)."""
    folder, base = os.path.split(events_path)
    return os.path.join(folder, base.replace("mission-events", "fault-events", 1))


def last_command_since(events):
    """판정만 할 때: 마지막 명령(마지막 수락과 그 앞 재시도들)이 시작된 시각."""
    plan_events = [(event_time(e), e.get("event")) for e in events
                   if e.get("event") in ("plan_submitted", "plan_rejected", "plan_accepted")]
    plan_events = [(t, k) for t, k in plan_events if t is not None]
    if not plan_events:
        return None
    accepted = [t for t, k in plan_events if k == "plan_accepted"]
    anchor = accepted[-1] if accepted else plan_events[-1][0]
    start = anchor
    for t, _ in reversed(plan_events):
        if t <= anchor and anchor - t <= RETRY_WINDOW_S:
            start = min(start, t)
    return start - 0.001


def record_run(path, name, since, until, ok):
    """run_live 기록 폴더에 이번 명령의 판정 구간을 남긴다 (--report 가 다시 판정한다)."""
    folder = os.path.dirname(os.path.abspath(path))
    if not os.path.basename(folder).startswith("live_"):
        return
    try:
        with open(os.path.join(folder, "demos.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps({"scenario": name, "since": since, "until": until, "passed": bool(ok),
                                "logged_at": datetime.now().astimezone().isoformat()}, ensure_ascii=False) + "\n")
    except OSError:
        pass


def follow(path, since, timeout_s, out=print, scenario=None, judge_since=None):
    """path 를 since 부터 따라가며 진행을 보여 주고 판정한다. 통과면 True.

    scenario 를 주면 끝난 뒤 그 장애 시나리오 기대대로 판정한다 (judge_since 부터의 기록으로. 재배정처럼 명령을
    두 번 할 때 앞 명령까지 함께 본다). run_live 기록이면 판정 구간을 demos.jsonl 에 남긴다 (--report 용).
    """
    run = Run(since=since)
    offset, started, plan_seen_at = 0, time.time(), None
    interrupted = False
    out(f"임무 진행 확인: {path}")
    try:
        while True:
            events, offset = read_events(path, offset)
            for ev in events:
                for t, text in run.feed(ev):
                    out(f"  [{hms(t)}] {text}")
            if run.submitted or run.accepted or run.rejections:
                plan_seen_at = plan_seen_at or time.time()
            if run.finished():
                break
            waited = time.time() - started
            if not run.accepted and waited > PLAN_WAIT_S and (not run.rejections or
                                                               time.time() - (plan_seen_at or started) > PLAN_WAIT_S):
                break
            if waited > timeout_s:
                out(f"  {timeout_s:g}초가 지나 따라가기를 멈춘다 (비행은 계속된다)")
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        interrupted = True
        out("\n  따라가기를 멈췄다 (비행은 계속된다). 나중에 판정만: python3 mission_check.py")
    begin = judge_since if judge_since is not None else since
    if scenario:
        events, _ = read_events(path)
        judged = Run(since=begin)
        for ev in events:
            judged.feed(ev)
        faults, _ = read_events(faults_path(path))
        ok, lines = scenario_verdict(judged, in_window(faults, begin), scenario)
    else:
        ok, lines = run.verdict()
    for line in lines:
        out(line)
    if not interrupted and scenario not in STAGES:
        record_run(path, scenario or "say", begin, time.time(), ok)
    return ok


def check_file(path, since=None, out=print):
    events, _ = read_events(path)
    since = since if since is not None else last_command_since(events)
    run = Run(since=since)
    out(f"기록: {path}")
    for ev in events:
        for t, text in run.feed(ev):
            out(f"  [{hms(t)}] {text}")
    ok, lines = run.verdict()
    for line in lines:
        out(line)
    return ok


# ------------------------------------------------------------------ 장애 시나리오 판정

def _faults_of(faults, drone, event):
    return [e for e in faults if e.get("event") == event
            and (drone == e.get("drone") or drone in (e.get("identities") or []))]


def _moves(task):
    reached = sum(1 for cid in task["goto_order"] if cid in task["done"])
    return f"이동 {reached}/{len(task['goto_order'])}"


def _why(task):
    root = Run.root_failure(task)
    return f" ({root[0]} {root[1]} {root[2]})" if root else ""


def _where(run, task):
    if not task:
        return ""
    reach = area_reach_m(task)
    return f" {run.area_label(task['area'])}" + (f" 홈에서 {num(round(reach))}m" if reach is not None else "")


def _plan_check(run, add):
    if run.accepted:
        note = f", 거부 {len(run.rejections)}번 뒤" if run.rejections else ""
        pairs = ", ".join(f"{task['drone']} →{_where(run, task)}" for task in run.first_tasks())
        add(True, f"계획 수락 (시도 {run.accepted[0]}{note}): {pairs}")
    else:
        add(False, "계획이 수락되지 않았다"
            + (f" (거부: {'; '.join(run.rejections[-1][1])[:200]})" if run.rejections else ""))


def _others_ok(run, faulted, add):
    """장애를 안 건 드론의 첫 작업이 성공했는가."""
    for task in run.first_tasks():
        if task["drone"] in faulted:
            continue
        stops = [flag for d, flag in run.emergency if d == task["drone"]]
        ok = task["state"] == "succeeded" and not stops
        add(ok, f"{task['drone']} 장애 없음 →{_where(run, task)} 수색 성공 ({_moves(task)})" if ok
            else f"{task['drone']} 장애를 안 걸었는데 작업이 {task['state'] or '끝나지 않음'}{_why(task)}")


def _timed_out_check(run, drone, add, why):
    task = run.first_task_of(drone)
    root = Run.root_failure(task)
    ok = task is not None and task["state"] == "failed" and root is not None and root[2] == "WAYPOINT_TIMEOUT"
    add(ok, f"{drone} 미들웨어: {root[0]} 을 시간 안에 못 가 WAYPOINT_TIMEOUT ({why})" if ok
        else f"{drone} WAYPOINT_TIMEOUT 이 없다 (작업 {task['state'] if task else '없음'}{_why(task) if task else ''})")


def _returned_check(run, drone, add):
    """미도달 뒤 미들웨어가 귀환 구간을 보내 홈에 내렸는가 (timeout_return_finished)."""
    leg, outcome = run.returned.get(drone) or (None, None)
    if leg == "reached" and outcome == "landed":
        add(True, f"{drone} 홈으로 귀환 후 착륙 (미들웨어 귀환 구간 도착 → 착륙 확인)")
    elif leg or outcome:
        add(False, f"{drone} 귀환이 끝나지 않았다 (귀환 {leg}, {outcome})")
    else:
        landed = run.landed(drone)
        add(False, f"{drone} 귀환 기록이 없다" + (f" - {landed}" if landed else "")
            + " (미들웨어가 귀환 처리를 안 했다: 팀 저장소 dispatcher.py 가 이 PR 의 것인지 볼 것)")


def _normal_checks(run, faults, add):
    """정상 운용: 장애 없이 배정 규칙대로 두 드론이 각자 섹터를 끝까지 수색하고 홈에 착륙했는가."""
    injected = sorted({f"{e.get('kind')} 장애 시작" for e in faults if e.get("event") == "fault_started"}
                      | {f"통신 {e.get('state_to')}" for e in faults if e.get("event") == "fault_link_state"
                         and e.get("state_to") in ("degraded", "down")}
                      | {str(e.get("event")) for e in faults if e.get("event") in FAULT_EFFECTS})
    add(not injected, "장애 주입 없음" if not injected
        else f"장애가 걸려 있었다 ({', '.join(injected)}) - 정상 운용 시험이 아니다 (fault.py clear 뒤 다시)")
    tasks = run.first_tasks()
    if len(tasks) < 2:
        add(False, "두 드론에 섹터를 하나씩 주지 않았다 (남·북을 drone-01·drone-02 가 하나씩 맡을 것으로 기대)")
    else:
        ordered = sorted(tasks, key=lambda t: area_reach_m(t) if area_reach_m(t) is not None else math.inf)
        ok = tuple(t["drone"] for t in ordered[:2]) == TIE_ORDER
        detail = ", ".join(f"{t['drone']} →{_where(run, t)}" for t in ordered)
        add(ok, ("배정 규칙: 배터리가 같으면 drone-01 에 가까운 섹터 - " if ok
                 else "배정 규칙과 다르다 (배터리가 같으면 drone-01 에 가까운 섹터) - ") + detail)
    for task in tasks:
        landed = any(task["kinds"].get(cid) == "land" for cid in task["done"])
        ok = task["state"] == "succeeded" and not task["failed"] and landed
        add(ok, f"{task['drone']}{_where(run, task)} 수색 끝까지 ({_moves(task)}) → 홈 착륙" if ok
            else f"{task['drone']} 작업 {task['state'] or '안 끝남'}{_why(task)}")
    troubles = ([f"{d} 통신 두절" for d, seen in run.link.items() if seen.get("lost")]
                + [f"{d} 미도달 귀환" for d in run.returned]
                + [f"{d} 비상 정지" for d, _ in run.emergency])
    add(not troubles, "통신 두절·웨이포인트 미도달·비상 정지 없음" if not troubles
        else "정상 운용인데 " + ", ".join(troubles))


def _link_checks(run, faults, add):
    downs = sorted({e.get("drone") for e in faults
                    if e.get("event") == "fault_link_state" and e.get("state_to") == "down" and e.get("drone")})
    if not downs:
        add(False, "어느 드론도 통신이 끊기지 않았다 (먼 섹터에서 330m 를 넘지 않았다) - 장애 시험이 안 됐다")
    for drone in downs:
        task = run.first_task_of(drone)
        down = _faults_of(faults, drone, "fault_link_state")
        degraded = any(e.get("state_to") == "degraded" for e in down)
        cut = next(e for e in down if e.get("state_to") == "down")
        rtl = any(e.get("sent") for e in _faults_of(faults, drone, "fault_failsafe"))
        add(True, f"{drone}{_where(run, task)} 장애: " + ("링크 악화 → " if degraded else "")
            + (f"{num(round(cut['distance_m']))}m 에서 " if isinstance(cut.get("distance_m"), (int, float)) else "")
            + "두절" + (" → 기체 failsafe 귀환(RTL)" if rtl else " (기체 failsafe 기록 없음)"))
        seen = run.link.get(drone) or {}
        root = Run.root_failure(task)
        noticed = seen.get("lost") or (root is not None and root[2] == "LINK_LOST")
        add(noticed, f"{drone} 미들웨어가 두절을 알아챔 (LINK_LOST)" if noticed
            else f"{drone} 미들웨어가 두절을 알아채지 못했다 (link_lost 기록 없음)")
        restored = seen.get("restored") or []
        add(restored, f"{drone} 통신 복구 ({num(restored[0])}초 두절)" if restored
            else f"{drone} 통신이 복구되지 않았다 (link_restored 기록 없음)")
        home = seen.get("recovery") in ("landed", "already_on_ground")
        add(home, f"{drone} 홈으로 귀환 후 착륙 (두절 처리 끝: {seen.get('recovery')}"
            + (f", 귀환 {seen.get('return_leg')}" if seen.get("return_leg") else "") + ")" if home
            else f"{drone} 귀환·착륙 기록이 없다 (두절 처리 끝: {seen.get('recovery')})")
    _others_ok(run, set(downs), add)


def _battery_checks(run, faults, add):
    low = LOW_BATTERY_DRONE
    started = [e for e in _faults_of(faults, low, "fault_started") if e.get("kind") == "battery"]
    p = (started[-1].get("params") or {}) if started else {}
    ok = bool(started) and p.get("rtl_pct") is not None
    add(ok, f"{low} 장애: 배터리 {num(started[-1].get('start_pct', p.get('from_pct')))}% 로 시작, 비행 중 줄어 "
            f"{num(p.get('rtl_pct'))}% 이하면 귀환" if ok
        else f"{low} 배터리 부족 장애 기록이 없다 - 장애 시험이 안 됐다")
    mine = run.first_task_of(low)
    others = [task for task in run.first_tasks() if task["drone"] != low]
    if mine is None or not others:
        add(False, "두 드론에 섹터를 하나씩 주지 않았다 (배터리 부족 드론에 가까운 섹터를 줄 것으로 기대)")
    else:
        reach = area_reach_m(mine)
        nearer = reach is not None and all(area_reach_m(t) is not None and reach < area_reach_m(t) for t in others)
        detail = ", ".join(f"{task['drone']} →{_where(run, task)}" for task in [mine] + others)
        add(nearer, ("우선순위 배정: 배터리 적은 드론에 가까운 섹터 - " if nearer
                     else "배터리 적은 드론이 더 먼 섹터를 받았다 - ") + detail)
    sent = [e for e in _faults_of(faults, low, "fault_battery_low") if e.get("sent")]
    add(sent, f"{low} 배터리 {num(sent[0].get('pct'))}% → 기체 배터리 failsafe 귀환(RTL)"
              + (f" (홈에서 {num(round(sent[0]['home_distance_m']))}m)"
                 if isinstance(sent[0].get("home_distance_m"), (int, float)) else "") if sent
        else f"{low} 배터리 부족 귀환(RTL)이 일어나지 않았다 (비행 중 기준 이하로 안 내려갔다)")
    _timed_out_check(run, low, add, "timeout 처럼 처리")
    _returned_check(run, low, add)
    _others_ok(run, {low}, add)


def _timeout_checks(run, faults, add):
    stuck = STALL_DRONE
    task = run.first_task_of(stuck)
    dropped = [e for e in _faults_of(faults, stuck, "fault_goto_dropped") if e.get("reason") != "battery_empty"]
    add(dropped, f"{stuck}{_where(run, task)} 장애: goto {len(dropped)}번 무시 (웨이포인트로 안 감)" if dropped
        else f"{stuck} goto 무시 기록이 없다 - 장애 시험이 안 됐다")
    _timed_out_check(run, stuck, add, "일정 시간 안에 도달 못 함")
    _returned_check(run, stuck, add)
    _others_ok(run, {stuck}, add)


def _reassign_checks(run, add):
    """첫 명령에서 실패한 작업의 섹터를 다른 드론이 다음 명령으로 다시 맡아 성공했는가."""
    failed = [task for task in run.first_tasks() if task["state"] not in (None, "succeeded")]
    if not failed:
        add(False, "못 끝낸 섹터가 없다 (첫 명령에서 실패한 작업이 없음)")
        return
    for lost in failed:
        label = run.area_label(lost["area"])
        later = [task for task in run.tasks.values()
                 if task["area"] == lost["area"] and task["drone"] != lost["drone"]
                 and (task["start"] or 0) > (lost["start"] or 0)]
        if not later:
            add(False, f"{label} ({lost['drone']} 실패) 를 다른 드론에 다시 배정한 계획이 없다")
            continue
        task = later[-1]
        add(True, f"재배정 계획 수락: {label} ({lost['drone']} 실패) → {task['drone']}")
        ok = task["state"] == "succeeded"
        add(ok, f"{task['drone']} 재배정 수색 성공 ({_moves(task)}, 착륙까지)" if ok
            else f"{task['drone']} 재배정 작업 {task['state'] or '안 끝남'}{_why(task)}")


def judge(run, faults, name):
    """시험 하나의 검사 [(통과?, 글)]."""
    base = STAGES.get(name, name)
    checks = []

    def add(ok, text):
        checks.append((bool(ok), text))

    _plan_check(run, add)
    if base == "normal":
        _normal_checks(run, faults, add)
    elif base == "link":
        _link_checks(run, faults, add)
    elif base == "battery":
        _battery_checks(run, faults, add)
    elif base == "timeout":
        _timeout_checks(run, faults, add)
    if base in ("battery", "timeout") and name not in STAGES:
        _reassign_checks(run, add)
    return checks


def scenario_verdict(run, faults, name):
    """시험 판정 (통과 여부, 줄들)."""
    base = STAGES.get(name, name)
    checks = judge(run, faults, name)
    ok = bool(checks) and all(flag for flag, _ in checks)
    title, story = SCENARIOS[base]
    stage = " 1단계(재배정 전)" if name in STAGES else ""
    head = f"판정({title}{stage}): {'통과' if ok else '실패'} - 기대: {story}"
    return ok, [head] + [f"  [{'OK' if flag else '실패'}] {text}" for flag, text in checks]


def check_scenario(path, name, since=None, out=print):
    """run_live 기록에서 시나리오 하나를 다시 판정한다 (since 가 없으면 demos.jsonl 의 마지막 그 시나리오 구간)."""
    events, _ = read_events(path)
    faults, _ = read_events(faults_path(path))
    until = None
    if since is None:
        records = [r for r in read_events(os.path.join(os.path.dirname(path), "demos.jsonl"))[0]
                   if r.get("scenario") == name]
        if records:
            since, until = records[-1].get("since"), records[-1].get("until")
    run = Run(since=since, until=until)
    for ev in events:
        run.feed(ev)
    out(f"기록: {path}")
    ok, lines = scenario_verdict(run, in_window(faults, since, until), name)
    for line in lines:
        out(line)
    return ok


# ------------------------------------------------------------------ 검증 현황 (--report)

def read_setting(path):
    """설정 파일을 비교할 수 있는 값으로: parm = {이름: 값}, json = dict (개발 환경 경로는 뺀다). 없으면 None."""
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except (OSError, TypeError):
        return None
    if path.endswith(".json"):
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            return None
        if isinstance(data, dict):
            data.pop("simulation_app_path", None)
        return data
    values = {}
    for raw in text.splitlines():
        parts = raw.split("#", 1)[0].replace(",", " ").split()
        if len(parts) >= 2:
            try:
                values[parts[0]] = float(parts[1])
            except ValueError:
                pass
    return values


def current_settings():
    """지금 설정 파일 (개발 환경 config_reference·팀 저장소 config 어느 쪽이든)."""
    conf = next((d for d in (os.path.join(BASE, "config_reference"), os.path.join(BASE, "config"))
                 if os.path.isfile(os.path.join(d, "mavlink_drone.parm"))), None)
    return {"mavlink_drone.parm": read_setting(os.path.join(conf, "mavlink_drone.parm")) if conf else None,
            "dds_drone.parm": read_setting(os.path.join(conf, "dds_drone.parm")) if conf else None,
            "drones.sitl.json": read_setting(os.path.join(BASE, "scenarios", "drones.sitl.json"))}


def same_settings(folder, current):
    """기록 폴더에 남은 설정 3개가 지금 설정과 모두 같으면 True (하나라도 없으면 False)."""
    for name in SETTING_FILES:
        saved = read_setting(os.path.join(folder, name))
        if saved is None or current.get(name) is None or saved != current[name]:
            return False
    return True


def collect(root=LOG_ROOT, current=None):
    """{시나리오: [(시각, 통과?, 출처, 지금 설정?)]} - run_live 기록의 demos.jsonl 구간을 모두 다시 판정한다."""
    current = current if current is not None else current_settings()
    found = {name: [] for name in SCENARIOS}
    for folder in sorted(glob.glob(os.path.join(root, "live_*"))):
        records = [r for r in read_events(os.path.join(folder, "demos.jsonl"))[0] if r.get("scenario") in SCENARIOS]
        if not records:
            continue
        events, _ = read_events(os.path.join(folder, "mission-events.jsonl"))
        faults, _ = read_events(os.path.join(folder, "fault-events.jsonl"))
        same = same_settings(folder, current)
        for rec in records:
            name, since, until = rec["scenario"], rec.get("since"), rec.get("until")
            run = Run(since=since, until=until, names={})
            for ev in events:
                run.feed(ev)
            ok, _ = scenario_verdict(run, in_window(faults, since, until), name)
            found[name].append((since or 0.0, ok, os.path.basename(folder), same))
    return found


def report(root=LOG_ROOT, out=print, current=None):
    """정상 운용·장애 시나리오마다 '지금 설정'으로 돌린 마지막 기록의 판정. 전부 통과면 0."""
    found = collect(root, current)
    passed, todo = 0, []
    out(f"정상 운용 + 장애 시나리오 3개 검증 현황 (기록 {root})")
    out("  '지금 설정' = 기록 폴더에 남은 속도·경로 설정(mavlink_drone.parm, dds_drone.parm, drones.sitl.json)이 지금과 같다")
    for name, (title, story) in SCENARIOS.items():
        runs = sorted(found[name], key=lambda r: r[0])
        now_runs = [r for r in runs if r[3]]
        out(f"  {title}: {story}")
        if now_runs:
            when, ok, folder, _ = now_runs[-1]
            out(f"       {'통과' if ok else '실패'}  {stamp(when)}  {folder}  (지금 설정)")
            if ok:
                passed += 1
                continue
            out(f"       자세히: python3 {HERE}/mission_check.py {os.path.join(root, folder)} --scenario {name}")
        elif runs:
            when, ok, folder, _ = runs[-1]
            out(f"       지금 설정 기록 없음 (예전 설정: {'통과' if ok else '실패'} {stamp(when)} {folder})")
        else:
            out("       기록 없음")
        out("       → " + DEMO_COMMANDS[name].format(d=HERE) + "   (LLM 없이: 뒤에 --no-llm)")
        todo.append(title)
    out(f"지금 설정으로 통과: {passed}/{len(SCENARIOS)}" + (f". 남은 것: {', '.join(todo)}" if todo else ". 전부 검증됨"))
    return 0 if not todo else 1


def preflight(path, out=print):
    """새 장애 시험 전에: 이 run_live 기록에 비행 중이거나 실패한 작업이 있으면 알린다. 문제 없으면 0."""
    events, _ = read_events(path)
    run = Run(names={})
    for ev in events:
        run.feed(ev)
    running = [task["drone"] for task in run.tasks.values() if task["state"] is None]
    failed = [task["drone"] for task in run.tasks.values() if task["state"] not in (None, "succeeded")]
    if run.server_stopped:
        out("!! 이 기록의 미들웨어 서버가 이미 꺼졌다. run_live.sh 를 다시 띄울 것")
        return 1
    if running:
        out(f"!! 아직 비행 중인 작업이 있다 ({', '.join(running)}). 착륙한 뒤 할 것")
        return 1
    if failed:
        out(f"!! 앞 명령에서 실패한 드론이 있다 ({', '.join(sorted(set(failed)))}). 미들웨어에서 unavailable 로 남는다.")
        out("   run_live.sh 를 Ctrl+C 로 끄고: AirSim 다시 켜기 → start_hetero_sim.sh → run_live.sh 다음에 할 것")
        return 1
    return 0


# ------------------------------------------------------------------ 자체 점검

def selftest():
    """가짜 이벤트로 판정을 점검한다."""
    results = []

    def check(name, ok):
        results.append(bool(ok))
        print(f"  [{'OK' if ok else '실패'}] {name}")

    def at(sec):
        return datetime.fromtimestamp(1_800_000_000 + sec).astimezone().isoformat()

    def plan_events(fail_land=False):
        evs = [{"event": "server_started", "logged_at": at(0)},
               {"event": "plan_submitted", "logged_at": at(10), "mission_id": "mission-x", "attempt": 1,
                "plan": {"assignments": [{"drone_id": "drone-01", "area_id": "sector-9"}]}},
               {"event": "plan_rejected", "logged_at": at(11), "mission_id": "mission-x", "attempt": 1,
                "errors": [{"code": "UNKNOWN_AREA", "message": "area_id sector-9 is not in the mission context"}]},
               {"event": "plan_submitted", "logged_at": at(20), "mission_id": "mission-x", "attempt": 2,
                "plan": {"assignments": [{"drone_id": "drone-01", "area_id": "sector-0-0"},
                                         {"drone_id": "drone-02", "area_id": "sector-1-1"}]}}]
        tasks, groups = [], []
        for n, (drone, area) in enumerate((("drone-01", "sector-0-0"), ("drone-02", "sector-1-1")), 1):
            tid = f"task-{n:03d}-{drone}"
            tasks.append({"task_id": tid, "drone_id": drone, "area_id": area, "command_count": 4})
            groups.append([{"command_id": f"{tid}-c0{i}", "task_id": tid,
                            "payload": {"command_type": k, **({"altitude_m": 30.0} if k == "takeoff" else {})}}
                           for i, k in enumerate(("takeoff", "goto", "goto", "land"), 1)])
        evs.append({"event": "plan_accepted", "logged_at": at(21), "mission_id": "mission-x", "attempt": 2,
                    "tasks": tasks, "commands": groups})
        clock = 30
        for group in groups:
            for c in group:
                clock += 10
                bad = fail_land and c["payload"]["command_type"] == "land" and c["task_id"].endswith("drone-01")
                evs.append({"event": "command_transition", "logged_at": at(clock), "task_id": c["task_id"],
                            "command_id": c["command_id"], "command_type": c["payload"]["command_type"],
                            "status": "timed_out" if bad else "succeeded", "elapsed_since_sent_s": 9.5,
                            "error": {"code": "LAND_NOT_CONFIRMED", "message": "not disarmed"} if bad else None})
            clock += 1
            evs.append({"event": "task_finished", "logged_at": at(clock), "task_id": group[0]["task_id"],
                        "state": "failed" if fail_land and group[0]["task_id"].endswith("drone-01") else "succeeded"})
        return evs

    run = Run(since=None, names={"sector-0-0": "남", "sector-1-1": "북"})
    lines = [text for ev in plan_events() for _, text in run.feed(ev)]
    ok, summary = run.verdict()
    check("거부 뒤 다시 내서 수락되고 두 작업이 다 끝나면 통과", ok and run.finished())
    check("거부 이유(code)를 진행 줄에 보여 준다", any("UNKNOWN_AREA" in s for s in lines))
    check("이동은 k/N 으로 보여 준다", any("drone-02 이동 2/2 완료" in s for s in lines))
    check("섹터에 동서남북 이름을 붙인다", any("drone-01 → 남(sector-0-0)" in s for s in lines))
    check("요약에 작업별 이동 수와 고도", any("drone-01 남(sector-0-0), 30m: 이동 2/2" in s for s in summary))

    run = Run(names={})
    for ev in plan_events(fail_land=True):
        run.feed(ev)
    ok, summary = run.verdict()
    check("착륙 확인 실패면 실패, 이유(code)를 보여 준다",
          not ok and any("drone-01 착륙 timed_out (LAND_NOT_CONFIRMED" in s for s in summary))

    run = Run(names={})
    run.feed({"event": "server_started", "logged_at": at(0)})
    ok, summary = run.verdict()
    check("계획이 없으면 실패 (계획이 제출되지 않았다)", not ok and any("제출되지 않았다" in s for s in summary))

    events = plan_events()
    since = last_command_since(events)
    check("판정만 할 때 마지막 명령의 시작 = 첫 제출 (재시도 포함)", since is not None and abs(since - (1_800_000_010 - 0.001)) < 1e-6)
    later = Run(since=since + 5, names={})
    for ev in events:
        later.feed(ev)
    check("since 앞의 이벤트는 세지 않는다", later.submitted == 1 and not later.rejections)
    window = Run(since=since, until=1_800_000_000 + 25, names={})
    for ev in events:
        window.feed(ev)
    check("until 뒤의 이벤트도 세지 않는다 (판정 구간)", window.accepted == [2] and not any(
        task["done"] for task in window.tasks.values()))

    def geo(north, east):
        lat0, lon0 = -35.3632609, 149.1651968
        return {"latitude": lat0 + north / M_PER_DEG,
                "longitude": lon0 + east / (M_PER_DEG * math.cos(math.radians(lat0)))}

    def flight(assign, clock, outcome=None, attempt=1, returned=True):
        """명령 하나의 가짜 기록 (2026-11 실제 기록과 같은 모양). assign = [(드론, 섹터, 홈에서 섹터 중심 m)],
        outcome = {드론: 'link' | 'battery' | 'stall'}. (미들웨어 이벤트, 장애 기록)"""
        evs, fevs, tasks, groups = [], [], [], []
        for n, (drone, area, reach) in enumerate(assign, 1):
            tid = f"task-{clock}-{n}-{drone}"
            side = reach / math.sqrt(2)
            cmds = [("takeoff", None), ("goto", geo(side - 20, side - 20)), ("goto", geo(side + 20, side + 20)),
                    ("goto", geo(0, 0)), ("land", None)]
            tasks.append({"task_id": tid, "drone_id": drone, "area_id": area})
            groups.append([{"command_id": f"{tid}-c{i}", "task_id": tid,
                            "payload": {"command_type": k, **({"target": target} if target else {})}}
                           for i, (k, target) in enumerate(cmds, 1)])
        evs.append({"event": "plan_accepted", "logged_at": at(clock), "mission_id": "m", "attempt": attempt,
                    "tasks": tasks, "commands": groups})
        for task, group in zip(tasks, groups):
            drone, kind = task["drone_id"], (outcome or {}).get(task["drone_id"])
            for c in group:
                clock += 5
                ctype = c["payload"]["command_type"]
                ev = {"event": "command_transition", "logged_at": at(clock), "task_id": task["task_id"],
                      "drone_id": drone, "command_id": c["command_id"], "command_type": ctype,
                      "status": "succeeded", "error": None}
                if kind and c["command_id"].endswith("-c2"):
                    if kind == "link":
                        ev.update(status="failed", error={"code": "LINK_LOST", "message": "no telemetry"})
                    else:
                        ev.update(status="timed_out", error={"code": "WAYPOINT_TIMEOUT", "message": "x"})
                elif kind and ctype != "takeoff":
                    ev.update(status="failed", error={"code": "TASK_ABORTED", "message": "x"})
                evs.append(ev)
            mark = {"logged_at": at(clock), "drone": drone, "identities": [drone]}
            if kind == "link":
                fevs += [{**mark, "event": "fault_link_state", "state_from": "ok", "state_to": "degraded",
                          "distance_m": 250.3},
                         {**mark, "event": "fault_link_state", "state_from": "degraded", "state_to": "down",
                          "distance_m": 330.4},
                         {**mark, "event": "fault_failsafe", "mode": "rtl", "sent": True}]
                evs += [{"event": "link_lost", "logged_at": at(clock), "drone_id": drone, "wait_s": 20.0},
                        {"event": "link_restored", "logged_at": at(clock + 1), "drone_id": drone, "outage_s": 8.0},
                        {"event": "link_recovery_finished", "logged_at": at(clock + 2), "drone_id": drone,
                         "outcome": "landed", "return_leg": "reached"}]
            if kind == "battery":
                fevs.append({**mark, "event": "fault_battery_low", "pct": 20.0, "sent": True, "mode": "rtl",
                             "home_distance_m": 160.2})
            if kind == "stall":
                fevs.append({**mark, "event": "fault_goto_dropped", "reason": "stall", "dropped": 1, "count": 1})
            if kind in ("battery", "stall"):
                evs.append({"event": "timeout_return_finished", "logged_at": at(clock + 1), "drone_id": drone,
                            "return_leg": "reached", "outcome": "landed"} if returned else
                           {"event": "emergency_stop", "logged_at": at(clock + 1), "drone_id": drone, "landed": True})
            clock += 3
            evs.append({"event": "task_finished", "logged_at": at(clock), "task_id": task["task_id"],
                        "drone_id": drone, "state": "failed" if kind else "succeeded"})
        return evs, fevs

    def fed(*event_lists):
        run = Run(names={})
        for evs in event_lists:
            for ev in evs:
                run.feed(ev)
        return run

    south_north = [("drone-01", "sector-0-0", 159), ("drone-02", "sector-1-1", 336)]

    evs, fevs = flight(south_north, 100, outcome={"drone-02": "link"})
    ok, lines = scenario_verdict(fed(evs), fevs, "link")
    check("1 통신 두절: 북 드론 악화 → 330m 두절 → RTL → LINK_LOST → 복구 → 귀환·착륙, 남 드론 성공이면 통과",
          ok and any("330m 에서 두절" in s for s in lines) and any("8초 두절" in s for s in lines))
    evs, _ = flight(south_north, 100)
    ok, lines = scenario_verdict(fed(evs), [], "link")
    check("1 통신 두절: 아무 드론도 안 끊기면 장애 시험이 안 된 것 → 실패", not ok and any("장애 시험이 안 됐다" in s for s in lines))

    low = [{"event": "fault_started", "logged_at": at(99), "drone": "drone-01", "identities": ["drone-01"],
            "kind": "battery", "start_pct": 40.0,
            "params": {"pct": 0.0, "from_pct": 40.0, "ramp_s": 0.0, "drain_s": 140.0, "rtl_pct": 20.0}}]
    first, fevs = flight(south_north, 100, outcome={"drone-01": "battery"})
    ok, lines = scenario_verdict(fed(first), low + fevs, "battery-1")
    check("2 배터리 부족 1단계: 40% 드론에 가까운 섹터 → 20% 귀환 → WAYPOINT_TIMEOUT → 귀환 착륙, 완충 드론 성공",
          ok and any("우선순위 배정" in s and "159m" in s for s in lines)
          and any("timeout 처럼 처리" in s for s in lines))
    second, _ = flight([("drone-02", "sector-0-0", 159)], 400, attempt=2)
    ok, lines = scenario_verdict(fed(first, second), low + fevs, "battery")
    check("2 배터리 부족: 못 끝낸 남(sector-0-0)을 완충 drone-02 가 다시 맡아 성공하면 통과",
          ok and any("재배정 계획 수락: sector-0-0 (drone-01 실패) → drone-02" in s for s in lines))
    swapped, fevs2 = flight([("drone-01", "sector-1-1", 336), ("drone-02", "sector-0-0", 159)], 100,
                            outcome={"drone-01": "battery"})
    ok, lines = scenario_verdict(fed(swapped), low + fevs2, "battery-1")
    check("2 배터리 부족: 부족 드론이 먼 섹터를 받으면 실패", not ok and any("더 먼 섹터" in s for s in lines))
    landed_there, fevs3 = flight(south_north, 100, outcome={"drone-01": "battery"}, returned=False)
    ok, lines = scenario_verdict(fed(landed_there), low + fevs3, "battery-1")
    check("2 배터리 부족: 귀환 없이 그 자리에 내리면 실패 (미들웨어 귀환 처리 필요)",
          not ok and any("귀환 기록이 없다" in s for s in lines))

    first, fevs = flight(south_north, 100, outcome={"drone-02": "stall"})
    ok, lines = scenario_verdict(fed(first), fevs, "timeout-1")
    check("3 웨이포인트 미도달 1단계: goto 무시 → WAYPOINT_TIMEOUT → 귀환 착륙, 다른 드론 성공이면 통과",
          ok and any("홈으로 귀환 후 착륙" in s for s in lines))
    second, _ = flight([("drone-01", "sector-1-1", 336)], 400, attempt=2)
    ok, lines = scenario_verdict(fed(first, second), fevs, "timeout")
    check("3 웨이포인트 미도달: 못 끝낸 북(sector-1-1)을 drone-01 이 다시 맡아 성공하면 통과",
          ok and any("재배정 계획 수락: sector-1-1 (drone-02 실패) → drone-01" in s for s in lines))
    ok, lines = scenario_verdict(fed(first), fevs, "timeout")
    check("3 웨이포인트 미도달: 재배정 명령이 없으면 실패", not ok and any("다시 배정한 계획이 없다" in s for s in lines))

    calm, _ = flight(south_north, 100)
    ok, lines = scenario_verdict(fed(calm), [], "normal")
    check("0 정상 운용: 장애 없이 drone-01 가까운 남·drone-02 먼 북, 둘 다 끝까지 수색 → 착륙이면 통과",
          ok and any("배정 규칙: 배터리가 같으면" in s for s in lines)
          and sum("수색 끝까지 (" in s for s in lines) == 2 and any("비상 정지 없음" in s for s in lines))
    swapped_calm, _ = flight([("drone-01", "sector-1-1", 336), ("drone-02", "sector-0-0", 159)], 100)
    ok, lines = scenario_verdict(fed(swapped_calm), [], "normal")
    check("0 정상 운용: drone-01 이 먼 섹터를 받으면 실패 (배정 규칙)", not ok and any("배정 규칙과 다르다" in s for s in lines))
    leftover = [{"event": "fault_started", "logged_at": at(99), "drone": "drone-02", "identities": ["drone-02"],
                 "kind": "link", "params": {"mode": "range"}}]
    ok, lines = scenario_verdict(fed(calm), leftover, "normal")
    check("0 정상 운용: 장애가 걸려 있었으면 실패 (정상 운용 시험이 아님)",
          not ok and any("장애가 걸려 있었다 (link 장애 시작)" in s for s in lines))
    stalled, _ = flight(south_north, 100, outcome={"drone-02": "stall"})
    ok, lines = scenario_verdict(fed(stalled), [], "normal")
    check("0 정상 운용: 작업 하나라도 실패하면 실패", not ok and any("drone-02 작업 failed" in s for s in lines))

    def write(folder, name, rows):
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, name), "w", encoding="utf-8") as f:
            f.write("".join(json.dumps(r) + "\n" for r in rows) if isinstance(rows, list) else rows)

    with tempfile.TemporaryDirectory() as root:
        current = {"mavlink_drone.parm": {"WPNAV_SPEED": 1800.0}, "dds_drone.parm": {"WP_SPD": 10.0},
                   "drones.sitl.json": {"waypoints": {"max_leg_m": 130}}}

        def settings(folder, speed):
            write(folder, "mavlink_drone.parm", f"# 시험\nWPNAV_SPEED {speed}\n")
            write(folder, "dds_drone.parm", "WP_SPD 10.0\n")
            write(folder, "drones.sitl.json", json.dumps({"simulation_app_path": "/x", "waypoints": {"max_leg_m": 130}}))

        new = os.path.join(root, "live_20261117_210000")
        evs, fevs = flight(south_north, 100, outcome={"drone-02": "link"})
        calm, _ = flight(south_north, 1000)          # 같은 기록 폴더에서 이어서 한 정상 운용
        write(new, "mission-events.jsonl", evs + calm)
        write(new, "fault-events.jsonl", fevs)
        write(new, "demos.jsonl", [{"scenario": "link", "since": 1_800_000_000 + 90, "until": 1_800_000_000 + 400,
                                    "passed": True},
                                   {"scenario": "normal", "since": 1_800_000_000 + 990,
                                    "until": 1_800_000_000 + 1300, "passed": True}])
        settings(new, 1800)
        old = os.path.join(root, "live_20261117_150000")
        first, fevs = flight(south_north, 100, outcome={"drone-01": "battery"})
        second, _ = flight([("drone-02", "sector-0-0", 159)], 400, attempt=2)
        write(old, "mission-events.jsonl", first + second)
        write(old, "fault-events.jsonl", low + fevs)
        write(old, "demos.jsonl", [{"scenario": "battery", "since": 1_800_000_000 + 90,
                                    "until": 1_800_000_000 + 900, "passed": True}])
        settings(old, 1440)
        lines = []
        code = report(root, out=lines.append, current=current)
        text = "\n".join(lines)
        check("현황: 지금 설정으로 통과한 것(0 정상 운용, 1 통신 두절)만 센다", "지금 설정으로 통과: 2/4" in text and code == 1)
        check("현황: 예전 설정 기록(속도가 다른 배터리)은 통과로 안 센다", "지금 설정 기록 없음 (예전 설정: 통과" in text)
        check("현황: 기록이 없는 시나리오는 확인 명령을 알려 준다", "run_live.sh demo timeout" in text)

    with tempfile.TemporaryDirectory() as folder:
        busy = os.path.join(folder, "mission-events.jsonl")
        evs, _ = flight(south_north, 100, outcome={"drone-02": "stall"})
        write(folder, "mission-events.jsonl", evs)
        quiet = []
        check("demo 전 점검: 앞 명령에서 실패한 드론이 있으면 멈춘다", preflight(busy, out=quiet.append) == 1
              and any("unavailable" in s for s in quiet))
        evs, _ = flight([("drone-01", "sector-0-0", 159)], 100)
        write(folder, "mission-events.jsonl", evs)
        check("demo 전 점검: 다 성공해 착륙했으면 통과", preflight(busy, out=quiet.append) == 0)

    passed = sum(results)
    print(f"{passed}/{len(results)} 통과")
    return 0 if passed == len(results) else 1


def main(argv=None):
    parser = argparse.ArgumentParser(prog="mission_check.py", description="자연어 명령·정상 운용·장애 시나리오 판정 (이벤트 기록)",
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("events", nargs="?", help="기록 폴더 또는 mission-events.jsonl (기본: 가장 최근 live_*)")
    parser.add_argument("--follow", action="store_true", help="새 계획을 끝날 때까지 따라가며 보여 주고 판정")
    parser.add_argument("--since", type=float, default=None, help="이 시각(epoch 초) 뒤의 이벤트만 본다")
    parser.add_argument("--judge-since", type=float, default=None,
                        help="--follow --scenario 판정을 이 시각부터의 기록으로 한다 (재배정 2단계에서 1단계까지 함께)")
    parser.add_argument("--timeout", type=float, default=1200.0, help="따라가는 최대 시간 초 (기본 1200)")
    parser.add_argument("--scenario", choices=list(SCENARIOS) + list(STAGES),
                        help="시나리오 기대대로 판정 (normal / link / battery / timeout. --follow 와 함께도 된다)")
    parser.add_argument("--report", action="store_true", help="정상 운용·장애 시나리오 3개가 지금 설정으로 검증됐는지 현황")
    parser.add_argument("--preflight", action="store_true", help="run_live 기록에 비행 중·실패한 작업이 있으면 알린다 (demo 전)")
    parser.add_argument("--selftest", action="store_true", help="가짜 이벤트로 판정 점검")
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()
    if args.report:
        return report()
    path = os.path.expanduser(args.events) if args.events else None
    if path and os.path.isdir(path):
        path = os.path.join(path, "mission-events.jsonl")
    path = path or newest_events()
    if args.preflight:
        return preflight(path) if path and os.path.exists(path) else 0
    if not path or not os.path.exists(path):
        print(f"미들웨어 이벤트 기록이 없다 ({LOG_ROOT}/live_*/mission-events.jsonl). run_live.sh 가 떠 있는지 볼 것")
        return 1
    if args.follow:
        ok = follow(path, args.since if args.since is not None else time.time() - 1.0, args.timeout,
                    scenario=args.scenario, judge_since=args.judge_since)
    elif args.scenario:
        ok = check_scenario(path, args.scenario, since=args.since)
    else:
        ok = check_file(path, args.since)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
