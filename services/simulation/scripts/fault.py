"""장애 주입 제어. 미들웨어 서버(또는 kpi_mission)를 띄운 채 다른 터미널에서 실행한다.

어댑터(MavlinkDrone, Ros2Drone)가 제어 파일을 0.2초마다 읽어 적용한다 (fault_injection.py).
대상은 drone_id(drone-01) 또는 연결 문자열(udpin:127.0.0.1:14561)로 지정한다. all 은 전부.

  배터리 부족 (잔량과 전압이 샘플마다 함께 바뀐다)
    python3 fault.py battery drone-01 0 --drain 120               비행(시동 걸린 동안) 120초에 걸쳐 지금 값 -> 0%
    python3 fault.py battery drone-01 0 --drain 120 --from 60     60%에서 시작
    python3 fault.py battery drone-02 25                          지금부터 25%
    python3 fault.py battery drone-01 15 --ramp 60 --after-takeoff 20
                                                                   이륙 20초 뒤부터 시계로 60초 동안 -> 15%
    0%가 되면 더 날 수 없으므로 그 자리 강제 착륙(LAND)하고, 이후 goto 는 무시, 이륙은 거부된다.
  통신 악화·두절 (링크 품질 link_quality_pct 0~100 이 함께 바뀐다)
    python3 fault.py link drone-02 range --degraded-m 250 --down-m 330
                                                                   홈에서 멀어질수록 품질 하락 -> 악화 -> 두절
    python3 fault.py link drone-01 degraded                       샘플을 2초에 1개만 (connected 유지), 품질 20%
    python3 fault.py link drone-01 down --for 15                  15초 동안 두절 (샘플·명령 모두 끊김)
    두절이 5초 이어지면 실제 기체가 스스로 하는 failsafe(RTL)를 어댑터가 대신 보낸다.
    --failsafe land 는 그 자리 착륙, --failsafe none 은 아무것도 안 함(제자리 비행)
  웨이포인트 미도달
    python3 fault.py stall drone-02                               다음 goto 1번을 기체에 보내지 않음
    python3 fault.py stall drone-02 --count 0 --after-takeoff 30  이륙 30초 뒤부터 계속 보내지 않음

  python3 fault.py status                                         걸린 장애, 연결된 어댑터, 최근 기록
  python3 fault.py clear [드론] [--kind battery|link|stall]
  python3 fault.py log [-n 20]                                    장애 이벤트 기록
  python3 fault.py stream [-n 20] [--follow] [--every 4]          미들웨어에 넘긴 텔레메트리 (배터리·링크 품질)
  python3 fault.py stream --csv out.csv                           같은 기록을 CSV 로 (그래프용)

공통 옵션: --after 초(지금부터), --after-takeoff 초(다음 이륙 완료부터), --for 초(지속 시간)
같은 드론에 같은 종류를 다시 걸면 이전 설정을 대체한다.
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime

# 개발 환경(평면 구조)에서도, 팀 저장소의 scripts/ 에서도 app/ 코드를 찾을 수 있게 한다.
_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "fault_injection.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)

from fault_injection import (DEFAULT_DEGRADED_INTERVAL_S,  # noqa: E402
                             DEFAULT_DEGRADED_QUALITY, DEFAULT_FAILSAFE,
                             DEFAULT_FAILSAFE_AFTER_S, DEFAULT_RANGE_HYSTERESIS_M,
                             FAILSAFE_MODES, KINDS, control_path, describe_params,
                             event_log_path, load_control, save_control, stream_log_path,
                             validate_entry)

# 미들웨어 config/drones.real.example.json, session_check.py 와 같은 구성
DEFAULT_DRONES = {"drone-01": "udpin:127.0.0.1:14561", "drone-02": "ros2:ap"}
TELEMETRY_TIMEOUT_S = 3.0      # 미들웨어 RegistryConfig 기본값


# ------------------------------------------------------------------ 대상 해석

def load_drone_map(path):
    """미들웨어 드론 설정 파일에서 drone_id -> connection_string 을 읽는다. 없으면 기본 구성."""
    if path:
        try:
            with open(os.path.expanduser(path), encoding="utf-8") as f:
                data = json.load(f)
            mapping = {d["drone_id"]: d.get("connection_string")
                       for d in data.get("drones", []) if d.get("drone_id")}
            if mapping:
                return mapping
            print(f"[주의] {path} 에 drones 가 없어 기본 구성을 쓴다", file=sys.stderr)
        except (OSError, ValueError, KeyError) as e:
            print(f"[주의] 드론 설정을 읽지 못해 기본 구성을 쓴다 ({path}: {e})", file=sys.stderr)
    return dict(DEFAULT_DRONES)


def resolve_target(target, drone_map):
    """(표시 이름, match 목록). 어댑터는 match 중 하나라도 자기 식별자와 같으면 적용한다."""
    if target in ("all", "*"):
        return "all", ["*"]
    if target in drone_map:
        match = [target]
        if drone_map[target]:
            match.append(drone_map[target])
        return target, match
    ids = [k for k, v in drone_map.items() if v == target]
    if ids:
        return ids[0], [target, *ids]
    print(f"[주의] '{target}' 은 드론 설정에 없다. 이 이름과 정확히 같은 식별자의 어댑터에만 적용된다.",
          file=sys.stderr)
    return target, [target]


# ------------------------------------------------------------------ 시간 표시

def start_spec(args, now):
    if getattr(args, "after", None) is not None:
        return {"type": "at", "at": now + args.after}
    if getattr(args, "after_takeoff", None) is not None:
        return {"type": "after_takeoff", "delay_s": args.after_takeoff}
    return {"type": "now"}


def window(entry):
    """CLI 가 계산할 수 있는 (시작, 끝). after_takeoff 는 어댑터만 알므로 (None, None)."""
    start = entry.get("start") or {"type": "now"}
    created = float(entry.get("created_at", 0))
    if start.get("type") == "now":
        begin = created
    elif start.get("type") == "at":
        begin = float(start.get("at", created))
    else:
        return None, None
    duration = entry.get("duration_s")
    return begin, (begin + float(duration) if duration is not None else None)


def expired(entry, now):
    begin, end = window(entry)
    return end is not None and now >= end


def state_text(entry, now):
    start = entry.get("start") or {"type": "now"}
    duration = entry.get("duration_s")
    if start.get("type") == "after_takeoff":
        text = f"다음 이륙 완료 {float(start.get('delay_s', 0)):g}초 뒤 시작 (어댑터가 판단)"
        return text + (f", {float(duration):g}초 지속" if duration is not None else "")
    begin, end = window(entry)
    if now < begin:
        return f"{begin - now:.0f}초 뒤 시작" + (f", {float(duration):g}초 지속" if duration else "")
    if end is not None and now >= end:
        return "종료됨"
    text = f"진행 중 {now - begin:.0f}초째"
    return text + (f", {end - now:.0f}초 남음" if end is not None else "")


def start_text(entry):
    start = entry.get("start") or {"type": "now"}
    if start["type"] == "now":
        text = "지금"
    elif start["type"] == "at":
        text = f"{start['at'] - entry['created_at']:.0f}초 뒤"
    else:
        text = f"다음 이륙 완료 {float(start['delay_s']):g}초 뒤"
    duration = entry.get("duration_s")
    return text + (f"부터 {float(duration):g}초 동안" if duration is not None else "부터 해제할 때까지")


# ------------------------------------------------------------------ 기록 읽기

def read_events(path, max_bytes=512 * 1024):
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            raw = f.read().decode("utf-8", errors="replace")
    except FileNotFoundError:
        return []
    events = []
    for line in raw.splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            continue               # 앞부분이 잘린 첫 줄 등
    return events


def local_time(iso):
    try:
        return datetime.fromisoformat(iso).astimezone().strftime("%H:%M:%S")
    except (TypeError, ValueError):
        return "??:??:??"


def format_event(ev):
    name, kind = ev.get("drone", "?"), ev.get("kind", "")
    event = ev.get("event", "")
    if event == "fault_started":
        detail = f"{kind} 시작 - {ev.get('summary')}"
    elif event == "fault_ended":
        detail = f"{kind} 종료 ({ev.get('reason')})"
    elif event == "fault_link_state":
        detail = f"통신 {ev.get('state_from')} -> {ev.get('state_to')}"
        if ev.get("distance_m") is not None:
            detail += f" (홈에서 {ev['distance_m']}m)"
        if ev.get("link_quality_pct") is not None:
            detail += f", 링크 품질 {ev['link_quality_pct']}%"
        if ev.get("tx_dropped") is not None:
            detail += f", 두절 중 전달 안 된 명령 {ev['tx_dropped']}건"
    elif event == "fault_goto_dropped":
        if ev.get("reason") == "battery_empty":
            detail = "goto 무시 (배터리 0%, 기체가 날 수 없음)"
        else:
            count = ev.get("count")
            detail = f"goto 무시 {ev.get('dropped')}/{'무제한' if count is None else count}"
    elif event == "fault_failsafe":
        detail = (f"기체 failsafe {str(ev.get('mode')).upper()} 보냄 (두절 {ev.get('after_s')}초)"
                  if ev.get("sent") else f"기체 failsafe 못 보냄: {ev.get('reason')}")
    elif event == "fault_battery_level":
        where = (f", 홈에서 {ev['home_distance_m']}m" if ev.get("home_distance_m") is not None else "")
        detail = (f"배터리 {ev.get('pct')}% {ev.get('voltage_v')}V "
                  f"({ev.get('level')}% 이하, 비행 {ev.get('flight_s')}초{where})")
    elif event == "fault_battery_empty":
        where = (f", 홈에서 {ev['home_distance_m']}m" if ev.get("home_distance_m") is not None else "")
        detail = ("배터리 0% - 그 자리 강제 착륙(LAND)" + where if ev.get("sent")
                  else f"배터리 0% - 강제 착륙 못 보냄: {ev.get('reason')}")
    elif event == "fault_takeoff_blocked":
        detail = f"이륙 거부 (배터리 {ev.get('pct')}%)"
    elif event == "fault_adapter_attached":
        detail = f"어댑터 연결 (pid {ev.get('pid')}, 걸려 있던 장애 {len(ev.get('entries') or [])}건)"
    else:
        detail = event
    return f"  {local_time(ev.get('logged_at'))}  {name:<10} {detail}"


# ------------------------------------------------------------------ 명령

def load_or_exit(path):
    try:
        return load_control(path)
    except ValueError as e:
        sys.exit(f"[오류] 제어 파일 형식이 깨졌다: {e}\n       지우고 다시 걸려면: rm {path}")


def put_entry(kind, args, params):
    drone_map = load_drone_map(args.config)
    target, match = resolve_target(args.drone, drone_map)
    now = time.time()
    entry = {
        "id": f"{kind}-{target}-{int(now * 1000)}",
        "kind": kind,
        "target": target,
        "match": match,
        "created_at": now,
        "start": start_spec(args, now),
        "duration_s": args.duration,
        "params": params,
    }
    try:
        validate_entry(entry)
    except (ValueError, TypeError) as e:
        sys.exit(f"[오류] {e}")

    path = control_path()
    data = load_or_exit(path)
    before = data["entries"]
    kept = [e for e in before if not (e.get("kind") == kind and e.get("target") == target)]
    replaced = len(before) - len(kept)
    kept = [e for e in kept if not expired(e, now)]       # 끝난 항목은 정리한다
    kept.append(entry)
    data["entries"] = kept
    save_control(data, path)

    print(f"[설정] {target} {kind}: {describe_params(kind, params)}")
    print(f"       {start_text(entry)}" + (f" (이전 {kind} 설정 대체)" if replaced else ""))
    print(f"       대상 식별자 {match} / 제어 파일 {path}")
    print("       적용 여부는 python3 fault.py status 또는 서버 터미널의 [장애 주입] 줄로 확인")


def cmd_battery(args):
    put_entry("battery", args, {"pct": args.pct, "from_pct": args.from_pct,
                                "ramp_s": args.ramp, "drain_s": args.drain})


def cmd_link(args):
    params = {"mode": args.mode}
    if args.mode in ("degraded", "range"):
        params["interval_s"] = args.interval
        if args.interval >= TELEMETRY_TIMEOUT_S:
            print(f"[주의] interval {args.interval:g}초는 미들웨어 telemetry_timeout_s({TELEMETRY_TIMEOUT_S:g}초) "
                  f"이상이라 degraded 가 아니라 disconnected 로 보인다", file=sys.stderr)
    if args.mode == "degraded":
        params["quality_pct"] = args.quality
    if args.mode == "range":
        if args.degraded_m is None or args.down_m is None:
            sys.exit("[오류] range 는 --degraded-m 과 --down-m 이 필요하다")
        params.update(degraded_m=args.degraded_m, down_m=args.down_m, hysteresis_m=args.hysteresis)
        if args.center:
            try:
                lat, lon = (float(v) for v in args.center.split(","))
            except ValueError:
                sys.exit("[오류] --center 는 위도,경도 형식이어야 한다 (예: -35.363261,149.165230)")
            params["center"] = [lat, lon]
        else:
            params["center"] = None   # 어댑터가 처음 관측한 위치(홈)를 기준으로 쓴다
    if args.mode in ("down", "range"):
        params.update(failsafe=args.failsafe, failsafe_after_s=args.failsafe_after)
    put_entry("link", args, params)


def cmd_stall(args):
    if args.count < 0:
        sys.exit("[오류] --count 는 0(무제한) 이상이어야 한다")
    put_entry("stall", args, {"count": args.count or None})


def cmd_clear(args):
    path = control_path()
    data = load_or_exit(path)
    before = data["entries"]
    if args.drone:
        target, match = resolve_target(args.drone, load_drone_map(args.config))
        names = set(match) - {"*"}

        def hit(e):
            return e.get("target") == target or bool(names & set(e.get("match") or []))
    else:
        def hit(e):
            return True
    kept = [e for e in before if not (hit(e) and (args.kind is None or e.get("kind") == args.kind))]
    data["entries"] = kept
    save_control(data, path)
    print(f"[해제] {len(before) - len(kept)}건 해제, 남은 장애 {len(kept)}건 ({path})")


def cmd_status(args):
    now = time.time()
    path = control_path()
    try:
        entries = load_control(path)["entries"]
    except ValueError as e:
        sys.exit(f"[오류] {e}")
    print(f"제어 파일: {path}")
    if os.environ.get("SKYBUDDY_FAULT_INJECTION", "1").strip().lower() in ("0", "false", "off", "no"):
        print("  [주의] 이 터미널은 SKYBUDDY_FAULT_INJECTION=0 이다. 서버 쪽도 0 이면 주입이 꺼져 있다")
    if not entries:
        print("  걸려 있는 장애 없음")
    for e in sorted(entries, key=lambda x: (str(x.get("target")), str(x.get("kind")))):
        print(f"  [{e.get('target')}] {e.get('kind')}: {describe_params(e.get('kind'), e.get('params'))}"
              f"  - {state_text(e, now)}")

    log_path = event_log_path()
    events = read_events(log_path)
    attached = {}
    for ev in events:
        if ev.get("event") == "fault_adapter_attached":
            attached[tuple(ev.get("identities") or [])] = ev
    print(f"\n최근 연결된 어댑터 ({log_path}):")
    if not attached:
        print("  기록 없음 - 미들웨어나 kpi_mission 이 이 코드로 드론에 연결하면 여기에 나온다")
    for identities, ev in attached.items():
        print(f"  {local_time(ev.get('logged_at'))}  {', '.join(identities)}  (pid {ev.get('pid')})")
    seen = {i for identities in attached for i in identities}
    for ev in events:                       # 연결 후 배운 별칭(drone-01 등)도 포함한다
        seen.update(ev.get("identities") or [])
    if seen:
        for e in entries:
            match = set(e.get("match") or [])
            if "*" not in match and not (match & seen):
                print(f"  [주의] [{e.get('target')}] {e.get('kind')} 는 연결된 어댑터와 식별자가 맞지 않는다 "
                      f"({sorted(match)}). --config 로 미들웨어 드론 설정 파일을 지정할 것")

    recent = [ev for ev in events if ev.get("event") != "fault_adapter_attached"][-args.n:]
    print("\n최근 기록:")
    if not recent:
        print("  없음")
    for ev in recent:
        print(format_event(ev))


def cmd_log(args):
    events = read_events(event_log_path())
    for ev in events[-args.n:]:
        print(format_event(ev))
    if not events:
        print(f"기록 없음 ({event_log_path()})")


# ------------------------------------------------------------------ 텔레메트리 기록 (stream)

# CSV 로 풀어 쓸 시뮬레이터 쪽 참값 (미들웨어에는 가지 않는 값)
STREAM_SIM_FIELDS = ["armed", "flight_s", "home_distance_m", "link_state",
                     "raw_battery_pct", "raw_battery_voltage_v"]


def stream_path(args):
    path = args.file or stream_log_path()
    if not path:
        sys.exit("[오류] SKYBUDDY_TELEMETRY_LOG=0 이라 기록하지 않는 설정이다. --file 로 파일을 지정할 것")
    return os.path.expanduser(path)


def local_time_tenths(iso):
    try:
        t = datetime.fromisoformat(iso).astimezone()
    except (TypeError, ValueError):
        return "??:??:??.?"
    return t.strftime("%H:%M:%S") + f".{t.microsecond // 100000}"


def _num(value, spec):
    return "-" if value is None else format(value, spec)


def format_sample(rec):
    t = rec.get("telemetry") or {}
    s = rec.get("sim") or {}
    armed = s.get("armed")
    state = "비행" if armed else ("지상" if armed is False else "?")
    return (f"  {local_time_tenths(rec.get('logged_at'))}  {str(rec.get('drone', '?')):<9}"
            f" 배터리 {_num(t.get('battery_remaining_pct'), '5.1f')}%"
            f" {_num(t.get('battery_voltage_v'), '5.2f')}V"
            f"  링크 {_num(t.get('link_quality_pct'), '5.1f')}%"
            f"  고도 {_num(t.get('relative_alt_m'), '5.1f')}m"
            f"  홈거리 {_num(s.get('home_distance_m'), '4.0f')}m"
            f"  통신 {str(s.get('link_state', '?')):<8} {state} {_num(s.get('flight_s'), '.0f')}초")


def pick(records, drone=None, every=1):
    """드론 필터와 '드론마다 N개 중 1개' 솎기."""
    counters = {}
    out = []
    for rec in records:
        name = rec.get("drone")
        if drone and name != drone:
            continue
        counters[name] = counters.get(name, 0) + 1
        if (counters[name] - 1) % max(1, every) == 0:
            out.append(rec)
    return out


def summarize(records):
    per = {}
    for rec in records:
        t = rec.get("telemetry") or {}
        per.setdefault(rec.get("drone"), []).append(t)
    lines = []
    for name, samples in per.items():
        bats = [s.get("battery_remaining_pct") for s in samples if s.get("battery_remaining_pct") is not None]
        links = [s.get("link_quality_pct") for s in samples if s.get("link_quality_pct") is not None]
        text = f"  {name}: 샘플 {len(samples)}개"
        if bats:
            text += f", 배터리 {bats[0]:g}% -> {bats[-1]:g}% (최저 {min(bats):g}%)"
        if links:
            text += f", 링크 품질 최저 {min(links):g}%"
        lines.append(text)
    return lines


def write_stream_csv(records, out_path):
    import csv
    tele_keys = []
    for rec in records:
        for k in (rec.get("telemetry") or {}):
            if k not in tele_keys:
                tele_keys.append(k)
    fields = ["logged_at", "drone"] + tele_keys + [f"sim_{k}" for k in STREAM_SIM_FIELDS] + ["sim_faults"]
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for rec in records:
            t = rec.get("telemetry") or {}
            s = rec.get("sim") or {}
            row = {"logged_at": rec.get("logged_at"), "drone": rec.get("drone")}
            row.update({k: ("" if t.get(k) is None else t.get(k)) for k in tele_keys})
            row.update({f"sim_{k}": ("" if s.get(k) is None else s.get(k)) for k in STREAM_SIM_FIELDS})
            row["sim_faults"] = " ".join(f"{k}={v}" for k, v in (s.get("faults") or {}).items())
            writer.writerow(row)
    return len(records)


def follow_stream(path, drone, every):
    print(f"{path} 를 따라가며 보여 준다 (Ctrl+C 로 끝)")
    try:
        pos = os.path.getsize(path)
    except FileNotFoundError:
        pos = 0
    buf, counters = "", {}
    try:
        while True:
            try:
                size = os.path.getsize(path)
            except FileNotFoundError:
                size = 0
            if size < pos:                      # 새로 만들어졌거나 .1 로 넘어갔다
                pos = 0
            if size > pos:
                with open(path, "rb") as f:
                    f.seek(pos)
                    chunk = f.read(size - pos)
                pos = size
                buf += chunk.decode("utf-8", errors="replace")
                lines = buf.split("\n")
                buf = lines.pop()
                for line in lines:
                    try:
                        rec = json.loads(line)
                    except ValueError:
                        continue
                    name = rec.get("drone")
                    if drone and name != drone:
                        continue
                    counters[name] = counters.get(name, 0) + 1
                    if (counters[name] - 1) % max(1, every) == 0:
                        print(format_sample(rec), flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()


def cmd_stream(args):
    path = stream_path(args)
    if args.follow:
        follow_stream(path, args.drone, args.every)
        return
    if args.csv:
        try:
            with open(path, encoding="utf-8") as f:
                records = []
                for line in f:
                    try:
                        records.append(json.loads(line))
                    except ValueError:
                        continue
        except FileNotFoundError:
            sys.exit(f"[오류] 기록 없음 ({path})")
        records = pick(records, args.drone, args.every)
        n = write_stream_csv(records, args.csv)
        print(f"저장: {args.csv} ({n}줄, 원본 {path})")
        return
    records = read_events(path, max_bytes=4 * 1024 * 1024)
    if not records:
        print(f"기록 없음 ({path}) - 미들웨어 서버(또는 kpi_mission)가 이 코드로 드론 텔레메트리를 읽으면 쌓인다")
        return
    shown = pick(records, args.drone, args.every)
    print(f"{path} (마지막 {min(args.n, len(shown))}줄"
          + (f", 드론마다 {args.every}개 중 1개" if args.every > 1 else "") + ")")
    for rec in shown[-args.n:]:
        print(format_sample(rec))
    print("요약 (파일 끝 4MB 기준):")
    for line in summarize(pick(records, args.drone)):
        print(line)


# ------------------------------------------------------------------ 진입점

def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="fault.py", description="SkyBuddy 장애 주입 제어",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p, timed=True):
        p.add_argument("--config", default=os.environ.get("MIDDLEWARE_DRONES_CONFIG"),
                       help="미들웨어 드론 설정 JSON (drone_id -> 연결 문자열). 기본: 환경 변수 "
                            "MIDDLEWARE_DRONES_CONFIG, 없으면 drone-01/drone-02 기본 구성")
        if timed:
            when = p.add_mutually_exclusive_group()
            when.add_argument("--after", type=float, metavar="초", help="지금부터 이 시간 뒤에 시작")
            when.add_argument("--after-takeoff", type=float, metavar="초",
                              help="다음 이륙 완료(목표 고도 도달)부터 이 시간 뒤에 시작")
            p.add_argument("--for", dest="duration", type=float, metavar="초",
                           help="지속 시간. 없으면 clear 할 때까지")

    p = sub.add_parser("battery", help="배터리 잔량 (정한 값 / 비행 시간 기준 감소 / 시계 기준 감소)")
    p.add_argument("drone")
    p.add_argument("pct", type=float, help="목표 잔량(%%). 0 이면 도달할 때 그 자리 강제 착륙")
    how = p.add_mutually_exclusive_group()
    how.add_argument("--drain", type=float, default=0.0, metavar="초",
                     help="시동이 걸린 동안만 줄여, 이 시간만큼 비행하면 pct 가 된다 (지상에서는 그대로)")
    how.add_argument("--ramp", type=float, default=0.0, metavar="초",
                     help="시계 기준으로 이 시간 동안 pct 까지 줄인다 (지상에서도 줄어든다)")
    p.add_argument("--from", dest="from_pct", type=float, default=None, metavar="PCT",
                   help="감소 시작 잔량 (기본: 장애가 걸리는 순간의 실제 값. SITL 은 100)")
    common(p)
    p.set_defaults(func=cmd_battery)

    p = sub.add_parser("link", help="통신 악화·두절")
    p.add_argument("drone")
    p.add_argument("mode", choices=["degraded", "down", "range"])
    p.add_argument("--interval", type=float, default=DEFAULT_DEGRADED_INTERVAL_S, metavar="초",
                   help=f"degraded 에서 샘플 1개를 내주는 간격 (기본 {DEFAULT_DEGRADED_INTERVAL_S:g})")
    p.add_argument("--quality", type=float, default=DEFAULT_DEGRADED_QUALITY, metavar="PCT",
                   help=f"degraded 에서 보고할 링크 품질 (기본 {DEFAULT_DEGRADED_QUALITY:g}). "
                        f"range 는 거리로 계산한다: 홈 100, --down-m 에서 0")
    p.add_argument("--degraded-m", type=float, help="range: 홈에서 이 거리를 넘으면 degraded")
    p.add_argument("--down-m", type=float, help="range: 홈에서 이 거리를 넘으면 down")
    p.add_argument("--center", metavar="위도,경도",
                   help="range 기준점. 없으면 어댑터가 처음 관측한 위치(이륙 전 홈)")
    p.add_argument("--hysteresis", type=float, default=DEFAULT_RANGE_HYSTERESIS_M, metavar="m",
                   help=f"range 에서 좋아지는 쪽으로 바뀔 때만 더 들어와야 하는 거리 "
                        f"(기본 {DEFAULT_RANGE_HYSTERESIS_M:g})")
    p.add_argument("--failsafe", choices=list(FAILSAFE_MODES), default=DEFAULT_FAILSAFE,
                   help="두절이 이어질 때 기체가 스스로 할 동작 (ArduPilot FS_GCS_ENABLE 흉내). "
                        "rtl=홈 복귀 후 착륙(기본), land=그 자리 착륙, none=제자리 비행")
    p.add_argument("--failsafe-after", type=float, default=DEFAULT_FAILSAFE_AFTER_S, metavar="초",
                   help=f"두절 후 failsafe 까지 시간 (기본 {DEFAULT_FAILSAFE_AFTER_S:g} = "
                        f"ArduPilot FS_GCS_TIMEOUT 기본값)")
    common(p)
    p.set_defaults(func=cmd_link)

    p = sub.add_parser("stall", help="웨이포인트 미도달 (goto 를 기체에 보내지 않음)")
    p.add_argument("drone")
    p.add_argument("--count", type=int, default=1, help="무시할 goto 횟수 (기본 1, 0 = 무제한)")
    common(p)
    p.set_defaults(func=cmd_stall)

    p = sub.add_parser("clear", help="장애 해제")
    p.add_argument("drone", nargs="?", help="없으면 전부")
    p.add_argument("--kind", choices=list(KINDS))
    common(p, timed=False)
    p.set_defaults(func=cmd_clear)

    p = sub.add_parser("status", help="걸린 장애와 최근 기록")
    p.add_argument("-n", type=int, default=10, help="최근 기록 줄 수")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("log", help="장애 이벤트 기록 보기")
    p.add_argument("-n", type=int, default=20)
    p.set_defaults(func=cmd_log)

    p = sub.add_parser("stream", help="미들웨어에 넘긴 텔레메트리 기록 보기 (배터리·전압·링크 품질)")
    p.add_argument("-n", type=int, default=20, help="마지막 몇 줄 (기본 20)")
    p.add_argument("--drone", help="이 드론만 (예: drone-01)")
    p.add_argument("--every", type=int, default=1, metavar="N",
                   help="드론마다 N개 중 1개만 (샘플이 약 4Hz 라 4 = 약 1초에 1줄)")
    p.add_argument("--follow", "-f", action="store_true", help="새로 쌓이는 줄을 계속 보여 준다 (Ctrl+C 로 끝)")
    p.add_argument("--csv", metavar="파일", help="기록 전체를 CSV 로 저장 (그래프용)")
    p.add_argument("--file", help="텔레메트리 기록 파일 (기본: SKYBUDDY_TELEMETRY_LOG, "
                                  "없으면 ~/skybuddy/logs/telemetry-stream.jsonl)")
    p.set_defaults(func=cmd_stream)

    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
