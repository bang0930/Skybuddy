"""두 드론이 실제로 같은 속도로 나는지 비행 기록으로 확인한다 (기체에 명령은 안 보낸다).

  python3 speed_check.py                                  가장 최근 비행 기록 (~/skybuddy/logs/live_*·scenario_*)
  python3 speed_check.py ~/skybuddy/logs/live_20261117_134357
  python3 speed_check.py --selftest

보는 것
  구간   미들웨어 이벤트 기록(mission-events*.jsonl)의 goto 하나 = 보낸 시각부터 도착 판정(3m)까지.
         길이는 보낸 순간의 드론 위치(텔레메트리)에서 목표까지
  속도   텔레메트리 기록(telemetry*.jsonl)의 수평 속도 (vx, vy)
  유지   2초 넘게 계속 낸 가장 빠른 속도 = 순항 속도 (긴 구간이 있어야 나온다)

같은 길이(10m 단위로 묶음) 구간의 평균 시간을 드론끼리 비교한다. 비가 1.0 에 가까우면(±10%) 같은 속도다.
drone-01(ArduCopter 4.3)이 다르면 mavlink_drone.parm 에 넣을 값을 알려 준다 (4.3 은 실제 속도가 설정의
약 0.555 배라 drone-02 와 설정값이 다르다. INTERFACE.md 3-2).
"""
import argparse
import glob
import json
import math
import os
import sys
import tempfile
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.expanduser("~/skybuddy/logs")
M_PER_DEG = 111320.0
HOLD_S = 2.0            # 유지 속도: 이 시간 넘게 계속 낸 속도
BIN_M = 10.0            # 같은 길이로 묶는 단위
SAME = 0.10             # 이만큼(10%) 안이면 같은 속도로 본다
ACCEL_MAX = 560.0       # WPNAV_ACCEL 제안 상한 (ANGLE_MAX 30도 = 566 cm/s/s)


def conf_dir():
    """개발 환경(config_reference)과 팀 저장소(services/simulation/config) 어느 쪽이든."""
    for d in (os.path.join(HERE, "config_reference"), os.path.join(os.path.dirname(HERE), "config")):
        if os.path.isfile(os.path.join(d, "mavlink_drone.parm")):
            return d
    return None


def parse_time(text):
    return datetime.fromisoformat(str(text).replace("Z", "+00:00")).timestamp()


def read_jsonl(path):
    rows = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    except OSError:
        pass
    return rows


def read_parm(path):
    """{이름: 값} (sim_vehicle.py 형식: 한 줄에 '이름 값', # 뒤는 주석)."""
    values = {}
    try:
        with open(path, encoding="utf-8") as f:
            for raw in f:
                parts = raw.split("#", 1)[0].replace(",", " ").split()
                if len(parts) >= 2:
                    try:
                        values[parts[0]] = float(parts[1])
                    except ValueError:
                        pass
    except OSError:
        pass
    return values


def dist_m(a, b):
    """두 (위도, 경도) 사이 수평 거리 m (수백 m 안이라 평면 근사)."""
    lat = math.radians((a[0] + b[0]) / 2.0)
    return math.hypot((b[0] - a[0]) * M_PER_DEG, (b[1] - a[1]) * M_PER_DEG * math.cos(lat))


def load_tracks(rows):
    """{드론: [(시각, 수평 속도, (위도, 경도) 또는 None)]} 시각순."""
    tracks = {}
    for rec in rows:
        tel = rec.get("telemetry") or {}
        drone, vx, vy = rec.get("drone"), tel.get("vx"), tel.get("vy")
        if not drone or vx is None or vy is None or not rec.get("logged_at"):
            continue
        try:
            when = parse_time(rec["logged_at"])
        except ValueError:
            continue
        pos = (tel["lat"], tel["lon"]) if tel.get("lat") is not None and tel.get("lon") is not None else None
        tracks.setdefault(drone, []).append((when, math.hypot(vx, vy), pos))
    for samples in tracks.values():
        samples.sort(key=lambda s: s[0])
    return tracks


def position_at(samples, when, max_gap=2.0):
    """when 에 가장 가까운 기록의 위치 (max_gap 초 넘게 떨어져 있으면 None)."""
    best, best_gap = None, None
    for t, _, pos in samples:
        if pos is None:
            continue
        gap = abs(t - when)
        if best_gap is None or gap < best_gap:
            best, best_gap = pos, gap
        elif t > when:
            break
    return best if best_gap is not None and best_gap <= max_gap else None


def load_legs(events, tracks):
    """{드론: [{start, end, seconds, meters, peak}]}. 도착(succeeded)한 goto 만."""
    targets, sent_at, legs = {}, {}, {}
    for ev in events:
        kind = ev.get("event")
        if kind == "plan_accepted":
            for task in ev.get("commands") or []:
                for cmd in task or []:
                    payload = cmd.get("payload") or {}
                    target = payload.get("target") or {}
                    if payload.get("command_type") == "goto" and "latitude" in target and "longitude" in target:
                        targets[cmd.get("command_id")] = (cmd.get("drone_id"),
                                                          (target["latitude"], target["longitude"]))
        elif kind == "command_transition" and ev.get("command_type") == "goto":
            cid, status = ev.get("command_id"), ev.get("status")
            try:
                when = parse_time(ev.get("updated_at") or ev.get("logged_at"))
            except (TypeError, ValueError):
                continue
            if status == "sent":
                sent_at[cid] = when
            elif status == "succeeded" and cid in sent_at and cid in targets:
                drone, target = targets[cid]
                start = sent_at.pop(cid)
                end = start + float(ev.get("elapsed_since_sent_s") or (when - start))
                samples = tracks.get(drone, [])
                origin = position_at(samples, start)
                if origin is None or end <= start:
                    continue
                speeds = [v for t, v, _ in samples if start <= t <= end]
                legs.setdefault(drone, []).append({
                    "start": start, "end": end, "seconds": end - start,
                    "meters": dist_m(origin, target), "peak": max(speeds) if speeds else None})
    return legs


def sustained_speed(samples, legs, hold_s=HOLD_S):
    """구간 안에서 hold_s 초 넘게 계속 낸 가장 빠른 속도 (없으면 None)."""
    best = None
    for leg in legs:
        window = [(t, v) for t, v, _ in samples if leg["start"] <= t <= leg["end"]]
        for i in range(len(window)):
            k = i
            while k < len(window) and window[k][0] - window[i][0] < hold_s:
                k += 1
            if k >= len(window):
                break
            low = min(v for _, v in window[i:k + 1])
            best = low if best is None else max(best, low)
    return best


def summarize(legs, samples):
    meters = sum(leg["meters"] for leg in legs)
    seconds = sum(leg["seconds"] for leg in legs)
    peaks = [leg["peak"] for leg in legs if leg["peak"] is not None]
    return {"legs": len(legs), "meters": meters, "seconds": seconds,
            "avg": meters / seconds if seconds > 0 else None,
            "peak": max(peaks) if peaks else None,
            "sustained": sustained_speed(samples, legs)}


def by_length(legs, bin_m=BIN_M):
    """{길이 묶음 m: 평균 시간 초}."""
    groups = {}
    for leg in legs:
        groups.setdefault(int(round(leg["meters"] / bin_m) * bin_m), []).append(leg["seconds"])
    return {k: sum(v) / len(v) for k, v in groups.items()}


def fmt(value, unit=" m/s"):
    return "-" if value is None else f"{value:.1f}{unit}"


def report(name, events_path, telemetry_path, parm_path=None, out=print):
    """한 비행 기록을 비교해 출력한다. {드론: 요약} 을 돌려준다 (구간이 없으면 빈 dict)."""
    tracks = load_tracks(read_jsonl(telemetry_path))
    legs = load_legs(read_jsonl(events_path), tracks)
    out(f"기록: {name}")
    if not legs:
        out("  도착한 이동(goto) 구간이 없다 (임무가 안 돌았거나 기록이 비었다)")
        return {}
    rows = {}
    for drone in sorted(legs):
        s = rows[drone] = summarize(legs[drone], tracks.get(drone, []))
        out(f"  {drone}  구간 {s['legs']}개  평균 {fmt(s['avg'])}  최고 {fmt(s['peak'])}  유지 {fmt(s['sustained'])}")
    drones = sorted(rows)
    if len(drones) < 2:
        return rows
    d1, d2 = drones[0], drones[1]
    g1, g2 = by_length(legs[d1]), by_length(legs[d2])
    common = sorted(set(g1) & set(g2), reverse=True)
    ratios = []
    if common:
        out(f"  같은 길이 구간 평균 시간 ({d1} / {d2})")
        for length in common:
            ratio = g1[length] / g2[length]
            ratios.append(ratio)
            out(f"    {length:4d}m  {g1[length]:5.1f}초 / {g2[length]:5.1f}초  = {ratio:.2f}")
    if ratios:
        ratio = sum(ratios) / len(ratios)
    elif rows[d1]["avg"] and rows[d2]["avg"]:
        ratio = rows[d2]["avg"] / rows[d1]["avg"]
    else:
        return rows
    if abs(ratio - 1.0) <= SAME:
        out(f"  판정: 같은 속도 (구간 시간 비 {ratio:.2f}, ±{SAME:.0%} 안)")
    else:
        slower = d1 if ratio > 1 else d2
        out(f"  판정: {slower} 쪽이 {max(ratio, 1 / ratio):.2f}배 느리다 (구간 시간 비 {ratio:.2f})")
    if parm_path is None:       # 비행할 때 쓴 값 (run_live.sh 가 기록 폴더에 남긴다). 없으면 지금 설정 파일
        logged = os.path.join(os.path.dirname(os.path.abspath(events_path)), "mavlink_drone.parm")
        parm_path = logged if os.path.isfile(logged) else None
    hint(rows.get("drone-01"), rows.get("drone-02"), ratio if d1 == "drone-01" else 1 / ratio, parm_path, out)
    return rows


def hint(r1, r2, time_ratio, flight_parm, out):
    """drone-01(4.3) 이 다르면 mavlink_drone.parm 에 넣을 값을 알려 준다.

    flight_parm = 그 비행에 쓴 값 (없으면 지금 설정 파일). 고칠 곳은 언제나 설정 파일이다.
    """
    if not r1 or not r2:
        return
    edit_path = os.path.join(conf_dir(), "mavlink_drone.parm") if conf_dir() else "mavlink_drone.parm"
    used = read_parm(flight_parm or edit_path)
    speed, accel = used.get("WPNAV_SPEED"), used.get("WPNAV_ACCEL")
    s1, s2 = r1["sustained"], r2["sustained"]
    changes = []
    if speed and s1 and s2 and abs(s1 / s2 - 1.0) > SAME:
        out(f"  유지 속도 drone-01 {s1:.1f} / drone-02 {s2:.1f} m/s. drone-01 실제 속도 = 설정의 {s1 / (speed / 100):.3f}배")
        changes.append(f"WPNAV_SPEED {speed:g} -> {round(speed * s2 / s1 / 10) * 10:g}")
    elif accel and abs(time_ratio - 1.0) > SAME:
        # 순항 속도는 같은데 구간 시간이 다르면 가감속 차이다. 구간 시간은 가속의 제곱근에 반비례에 가깝다
        new = min(ACCEL_MAX, round(accel * time_ratio ** 2 / 10) * 10)
        if new != accel:
            changes.append(f"WPNAV_ACCEL {accel:g} -> {new:g}")
    if changes:
        out(f"  고칠 값 ({edit_path}): " + ", ".join(changes))
        out("  고친 뒤 start_hetero_sim.sh 를 다시 돌리면 들어간다 (drone-02 값은 그대로)")


def pairs_in(dirpath):
    """[(이벤트 기록, 텔레메트리 기록)] - mission-events_X.jsonl 과 telemetry_X.jsonl 을 짝짓는다."""
    pairs = []
    for events in sorted(glob.glob(os.path.join(dirpath, "mission-events*.jsonl"))):
        suffix = os.path.basename(events)[len("mission-events"):]
        telemetry = os.path.join(dirpath, "telemetry" + suffix)
        if os.path.exists(telemetry):
            pairs.append((events, telemetry))
    return pairs


def has_flight(pairs):
    for events, _ in pairs:
        try:
            with open(events, encoding="utf-8") as f:
                text = f.read()
        except OSError:
            continue
        if '"goto"' in text and '"succeeded"' in text:
            return True
    return False


def newest_run(root=LOG_ROOT):
    """비행(도착한 goto)이 있는 가장 최근 기록 폴더."""
    dirs = [d for d in glob.glob(os.path.join(root, "live_*")) + glob.glob(os.path.join(root, "scenario_*"))
            if os.path.isdir(d)]
    for d in sorted(dirs, key=os.path.getmtime, reverse=True):
        if has_flight(pairs_in(d)):
            return d
    return None


def selftest():
    """가짜 기록으로 계산을 점검한다."""
    results = []

    def check(name, ok):
        results.append(ok)
        print(f"  [{'OK' if ok else '실패'}] {name}")

    def iso(t):
        return datetime.fromtimestamp(t, tz=timezone.utc).isoformat()

    lat0, lon0, t0 = -35.3625, 149.1657, 1_800_000_000.0
    deg_per_m_lon = 1.0 / (M_PER_DEG * math.cos(math.radians(lat0)))
    target = (lat0, lon0 + 60 * deg_per_m_lon)
    events = [{"event": "plan_accepted", "commands": [
        [{"command_id": f"c{i}", "drone_id": d, "payload": {"command_type": "goto",
          "target": {"latitude": target[0], "longitude": target[1]}}}]
        for i, d in ((1, "drone-01"), (2, "drone-02"))]}]
    telemetry = []
    for cid, drone, speed in (("c1", "drone-01", 3.0), ("c2", "drone-02", 6.0)):
        seconds = 60.0 / speed
        events.append({"event": "command_transition", "command_type": "goto", "command_id": cid,
                       "status": "sent", "updated_at": iso(t0)})
        events.append({"event": "command_transition", "command_type": "goto", "command_id": cid,
                       "status": "succeeded", "updated_at": iso(t0 + seconds), "elapsed_since_sent_s": seconds})
        for n in range(int(seconds / 0.25) + 1):
            t = t0 + n * 0.25
            telemetry.append({"logged_at": iso(t), "drone": drone, "telemetry": {
                "lat": lat0, "lon": lon0 + speed * n * 0.25 * deg_per_m_lon, "vx": 0.0, "vy": speed}})
    with tempfile.TemporaryDirectory() as tmp:
        ev_path = os.path.join(tmp, "mission-events_sections.jsonl")
        tel_path = os.path.join(tmp, "telemetry_sections.jsonl")
        parm = os.path.join(tmp, "mavlink_drone.parm")
        for path, rows in ((ev_path, events), (tel_path, telemetry)):
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(json.dumps(r) for r in rows) + "\n")
        with open(parm, "w", encoding="utf-8") as f:
            f.write("# 시험\nWPNAV_SPEED 600\nWPNAV_ACCEL 500\n")
        check("기록 짝짓기 (mission-events_X / telemetry_X)", pairs_in(tmp) == [(ev_path, tel_path)])
        check("비행 있는 기록으로 본다", has_flight(pairs_in(tmp)))
        tracks = load_tracks(read_jsonl(tel_path))
        legs = load_legs(read_jsonl(ev_path), tracks)
        check("드론마다 구간 1개", sorted(legs) == ["drone-01", "drone-02"] and all(len(v) == 1 for v in legs.values()))
        check("구간 길이 60m", all(abs(v[0]["meters"] - 60) < 0.5 for v in legs.values()))
        check("구간 시간 = 도착 판정까지 (20초 / 10초)",
              abs(legs["drone-01"][0]["seconds"] - 20) < 1e-6 and abs(legs["drone-02"][0]["seconds"] - 10) < 1e-6)
        check("유지 속도 = 순항 속도",
              abs(sustained_speed(tracks["drone-01"], legs["drone-01"]) - 3.0) < 1e-6
              and abs(sustained_speed(tracks["drone-02"], legs["drone-02"]) - 6.0) < 1e-6)
        lines = []
        rows = report("시험", ev_path, tel_path, parm_path=parm, out=lines.append)
        text = "\n".join(lines)
        check("평균 속도 3.0 / 6.0", abs(rows["drone-01"]["avg"] - 3.0) < 0.05 and abs(rows["drone-02"]["avg"] - 6.0) < 0.05)
        check("판정: drone-01 이 2배 느리다", "drone-01 쪽이 2.00배 느리다" in text)
        check("고칠 값: WPNAV_SPEED 600 -> 1200", "WPNAV_SPEED 600 -> 1200" in text)
        same = []
        report("같은 속도", ev_path, ev_path, out=same.append)       # 텔레메트리 없음 -> 구간 없음
        check("텔레메트리가 없으면 구간 없음으로 알린다", any("구간이 없다" in s for s in same))
    passed = sum(results)
    print(f"{passed}/{len(results)} 통과")
    return 0 if passed == len(results) else 1


def main(argv=None):
    parser = argparse.ArgumentParser(prog="speed_check.py", description="두 드론 실제 속도 비교 (비행 기록)",
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("run", nargs="?", help="기록 폴더 (기본: ~/skybuddy/logs 에서 비행이 있는 가장 최근 것)")
    parser.add_argument("--selftest", action="store_true", help="가짜 기록으로 계산 점검")
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()
    run = args.run or newest_run()
    if not run:
        print(f"비행 기록이 없다 ({LOG_ROOT}/live_* 또는 scenario_*). 기록 폴더를 직접 줄 것")
        return 1
    pairs = pairs_in(run)
    if not pairs:
        print(f"{run} 에 mission-events*.jsonl / telemetry*.jsonl 짝이 없다")
        return 1
    for events, telemetry in pairs:
        suffix = os.path.basename(events)[len("mission-events"):-len(".jsonl")]
        report(run + suffix, events, telemetry)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
