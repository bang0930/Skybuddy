"""장애 주입이 미들웨어까지 알맞은 형태로 전달되는지 시뮬레이터 없이 검증한다.

  python3 verify_faults.py            <- 개발 환경(평면 구조) 기준 경로
  python3 tests/test_faults.py        <- 팀 저장소 기준 경로

  1) 제어 파일 해석과 시간 규칙           표준 라이브러리만 (가짜 시계)
     연속 값: 비행 중에만 닳는 배터리·전압, 링크 품질, 텔레메트리 기록
  2) MavlinkDrone + 가짜 pymavlink 연결    pymavlink 필요
  3) Ros2Drone + 가짜 ROS 메시지           ROS 2 소싱 필요, 없으면 건너뜀
  4) 미들웨어 Registry -> DroneState       팀 저장소 미들웨어 코드 필요, 없으면 건너뜀
  5) 소스 규칙

가짜 연결은 pymavlink 와 같은 경로를 탄다: recv_match() 가 연결 객체의 recv_msg() 를 부르고,
mav.*_send() 가 연결 객체의 write() 로 나간다. 그래서 장애 주입이 감싼 함수가 실제와 똑같이 불린다.
2)~4) 는 실제 시간으로 돌아 약 20초 걸린다. [장애 주입] 줄은 서버 터미널에도 그대로 나오는 출력이다.
"""
import collections
import json
import os
import shutil
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "kpi_mission.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)
SCRIPTS_DIR = _HERE if os.path.exists(os.path.join(_HERE, "fault.py")) \
              else os.path.join(os.path.dirname(_HERE), "scripts")
sys.path.insert(1, SCRIPTS_DIR)

# 시험용 제어 파일·기록을 임시 폴더로 돌린다. 실제 /tmp/skybuddy_faults.json 은 건드리지 않는다.
WORK = tempfile.mkdtemp(prefix="skybuddy_faults_test_")
os.environ["SKYBUDDY_FAULT_FILE"] = os.path.join(WORK, "faults.json")
os.environ["SKYBUDDY_FAULT_LOG"] = os.path.join(WORK, "events.jsonl")
os.environ["SKYBUDDY_TELEMETRY_LOG"] = os.path.join(WORK, "stream.jsonl")
os.environ["SKYBUDDY_FAULT_POLL_S"] = "0"        # 매 호출마다 제어 파일 확인 (기다리지 않게)
os.environ.pop("SKYBUDDY_FAULT_INJECTION", None)
os.environ.pop("MIDDLEWARE_DRONES_CONFIG", None)  # fault.py 가 기본 구성(drone-01/02)을 쓰게

import fault                                                            # noqa: E402
from fault_injection import (FaultInjector, load_control, save_control,  # noqa: E402
                             stream_log_path)

HOME = (-35.363261, 149.165230)
MAV_CS = "udpin:127.0.0.1:14561"
ok = True
skipped = []


def check(label, condition, detail=""):
    global ok
    mark = "OK  " if condition else "실패"
    print(f"  [{mark}] {label}" + (f"  -> {detail}" if detail and not condition else ""))
    if not condition:
        ok = False


def cli(*argv):
    fault.main(list(argv))


def clear_all():
    save_control({"entries": []})


def events(name=None):
    path = os.environ["SKYBUDDY_FAULT_LOG"]
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        out = [json.loads(line) for line in f if line.strip()]
    return [e for e in out if name is None or e.get("event") == name]


def offset(lat, lon, north_m, east_m=0.0):
    import math
    return (lat + north_m / 111320.0,
            lon + east_m / (111320.0 * math.cos(math.radians(lat))))


# ====================================================================== 1) 해석 규칙

class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def entry(kind, params, *, target="drone-01", match=None, created=1000.0, start=None,
          duration=None, eid=None):
    return {"id": eid or f"{kind}-{target}-{created}", "kind": kind, "target": target,
            "match": match or [target, MAV_CS if target == "drone-01" else "ros2:ap"],
            "created_at": created, "start": start or {"type": "now"},
            "duration_s": duration, "params": params}


def injector(clock, identities=(MAV_CS,), drone_id=None):
    return FaultInjector(list(identities), drone_id=drone_id, clock=clock, sleep=clock.sleep,
                         poll_s=0)


def section_rules():
    print("=== 1) 제어 파일 해석과 시간 규칙 (가짜 시계) ===")
    clk = Clock()
    mav = injector(clk)                                     # 미들웨어처럼 drone_id 없이
    dds = injector(clk, identities=("ros2:ap",), drone_id="drone-02")

    save_control({"entries": [entry("battery", {"pct": 15.0})]})
    mav.refresh()
    dds.refresh()
    check("battery 15% 가 대상 드론 샘플에 들어감",
          mav.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"] == 15.0)
    check("다른 드론에는 적용되지 않음",
          dds.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"] == 100)
    check("연결 문자열로 맞춘 어댑터가 drone-01 이라는 이름을 배움", mav.name == "drone-01", mav.name)

    save_control({"entries": [entry("battery", {"pct": 20.0, "from_pct": 100.0, "ramp_s": 60.0})]})
    values = []
    for t in (1000.0, 1030.0, 1060.0, 1100.0):
        clk.t = t
        values.append(mav.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"])
    check("ramp: 100 -> 60 -> 20 -> 20 (60초 동안 선형)", values == [100.0, 60.0, 20.0, 20.0], str(values))

    save_control({"entries": [entry("link", {"mode": "down"}, created=1100.0,
                                    start={"type": "at", "at": 1110.0}, duration=5.0)]})
    states = []
    for t in (1105.0, 1110.0, 1114.9, 1115.0):
        clk.t = t
        states.append(mav.link_state())
    check("--after 10 --for 5: 시작 전 ok, 5초 동안 down, 끝나면 ok",
          states == ["ok", "down", "down", "ok"], str(states))
    reasons = [e.get("reason") for e in events("fault_ended") if e.get("kind") == "link"]
    check("종료 사유가 expired 로 기록됨", reasons[-1:] == ["expired"], str(reasons))

    clk.t = 1200.0
    save_control({"entries": [entry("battery", {"pct": 30.0}, created=1200.0,
                                    start={"type": "after_takeoff", "delay_s": 10.0})]})
    clk.t = 1190.0                       # 항목을 만들기 전의 이륙은 세지 않는다
    mav.note_takeoff()
    clk.t = 1205.0
    pending = mav.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"]
    clk.t = 1220.0
    mav.note_takeoff()
    clk.t = 1229.0
    before = mav.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"]
    clk.t = 1230.0
    after = mav.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"]
    check("--after-takeoff 10: 다음 이륙 10초 뒤부터 적용 (이전 이륙은 무시)",
          (pending, before, after) == (100, 100, 30.0), str((pending, before, after)))

    save_control({"entries": [entry("battery", {"pct": 50.0}, created=1300.0, eid="old"),
                              entry("battery", {"pct": 10.0}, created=1301.0, eid="new")]})
    clk.t = 1302.0
    check("같은 종류가 둘이면 나중에 만든 것만 적용",
          mav.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"] == 10.0)
    clear_all()
    clk.t = 1303.0
    check("제어 파일에서 지우면 바로 원래 값",
          mav.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"] == 100)
    check("해제가 cleared 로 기록됨",
          any(e.get("reason") == "cleared" for e in events("fault_ended")))

    save_control({"entries": [entry("stall", {"count": 2}, created=1400.0)]})
    clk.t = 1401.0
    drops = [mav.drop_goto(1, 2, 30) for _ in range(3)]
    check("stall --count 2: goto 2번 무시 후 정상", drops == [True, True, False], str(drops))
    check("횟수를 다 쓰면 count_reached 로 기록됨",
          any(e.get("reason") == "count_reached" for e in events("fault_ended")))

    save_control({"entries": [entry("link", {"mode": "degraded", "interval_s": 2.0}, created=1500.0)]})
    clk.t = 1500.0
    delivered = 0
    while True:
        ready, _ = mav.telemetry_gate(0.5)   # 기다릴 때는 가짜 시계가 그만큼 간다
        if clk.t >= 1510.0:
            break
        if not ready:
            continue
        mav.deliver({"battery_remaining_pct": 100})
        delivered += 1
        clk.t += 0.25                    # 정상 샘플 간격
    check("degraded 2초: 10초 동안 샘플 5개 (정상이면 40개)", delivered == 5, f"{delivered}개")

    rng = injector(clk, identities=("ros2:ap",), drone_id="drone-02")
    save_control({"entries": [entry("link", {"mode": "range", "degraded_m": 100.0, "down_m": 200.0,
                                             "hysteresis_m": 10.0, "center": None},
                                    target="drone-02", created=1600.0)]})
    clk.t = 1600.0
    rng.observe_position(*HOME)          # 처음 관측한 위치가 기준점(홈)
    seq = []
    for north in (50, 150, 250, 195, 185, 95, 85):
        rng.observe_position(*offset(HOME[0], HOME[1], north))
        seq.append(rng.link_state())
    check("range 100/200m: 멀어지면 degraded -> down, 돌아올 때는 10m 더 들어와야 회복",
          seq == ["ok", "degraded", "down", "down", "degraded", "degraded", "ok"], str(seq))
    last = [e for e in events("fault_link_state") if e.get("drone") == "drone-02"][-1]
    check("통신 상태 변화에 거리가 기록됨", last.get("distance_m") is not None, str(last))

    try:
        json.dumps(rng.status())
        serializable = True
    except (TypeError, ValueError):
        serializable = False
    check("fault_status() 가 JSON 으로 바로 직렬화됨", serializable)

    calls = []
    fs = injector(clk)
    fs.set_failsafe_handler(calls.append)
    save_control({"entries": [entry("link", {"mode": "down", "failsafe": "rtl",
                                             "failsafe_after_s": 5.0}, created=1700.0)]})
    seen = []
    for t in (1700.0, 1704.9, 1705.0, 1706.0):
        clk.t = t
        fs.link_state()
        seen.append(len(calls))
    check("down 5초 뒤 기체 failsafe RTL 을 한 번만 보냄 (#42 가 기다리는 동작)",
          seen == [0, 0, 1, 1] and calls == ["rtl"], str((seen, calls)))
    check("failsafe 가 기록됨", any(e.get("sent") and e.get("mode") == "rtl" for e in events("fault_failsafe")))
    check("fault_status() 에 보낸 failsafe 가 보임", fs.status()["link"].get("failsafe_sent") == "rtl",
          json.dumps(fs.status()["link"], ensure_ascii=False))
    clear_all()
    clk.t = 1750.0
    fs.link_state()                      # 두절이 끝나고
    save_control({"entries": [entry("link", {"mode": "down", "failsafe": "none"}, created=1800.0)]})
    clk.t = 1800.0
    fs.link_state()                      # 새 두절이 시작된 뒤
    clk.t = 1810.0
    fs.link_state()
    check("--failsafe none: 10초가 지나도 아무것도 안 보냄 (FS_GCS_ENABLE=0 과 같음)",
          calls == ["rtl"], str(calls))
    clear_all()

    off = FaultInjector([MAV_CS], enabled=False, poll_s=0)
    save_control({"entries": [entry("battery", {"pct": 5.0})]})
    check("SKYBUDDY_FAULT_INJECTION=0 이면 아무것도 바꾸지 않음",
          off.deliver({"battery_remaining_pct": 100})["battery_remaining_pct"] == 100
          and off.link_state() == "ok")
    clear_all()

    print(" fault.py (명령줄)")
    cli("battery", "drone-01", "15")
    entries = load_control()["entries"]
    check("drone-01 을 연결 문자열까지 묶어 기록 (미들웨어의 MavlinkDrone 에 적용되도록)",
          len(entries) == 1 and entries[0]["match"] == ["drone-01", MAV_CS], str(entries))
    cli("battery", "drone-01", "40")
    check("같은 드론·같은 종류를 다시 걸면 대체", len(load_control()["entries"]) == 1)
    cli("link", "drone-02", "range", "--degraded-m", "100", "--down-m", "200")
    check("range 설정 기록", len(load_control()["entries"]) == 2)
    for bad in (["battery", "drone-01", "150"],
                ["link", "drone-01", "range", "--degraded-m", "300", "--down-m", "200"],
                ["link", "drone-01", "range"]):
        try:
            cli(*bad)
            rejected = False
        except SystemExit:
            rejected = True
        check(f"잘못된 값 거부: {' '.join(bad)}", rejected)
    cli("clear", "drone-01")
    left = load_control()["entries"]
    check("clear drone-01 은 drone-01 것만 해제", [e["target"] for e in left] == ["drone-02"], str(left))
    cli("clear")
    check("clear 는 전부 해제", load_control()["entries"] == [])

    cli("battery", "drone-01", "0", "--drain", "120")
    params = load_control()["entries"][0]["params"]
    check("battery 0 --drain 120: 비행 시간 기준, 시작값은 그때의 실제 값(from_pct 없음)",
          params == {"pct": 0.0, "from_pct": None, "ramp_s": 0.0, "drain_s": 120.0}, str(params))
    cli("link", "drone-01", "degraded", "--quality", "35")
    params = [e for e in load_control()["entries"] if e["kind"] == "link"][0]["params"]
    check("link degraded --quality 35 기록", params.get("quality_pct") == 35.0, str(params))
    for bad in (["battery", "drone-01", "0", "--drain", "10", "--ramp", "10"],
                ["link", "drone-01", "degraded", "--quality", "150"]):
        try:
            cli(*bad)
            rejected = False
        except SystemExit:
            rejected = True
        check(f"잘못된 값 거부: {' '.join(bad)}", rejected)
    cli("clear")


def section_signals():
    print("\n=== 1-2) 연속 값: 비행 중에만 닳는 배터리·전압, 링크 품질, 텔레메트리 기록 (가짜 시계) ===")
    clear_all()
    clk = Clock(5000.0)
    inj = injector(clk)                                  # drone-01 (MAVLink 연결 문자열)
    landings = []
    inj.set_failsafe_handler(landings.append)
    inj.observe_position(*HOME)
    inj.observe_armed(False)

    def sample(battery=100, volt=12.6):
        return inj.deliver({"battery_remaining_pct": battery, "battery_voltage_v": volt,
                            "link_quality_pct": None})

    save_control({"entries": [entry("battery", {"pct": 0.0, "drain_s": 100.0}, created=5000.0,
                                    eid="drain-01")]})
    first = sample()
    check("drain 시작: 그때의 실제 값(100)에서 출발, 전압 12.6V (3셀 만충)",
          (first["battery_remaining_pct"], first["battery_voltage_v"]) == (100.0, 12.6), str(first))
    clk.t = 5050.0
    check("지상(시동 꺼짐) 50초: 줄지 않음", sample()["battery_remaining_pct"] == 100.0)
    inj.observe_armed(True)                               # 이륙
    values = []
    while clk.t < 5100.0:
        clk.t += 5.0
        values.append(sample()["battery_remaining_pct"])
    check("비행 50초: 5초마다 5%씩 이어서 감소 (100초에 0%가 되는 속도)",
          values == [95.0, 90.0, 85.0, 80.0, 75.0, 70.0, 65.0, 60.0, 55.0, 50.0], str(values))
    check("50% 일 때 전압 11.52V (셀당 3.84V x 3)", sample()["battery_voltage_v"] == 11.52)
    inj.observe_armed(False)                              # 착륙해 시동 꺼짐
    clk.t = 5130.0
    check("시동이 꺼진 30초 동안은 그대로 50%", sample()["battery_remaining_pct"] == 50.0)
    inj.observe_armed(True)                               # 다시 이륙
    while clk.t < 5180.0:
        clk.t += 5.0
        sample()
    last = sample()
    check("다시 비행 50초: 0%, 전압 9.81V", (last["battery_remaining_pct"], last["battery_voltage_v"]) == (0.0, 9.81),
          str(last))
    check("비행 중 0% -> 그 자리 강제 착륙(LAND)을 한 번만 보냄", landings == ["land"], str(landings))
    mine = [e for e in events("fault_battery_level") if e.get("entry_id") == "drain-01"]
    levels = [e.get("level") for e in mine]
    check("10% 단위 기록 90, 80, ..., 0 이 한 번씩 순서대로 남음", levels == list(range(90, -1, -10)), str(levels))
    lv30 = [e for e in mine if e.get("level") == 30][-1]
    check("기록에 그때의 비행 시간·위치·홈 거리가 붙음",
          lv30.get("flight_s") is not None and lv30.get("home_distance_m") is not None, str(lv30))
    check("0% 뒤 goto 는 무시", inj.drop_goto(HOME[0], HOME[1], 30) is True)
    check("0% 에서는 이륙 거부", "배터리" in (inj.takeoff_block() or ""))
    status = inj.status()["battery"]
    check("fault_status(): empty, 강제 착륙 보냄, drain 100초",
          status["empty"] and status["forced_landing_sent"] is True and status["drain_s"] == 100.0,
          json.dumps(status, ensure_ascii=False))

    dds = injector(clk, identities=("ros2:ap",), drone_id="drone-02")
    dds.observe_position(*HOME)                           # 처음 관측한 위치가 링크 기준점(홈)
    t = dds.deliver({"battery_remaining_pct": 80.0, "battery_voltage_v": 12.6, "link_quality_pct": None})
    check("장애가 없으면 값 그대로, 링크 품질 100",
          (t["battery_remaining_pct"], t["link_quality_pct"]) == (80.0, 100.0), str(t))
    save_control({"entries": [entry("battery", {"pct": 0.0, "drain_s": 60.0}, target="drone-02",
                                    created=clk.t)]})
    t = dds.deliver({"battery_remaining_pct": 80.0, "battery_voltage_v": 12.6, "link_quality_pct": None})
    check("--from 없이 걸면 그때의 실제 값(80%)에서 출발", t["battery_remaining_pct"] == 80.0, str(t))

    save_control({"entries": [entry("link", {"mode": "range", "degraded_m": 100.0, "down_m": 200.0,
                                             "hysteresis_m": 10.0, "center": None},
                                    target="drone-02", created=clk.t)]})
    quality = []
    for north in (0, 50, 150):
        dds.observe_position(*offset(HOME[0], HOME[1], north))
        quality.append(dds.deliver({"battery_remaining_pct": 100.0, "battery_voltage_v": 12.6,
                                    "link_quality_pct": None})["link_quality_pct"])
    check("range 100/200m 링크 품질: 홈 100 -> 50m 75 -> 150m 25 (200m 에서 0)",
          quality == [100.0, 75.0, 25.0], str(quality))
    change = [e for e in events("fault_link_state") if e.get("drone") == "drone-02"][-1]
    check("통신 상태 변화 기록에 링크 품질이 붙음", change.get("link_quality_pct") == 25.0, str(change))
    save_control({"entries": [entry("link", {"mode": "degraded", "quality_pct": 35.0},
                                    target="drone-02", created=clk.t, eid="degraded-02")]})
    check("degraded 링크 품질 = quality_pct (35)",
          dds.deliver({"battery_remaining_pct": 100.0, "link_quality_pct": None})["link_quality_pct"] == 35.0)

    path = stream_log_path()
    with open(path, encoding="utf-8") as f:
        records = [json.loads(line) for line in f if line.strip()]
    check("텔레메트리 기록: 미들웨어에 넘긴 샘플마다 한 줄", len(records) >= 20, f"{len(records)}줄")
    mine = [r for r in records if r["drone"] == "drone-01"]
    check("기록의 telemetry 는 넘긴 값 그대로, sim 에 시동·비행 시간·통신 상태",
          mine[-1]["telemetry"]["battery_remaining_pct"] == 0.0
          and {"armed", "flight_s", "link_state", "raw_battery_pct"} <= set(mine[-1]["sim"]),
          json.dumps(mine[-1], ensure_ascii=False)[:300])
    previous = os.environ.get("SKYBUDDY_TELEMETRY_LOG")
    os.environ["SKYBUDDY_TELEMETRY_LOG"] = "0"
    check("SKYBUDDY_TELEMETRY_LOG=0 이면 기록하지 않음", stream_log_path() is None)
    os.environ["SKYBUDDY_TELEMETRY_LOG"] = previous

    print(" fault.py stream")
    cli("stream", "-n", "3", "--every", "2")
    out_csv = os.path.join(WORK, "stream.csv")
    cli("stream", "--csv", out_csv)
    with open(out_csv, encoding="utf-8") as f:
        header = f.readline().strip().split(",")
    check("stream --csv: 배터리·전압·링크 품질·비행 시간 열이 있음",
          {"battery_remaining_pct", "battery_voltage_v", "link_quality_pct", "sim_flight_s"} <= set(header),
          str(header))

    cfg = os.path.join(WORK, "drones.json")
    with open(cfg, "w", encoding="utf-8") as f:
        json.dump({"drones": [{"drone_id": "drone-07", "connection_string": "udpin:127.0.0.1:14999"}]}, f)
    os.environ["MIDDLEWARE_DRONES_CONFIG"] = cfg
    try:
        named = FaultInjector(["udpin:127.0.0.1:14999"], poll_s=0)
        check("미들웨어 드론 설정이 있으면 장애를 걸기 전부터 drone_id 이름으로 기록 (연결 문자열 대신)",
              named.name == "drone-07", named.name)
    finally:
        os.environ.pop("MIDDLEWARE_DRONES_CONFIG", None)
    clear_all()


# ====================================================================== 2) MAVLink

class FakeMsg:
    def __init__(self, mtype, **fields):
        self._type = mtype
        self.__dict__.update(fields)

    def get_type(self):
        return self._type

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


class FakeMav:
    """master.mav.*_send() 흉내. 실제 pymavlink 처럼 연결 객체의 write() 로 내보낸다."""

    def __init__(self, master):
        self._master = master

    def __getattr__(self, name):
        if not name.endswith("_send"):
            raise AttributeError(name)

        def send(*args):
            self._master.write((name, args))
        return send


class FakeMaster:
    """pymavlink mavudp 흉내. 0.05초마다 하트비트·배터리·GPS·위치 메시지를 낸다."""

    def __init__(self):
        self.mav = FakeMav(self)
        self.messages = {}
        self.target_system = 0
        self.target_component = 0
        self.received = []                   # write() 를 통과해 기체에 닿은 송신
        self.lat, self.lon, self.alt = HOME[0], HOME[1], 30.0
        self.land_at = None                  # 착륙 명령을 받은 시각. land_delay 뒤 시동이 꺼진다
        self.land_delay = 1.0
        self.landing_vz = 0                  # 착륙 중(시동 꺼지기 전) 보고할 하강 속도 cm/s (아래가 +)
        self._queue = collections.deque()
        self._next_at = 0.0

    def _produce(self):
        now = time.time()
        if now < self._next_at:
            return
        self._next_at = now + 0.05
        landing = self.land_at is not None
        landed = landing and now - self.land_at > self.land_delay
        vz = self.landing_vz if landing and not landed else 0
        self._queue.extend([
            FakeMsg("HEARTBEAT", base_mode=0 if landed else 128, custom_mode=9 if landed else 4),
            FakeMsg("SYS_STATUS", voltage_battery=12600, battery_remaining=100),
            FakeMsg("GPS_RAW_INT", fix_type=3, satellites_visible=10, eph=70),
            FakeMsg("GLOBAL_POSITION_INT", lat=int(self.lat * 1e7), lon=int(self.lon * 1e7),
                    relative_alt=int(self.alt * 1000), vx=0, vy=0, vz=vz, hdg=9000),
        ])

    def recv_msg(self):
        self._produce()
        if not self._queue:
            return None
        msg = self._queue.popleft()
        self.messages[msg.get_type()] = msg      # pymavlink 도 숨기기 전에 여기까지는 갱신한다
        return msg

    def recv_match(self, type=None, blocking=False, timeout=None):
        types = [type] if isinstance(type, str) else type
        deadline = None if timeout is None else time.time() + timeout
        while True:
            if deadline is not None and time.time() >= deadline:
                return None
            msg = self.recv_msg()        # 인스턴스 속성 -> 장애 주입이 감싼 함수 (pymavlink 와 같다)
            if msg is None:
                if not blocking:
                    return None
                time.sleep(0.005)
                continue
            if types and msg.get_type() not in types:
                continue
            return msg

    def wait_heartbeat(self, timeout=None, blocking=True):
        return self.recv_match(type="HEARTBEAT", blocking=blocking, timeout=timeout)

    def write(self, item):
        self.received.append(item)
        name, args = item
        if name == "set_position_target_global_int_send":
            self.lat, self.lon = args[5] / 1e7, args[6] / 1e7     # 바로 도착한 것으로
        if name == "command_long_send" and len(args) > 2 and args[2] == 21:   # MAV_CMD_NAV_LAND
            self.land_at = time.time()

    def mode_mapping(self):
        return {"STABILIZE": 0, "GUIDED": 4, "RTL": 6, "LAND": 9}

    def close(self):
        pass


def gotos(fake):
    return [a for name, a in fake.received if name == "set_position_target_global_int_send"]


def count_samples(drone, seconds, timeout=0.3):
    n, end = 0, time.time() + seconds
    while time.time() < end:
        if drone.get_telemetry(timeout=timeout) is not None:
            n += 1
    return n


def section_mavlink(mavlink_drone):
    print("\n=== 2) MavlinkDrone + 가짜 pymavlink 연결 ===")
    clear_all()
    fake = FakeMaster()
    mavlink_drone.mavutil.mavlink_connection = lambda *a, **k: fake
    d = mavlink_drone.MavlinkDrone(MAV_CS)          # 미들웨어 ClientFactory 와 같은 생성 방식
    d.connect(timeout=2)

    t = d.get_telemetry(timeout=1)
    check("정상: 샘플 수신, 배터리 100, 링크 품질 100",
          t is not None and t["battery_remaining_pct"] == 100 and t["link_quality_pct"] == 100.0, str(t))
    check("어댑터 연결이 기록됨 (fault.py status 의 '최근 연결된 어댑터')",
          any(MAV_CS in (e.get("identities") or []) for e in events("fault_adapter_attached")))

    cli("battery", "drone-01", "15")
    t = d.get_telemetry(timeout=1)
    check("battery 15 -> get_telemetry 의 battery_remaining_pct 가 15.0",
          t is not None and t["battery_remaining_pct"] == 15.0, str(t and t["battery_remaining_pct"]))
    check("전압도 잔량에 맞춰 바뀜 (15% = 셀당 3.71V x 3셀 = 11.13V), 다른 필드는 그대로",
          t is not None and t["battery_voltage_v"] == 11.13 and t["gps_fix_type"] == 3,
          str(t and t["battery_voltage_v"]))
    check("기록 이름이 drone-01 로 바뀜 (연결 문자열로 맞춘 항목에서 배움)", d.faults.name == "drone-01")

    cli("link", "drone-01", "down")
    started = time.time()
    t = d.get_telemetry(timeout=0.3)
    waited = time.time() - started
    check("down: get_telemetry 가 timeout 만큼 기다린 뒤 None", t is None and waited >= 0.25,
          f"{t}, {waited:.2f}s")
    n_before = len(gotos(fake))
    target = offset(HOME[0], HOME[1], 30)
    d.goto(target[0], target[1], 30)
    check("down: goto 가 기체에 닿지 않음", len(gotos(fake)) == n_before)
    check("down: 하트비트도 안 보임 (land/모드 전환도 실제 두절처럼 시간 초과)",
          d.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2) is None)
    status = d.fault_status()
    check("fault_status(): link down, battery 15",
          status["link"]["state"] == "down" and status["battery"]["battery_remaining_pct"] == 15.0,
          json.dumps(status, ensure_ascii=False))

    cli("clear", "drone-01", "--kind", "link")
    t = d.get_telemetry(timeout=1)
    check("복구: 샘플 다시 수신 (배터리 주입은 유지)",
          t is not None and t["battery_remaining_pct"] == 15.0, str(t))
    check("MAVLink 는 두절 중 보낸 goto 를 다시 보내지 않음 (기체 위치는 그대로)",
          abs(fake.lat - HOME[0]) < 1e-9)
    d.goto(target[0], target[1], 30)
    check("복구 후 goto 는 전달됨", len(gotos(fake)) == n_before + 1)
    downs = [e for e in events("fault_link_state") if e.get("state_from") == "down"]
    check("두절 중 전달 안 된 송신 수가 기록됨", bool(downs) and downs[-1].get("tx_dropped", 0) >= 1,
          str(downs[-1:] if downs else None))

    cli("stall", "drone-01")
    n_before = len(gotos(fake))
    d.goto(HOME[0], HOME[1], 30)
    first = len(gotos(fake))
    d.goto(HOME[0], HOME[1], 30)
    check("stall (기본 1회): 첫 goto 만 무시, 다음은 전달",
          (first, len(gotos(fake))) == (n_before, n_before + 1))

    cli("link", "drone-01", "down", "--failsafe-after", "0.5")
    end = time.time() + 1.2
    while time.time() < end:
        d.get_telemetry(timeout=0.2)     # 어댑터가 도는 동안 failsafe 시각이 된다
    modes = [a for name, a in fake.received if name == "set_mode_send"]
    check("down 0.5초 뒤 기체 failsafe: 두절을 우회해 RTL(6) 모드 전환이 기체에 닿음",
          bool(modes) and all(a[2] == 6 for a in modes), str(modes))
    n_before = len(gotos(fake))
    d.goto(target[0], target[1], 30)
    check("failsafe 뒤에도 일반 명령은 계속 막힘", len(gotos(fake)) == n_before)
    cli("clear", "drone-01", "--kind", "link")

    normal = count_samples(d, 1.0)
    cli("link", "drone-01", "degraded", "--interval", "0.5")
    degraded = count_samples(d, 2.0)
    check(f"degraded 0.5초: 2초에 샘플 3~5개 (정상은 1초에 {normal}개)", 3 <= degraded <= 5,
          f"{degraded}개")
    cli("clear")

    print(" 배터리 연속 감소 (시동 걸린 동안만) -> 0% 강제 착륙")
    modes_before = len([a for name, a in fake.received if name == "set_mode_send"])
    cli("battery", "drone-01", "0", "--drain", "1.5")
    values = []
    end = time.time() + 2.5
    while time.time() < end:
        t = d.get_telemetry(timeout=0.3)
        if t is not None:
            values.append((t["battery_remaining_pct"], t["battery_voltage_v"]))
    pcts = [p for p, _ in values]
    check("drain 1.5초: 샘플마다 줄어 0%까지 (중간값이 이어짐)",
          bool(pcts) and pcts[-1] == 0.0 and sum(0 < p < 100 for p in pcts) >= 3
          and all(a >= b for a, b in zip(pcts, pcts[1:])), str(pcts[:12]))
    check("전압도 함께 내려감 (0% = 셀당 3.27V x 3셀 = 9.81V)",
          bool(values) and values[-1][1] == 9.81, str(values[-1:]))
    new_modes = [a for name, a in fake.received if name == "set_mode_send"][modes_before:]
    check("0%: 그 자리 강제 착륙 LAND(9) 모드 전환이 기체에 닿음", any(a[2] == 9 for a in new_modes),
          str(new_modes))
    n_before = len(gotos(fake))
    d.goto(target[0], target[1], 30)
    check("0% 뒤 goto 는 기체에 보내지 않음", len(gotos(fake)) == n_before)
    try:
        d.takeoff(30)
        refused = False
    except RuntimeError as e:
        refused = "배터리" in str(e)
    check("0% 에서는 이륙(시동) 거부", refused)
    cli("clear")

    print(" 착륙 중 연결 상태 (K1)")
    samples = []
    original = d.get_telemetry

    def tapped(timeout=2):                   # 미들웨어 Registry 의 기록 장치와 같은 방식
        sample = original(timeout=timeout)
        if sample is not None:
            samples.append(sample)
        return sample

    d.get_telemetry = tapped
    check("land(): 시동 꺼짐을 확인하고 True", d.land(timeout=5) is True)
    check("land() 를 기다리는 동안에도 위치 샘플이 나감 (착륙 중 disconnected 오판 방지)",
          len(samples) >= 3, f"{len(samples)}개")

    print(" 홈보다 낮은 곳에 착륙 (2026-11-09 배터리 0% 강제 착륙: 60초 뒤에도 내려가는 중이라 미들웨어가 공중 disarm)")
    fake.land_at = None
    fake.land_delay, fake.landing_vz = 3.0, 40       # 기본 대기(1초)보다 오래, 0.4m/s 로 내려간 뒤 시동 꺼짐
    d.get_telemetry(timeout=1)
    started = time.time()
    landed = d.land(timeout=1)
    waited = time.time() - started
    check("기본 대기 시간이 지나도 내려가는 중이면 계속 기다려 착륙을 확인 (True)",
          landed is True and waited >= 2.5, f"{landed}, {waited:.1f}초")
    fake.land_at = None
    fake.land_delay, fake.landing_vz = 999.0, 0      # 내려가지도 않고 시동도 안 꺼짐
    d.get_telemetry(timeout=1)
    started = time.time()
    landed = d.land(timeout=1)
    waited = time.time() - started
    check("내려가지도 않고 시동도 안 꺼지면 기본 대기 시간에 끝냄 (False, 다음 조치는 호출 측)",
          landed is False and waited < 3.5, f"{landed}, {waited:.1f}초")
    fake.land_delay, fake.landing_vz = 1.0, 0
    fake.land_at = None                      # 다시 시동이 걸린 상태
    d.get_telemetry(timeout=1)               # 시동 걸린 하트비트를 받아 둔다
    cli("link", "drone-01", "down", "--failsafe", "none")
    fake.land_at = time.time() - 10          # 기체는 실제로 내려앉아 시동이 꺼졌지만
    check("두절 중 land(): 숨긴 하트비트가 캐시로 새지 않아 착륙을 확인하지 못함 (실제 두절과 같음)",
          d.land(timeout=2) is False)
    cli("clear")
    del d.get_telemetry
    d.close()


# ====================================================================== 3) AP_DDS

class FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.latitude)


class FakeClient:
    srv_name = "/ap/mode_switch"

    def __init__(self):
        self.calls = 0

    def wait_for_service(self, timeout_sec=None):
        self.calls += 1
        return False


def geopose(lat, lon, alt):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(latitude=lat, longitude=lon, altitude=alt),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def section_dds(ros2_drone, RosTime):
    print("\n=== 3) Ros2Drone + 가짜 ROS 메시지 ===")
    clear_all()
    d = ros2_drone.Ros2Drone("ros2:ap", drone_id="drone-02")
    d.faults.sleep = lambda s: None                   # _call 의 시간 초과 대기를 건너뛴다
    pub = FakePublisher()
    d._cmd_pub = pub
    d.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: RosTime())))
    d._home_alt_amsl = 583.0

    stop = threading.Event()
    # up: ENU 위쪽 속도(m/s, 내려가면 음수), land_delay: LAND 를 받고 시동이 꺼지기까지(초)
    vehicle = {"armed": True, "up": 0.0, "land_delay": 1.5}

    def feeder():                                     # ArduPilot 이 토픽을 계속 발행하는 것처럼
        while not stop.is_set():
            d._on_geopose(geopose(HOME[0], HOME[1], 613.0))
            d._on_twist(SimpleNamespace(twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=vehicle["up"]))))
            d._on_battery(SimpleNamespace(voltage=12.6, percentage=1.0))
            d._on_status(SimpleNamespace(armed=vehicle["armed"]))
            time.sleep(0.05)

    threading.Thread(target=feeder, daemon=True).start()
    try:
        t = d.get_telemetry(timeout=1)
        check("정상: 샘플 수신, 배터리 100", t is not None and t["battery_remaining_pct"] == 100.0, str(t))

        cli("battery", "drone-02", "25")
        t = d.get_telemetry(timeout=1)
        check("battery 25 -> battery_remaining_pct 25.0",
              t is not None and t["battery_remaining_pct"] == 25.0)

        cli("link", "drone-02", "down")
        time.sleep(0.1)                  # 설정 직전에 이미 처리 중이던 콜백이 끝나기를 기다린다
        check("down: get_telemetry 가 None (콜백이 메시지를 버림)", d.get_telemetry(timeout=0.3) is None)
        target = offset(HOME[0], HOME[1], 40)
        d.goto(target[0], target[1], 30)
        check("down: goto 를 발행하지 않음, 목표는 기억", not pub.sent and
              abs(d._target.latitude - target[0]) < 1e-9)
        d._republish_target()
        check("down: 재발행도 하지 않음", not pub.sent)
        client = FakeClient()
        try:
            d._call(client, None, timeout=0.01, what="mode_switch")
            raised = False
        except RuntimeError:
            raised = True
        check("down: 서비스 호출(모드 전환·시동 등)이 시간 초과로 실패", raised and client.calls == 0)

        cli("clear", "drone-02", "--kind", "link")
        check("복구: 샘플 다시 수신", d.get_telemetry(timeout=1) is not None)
        d._republish_target()
        check("AP_DDS 는 복구 후 재발행 타이머가 두절 중 받은 목표를 전달 (MAVLink 와 다름)",
              len(pub.sent) == 1 and abs(pub.sent[0] - target[0]) < 1e-9)

        cli("stall", "drone-02")
        d.goto(HOME[0], HOME[1], 30)
        check("stall: goto 무시, 이전 목표 유지",
              abs(d._target.latitude - target[0]) < 1e-9 and len(pub.sent) == 1)

        class FakeModeClient:
            """/ap/mode_switch 흉내. LAND 를 받으면 vehicle["land_delay"] 초 뒤 내려앉아 시동이 꺼진다."""
            srv_name = "/ap/mode_switch"

            def __init__(self):
                self.modes = []

            def wait_for_service(self, timeout_sec=None):
                return True

            def call_async(self, req):
                self.modes.append(req.mode)
                if req.mode == 9:
                    threading.Timer(vehicle["land_delay"], lambda: vehicle.update(armed=False)).start()
                response = SimpleNamespace(status=True, curr_mode=req.mode)
                return SimpleNamespace(done=lambda: True, result=lambda: response)

        d._cli_mode = FakeModeClient()
        cli("link", "drone-02", "down", "--failsafe-after", "0.3")
        time.sleep(0.8)                  # 토픽 콜백이 도는 동안 failsafe 시각이 된다
        check("down 0.3초 뒤 기체 failsafe: mode_switch(RTL=6)를 응답 기다리지 않고 보냄",
              d._cli_mode.modes[:1] == [6], str(d._cli_mode.modes))
        check("failsafe 뒤 GUIDED 목표 재발행을 멈춤", d._target is None)
        cli("clear", "drone-02", "--kind", "link")

        print(" 착륙 중 연결 상태 (K1)")
        samples = []
        original = d.get_telemetry

        def tapped(timeout=2):
            sample = original(timeout=timeout)
            if sample is not None:
                samples.append(sample)
            return sample

        d.get_telemetry = tapped
        landed = d.land(timeout=6)
        del d.get_telemetry
        check("AP_DDS land(): 시동 꺼짐을 확인하고 True", landed is True)
        check("AP_DDS land() 를 기다리는 동안에도 위치 샘플이 나감", len(samples) >= 3, f"{len(samples)}개")

        print(" 홈보다 낮은 곳에 착륙 (AP_DDS)")
        vehicle.update(armed=True, up=-0.4, land_delay=3.0)   # 0.4m/s 로 3초 내려간 뒤 시동 꺼짐
        time.sleep(0.3)
        started = time.time()
        landed = d.land(timeout=1)
        waited = time.time() - started
        check("AP_DDS: 기본 대기 시간이 지나도 내려가는 중이면 계속 기다려 착륙을 확인",
              landed is True and waited >= 2.5, f"{landed}, {waited:.1f}초")
        vehicle.update(up=0.0, land_delay=1.5)

        print(" 배터리 연속 감소 -> 0% 강제 착륙 (AP_DDS)")
        vehicle["armed"] = True          # 다시 이륙한 상태. 시동 여부는 /ap/status 콜백으로 들어온다
        time.sleep(0.3)
        d._cli_mode.modes.clear()
        cli("battery", "drone-02", "0", "--drain", "1.0")
        pcts = []
        end = time.time() + 1.8
        while time.time() < end:
            t = d.get_telemetry(timeout=0.3)
            if t is not None:
                pcts.append(t["battery_remaining_pct"])
        check("AP_DDS drain: 샘플마다 줄어 0%까지", bool(pcts) and pcts[-1] == 0.0
              and any(0 < p < 100 for p in pcts), str(pcts[:10]))
        check("AP_DDS 0%: 응답을 기다리지 않고 LAND(9) 모드 전환", 9 in d._cli_mode.modes,
              str(d._cli_mode.modes))
        d.goto(target[0], target[1], 30)
        check("AP_DDS 0% 뒤 goto 무시 (재발행할 목표도 없음)", d._target is None)
        try:
            d.takeoff(30)
            refused = False
        except RuntimeError as e:
            refused = "배터리" in str(e)
        check("AP_DDS 0% 에서는 이륙 거부", refused)
    finally:
        stop.set()
        cli("clear")


# ====================================================================== 4) 미들웨어

def find_middleware():
    for c in (os.environ.get("SKYBUDDY_MIDDLEWARE_DIR"),
              os.path.join(os.path.dirname(os.path.dirname(_HERE)), "middleware"),
              os.path.expanduser("~/skybuddy-team/services/middleware")):
        if c and os.path.exists(os.path.join(c, "app", "runtime", "registry.py")):
            return c
    return None


def section_middleware(mavlink_drone):
    print("\n=== 4) 미들웨어 Registry -> DroneState (실제 미들웨어 코드) ===")
    mw = find_middleware()
    if mw is None:
        skipped.append(("미들웨어 경로", "팀 저장소를 찾지 못함 (SKYBUDDY_MIDDLEWARE_DIR 로 지정 가능)"))
        print("  [건너뜀] 미들웨어 코드를 찾지 못함 (~/skybuddy-team/services/middleware)")
        return
    sys.path.append(mw)
    try:
        from app.runtime.registry import DroneRegistry, RegistryConfig
        from app.schemas import ConnectionStatus, DroneCommandType
    except Exception as e:  # noqa: BLE001 - 의존성 누락을 그대로 보여준다
        skipped.append(("미들웨어 경로", f"import 실패: {e}"))
        print(f"  [건너뜀] 미들웨어 import 실패: {e}")
        return
    print(f"  미들웨어: {mw}")
    clear_all()
    fake = FakeMaster()
    mavlink_drone.mavutil.mavlink_connection = lambda *a, **k: fake
    config = RegistryConfig.model_validate({
        "telemetry_timeout_s": 1.0,                  # 시험 시간을 줄이려고 1초 (기본 3초)
        "simulation_app_path": APP_DIR,              # 실제 ClientFactory 가 이 폴더의 어댑터를 불러온다
        "waypoints": {"generator": "vertex_traversal"},
        "drones": [{"drone_id": "drone-01", "protocol": "mavlink", "mode": "real",
                    "connection_string": MAV_CS}],
    })
    registry = DroneRegistry.from_config(config)
    errors = registry.connect_all()
    check("Registry 연결 (ClientFactory -> MavlinkDrone)", not errors, str(errors))
    try:
        s = registry.state("drone-01")
        check("정상: connected, battery_percent 100", s.connection_status == ConnectionStatus.CONNECTED
              and s.battery_percent == 100.0, f"{s.connection_status} {s.battery_percent}")
        check("RETURN_HOME 능력은 아직 광고하지 않음 (어댑터에 return_home 없음 -> goto(home) 유지)",
              DroneCommandType.RETURN_HOME not in s.capabilities.commands)

        cli("battery", "drone-01", "15")
        s = registry.state("drone-01")
        check("battery 15 -> DroneState.battery_percent 15.0, battery_voltage_v 11.13",
              s.battery_percent == 15.0 and s.battery_voltage_v == 11.13,
              f"{s.battery_percent} {s.battery_voltage_v}")
        check("새 필드 link_quality_pct 가 있어도 미들웨어 매퍼가 그대로 받음 (모르는 키는 무시)",
              s.connection_status == ConnectionStatus.CONNECTED)

        cli("link", "drone-01", "down")
        started, s = time.time(), None
        while time.time() - started < 4.0:
            s = registry.state("drone-01")
            if s.connection_status == ConnectionStatus.DISCONNECTED:
                break
        elapsed = time.time() - started
        check("down -> telemetry_timeout_s 뒤 connection_status disconnected",
              s.connection_status == ConnectionStatus.DISCONNECTED and elapsed >= 1.0,
              f"{s.connection_status} {elapsed:.1f}s")

        cli("clear", "drone-01", "--kind", "link")
        s = registry.state("drone-01")
        check("복구 -> 다시 connected", s.connection_status == ConnectionStatus.CONNECTED,
              str(s.connection_status))
        client = registry.get("drone-01").client
        status = client.fault_status()
        check("미들웨어가 client.fault_status() 로 주입 상태를 읽을 수 있음",
              status["drone"] == "drone-01" and status["battery"]["battery_remaining_pct"] == 15.0,
              json.dumps(status, ensure_ascii=False))
        section_link_loss(registry, fake)
    finally:
        cli("clear")
        registry.close_all()


def section_link_loss(registry, fake):
    """미들웨어 #42(통신 두절 처리)와 맞물리는지: LINK_LOST 판정, 기체 failsafe, 복구 대기."""
    print(" 미들웨어 #42 통신 두절 처리와 맞물림")
    try:
        from app.runtime import Dispatcher, DispatcherConfig, EventLog
    except ImportError as e:
        skipped.append(("미들웨어 #42", f"import 실패: {e}"))
        print(f"  [건너뜀] {e}")
        return
    if "link_loss_timeout_s" not in DispatcherConfig.model_fields or \
            not hasattr(Dispatcher, "_poll_arrival") or not hasattr(Dispatcher, "_wait_for_link"):
        skipped.append(("미들웨어 #42", "받아 둔 미들웨어가 #42 이전 버전 (팀 저장소 git pull 필요)"))
        print("  [건너뜀] 받아 둔 미들웨어에 #42 통신 두절 처리가 없다 (git pull 필요)")
        return
    dispatcher = Dispatcher(
        registry, EventLog(None),
        config=DispatcherConfig(link_loss_timeout_s=1.0, link_recovery_timeout_s=3.0),
        telemetry_poll_timeout_s=0.3)
    far = offset(HOME[0], HOME[1], 200)
    cli("link", "drone-01", "down", "--failsafe-after", "0.5")
    started = time.time()
    outcome, detail = dispatcher._poll_arrival("drone-01", far[0], far[1], timeout_s=10)
    elapsed = time.time() - started
    check("goto 중 두절 -> link_loss_timeout_s(시험값 1초) 뒤 link_lost",
          outcome == "link_lost" and 1.0 <= elapsed < 3.0, f"{outcome} {elapsed:.1f}s ({detail})")
    modes = [a for name, a in fake.received if name == "set_mode_send"]
    check("그 사이 기체 failsafe RTL 이 기체에 닿음 (#42 는 두절 중 명령을 안 보내고 이것을 기다림)",
          bool(modes) and modes[-1][2] == 6, str(modes))
    threading.Timer(1.0, lambda: cli("clear", "drone-01", "--kind", "link")).start()
    sample, outage = dispatcher._wait_for_link(registry.get("drone-01"))
    check("복구 대기 중 링크가 돌아오면 샘플을 받아 귀환 단계로 넘어감",
          sample is not None and outage < 3.0, f"sample={sample is not None}, {outage:.1f}s")


# ====================================================================== 5) 소스 규칙

def section_source():
    print("\n=== 5) 소스 규칙 ===")
    mav = open(os.path.join(APP_DIR, "mavlink_drone.py"), encoding="utf-8").read()
    ros = open(os.path.join(APP_DIR, "ros2_drone.py"), encoding="utf-8").read()
    check("어댑터가 get_telemetry 를 인스턴스에서 바꿔 끼우지 않음 (미들웨어 텔레메트리 기록 유지)",
          "self.get_telemetry =" not in mav and "self.get_telemetry =" not in ros)
    connect = mav[mav.index("def connect"):mav.index("def close")]
    check("MAVLink: 하트비트 대기 전에 장애 주입을 붙임",
          connect.index("self.faults.attach_mavlink") < connect.index("self.master.wait_heartbeat"))
    geo = ros[ros.index("def _on_geopose"):ros.index("def _on_twist")]
    check("AP_DDS: 두절 중에도 실제 위치를 장애 주입기에 알림 (range 복구 판단)",
          geo.index("observe_position") < geo.index("link_down"))
    stat = ros[ros.index("def _on_status"):ros.index("def _call")]
    check("AP_DDS: 두절 중에도 실제 시동 상태를 알림 (배터리는 링크와 상관없이 닳는다)",
          stat.index("observe_armed") < stat.index("link_down"))
    inj_src = open(os.path.join(APP_DIR, "fault_injection.py"), encoding="utf-8").read()
    hook = inj_src[inj_src.index("def attach_mavlink"):inj_src.index("def status")]
    check("MAVLink: 수신 훅이 하트비트로 실제 시동 상태를 알림 (두절 판단보다 먼저)",
          hook.index("observe_armed") < hook.index("if self.link_down()"))
    for name, src in (("mavlink_drone.py", mav), ("ros2_drone.py", ros)):
        takeoff = src[src.index("def takeoff"):src.index("def goto")]
        first_command = min(takeoff.index(c) for c in ("self.set_mode(", "self._call(", "self._set_mode(")
                            if c in takeoff)
        check(f"{name}: takeoff 가 기체에 무엇이든 보내기 전에 배터리 0% 를 확인 (시동 거부)",
              "self.faults.takeoff_block()" in takeoff
              and takeoff.index("self.faults.takeoff_block()") < first_command)
        tele = src[src.index("def get_telemetry"):]
        check(f"{name}: 텔레메트리에 link_quality_pct 키가 있음 (스키마 18필드)", '"link_quality_pct"' in tele)
    for name, src in (("mavlink_drone.py", mav), ("ros2_drone.py", ros)):
        land = src[src.index("def land"):src.index("def disarm")]
        check(f"{name}: land() 가 기다리는 동안 get_telemetry 로 위치를 읽음 (K1)",
              "self.get_telemetry(" in land)
        check(f"{name}: land() 가 내려가는 중이면 기본 대기 시간 뒤에도 기다림 (LandingWait)",
              "LandingWait(" in land and "wait.observe(self.get_telemetry(" in land)
        tele = src[src.index("def get_telemetry"):]
        check(f"{name}: get_telemetry 첫 동작이 장애 주입 게이트, 반환은 deliver()",
              tele.index("telemetry_gate") < tele.index("self._lock" if "ros2" in name else "self._drain()")
              and "self.faults.deliver(" in tele)


# ====================================================================== 실행

def main():
    original_connection = None
    try:
        section_rules()
        section_signals()

        try:
            import mavlink_drone
            original_connection = mavlink_drone.mavutil.mavlink_connection
        except ImportError as e:
            mavlink_drone = None
            skipped.append(("MavlinkDrone", f"{e} (pip install pymavlink)"))
            print(f"\n=== 2) 건너뜀: {e} ===")
        if mavlink_drone is not None:
            section_mavlink(mavlink_drone)

        try:
            import ros2_drone
            from builtin_interfaces.msg import Time as RosTime
        except ImportError as e:
            skipped.append(("Ros2Drone", f"{e} (source /opt/ros/humble/setup.bash && "
                                         f"source ~/ardu_ws/install/setup.bash)"))
            print(f"\n=== 3) 건너뜀: {e} ===")
        else:
            section_dds(ros2_drone, RosTime)

        if mavlink_drone is not None:
            section_middleware(mavlink_drone)
        section_source()
    finally:
        if original_connection is not None:
            import mavlink_drone
            mavlink_drone.mavutil.mavlink_connection = original_connection
        shutil.rmtree(WORK, ignore_errors=True)

    print()
    for name, why in skipped:
        print(f"건너뜀: {name} - {why}")
    print("전부 통과." if ok else "실패 항목이 있다. 위를 확인할 것.")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
