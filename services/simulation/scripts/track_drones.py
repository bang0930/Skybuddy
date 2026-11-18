"""AirSim 화면에 두 드론의 위치를 드론마다 다른 색으로 표시한다 (영상·시연용). 기체에 명령은 보내지 않는다.

  python3 track_drones.py                          Ctrl+C 로 끝낼 때까지 표시
  python3 track_drones.py --duration 600           600초 뒤 스스로 끝냄
  python3 track_drones.py view --follow drone-02   B·F 시점이 따라갈 드론을 바꾼다 (AirSim 다시 켜야 적용)
  python3 track_drones.py view --follow drone-01 --subwindow drone-02
                                                   drone-01 을 따라가면서 drone-02 카메라를 작은 창에 띄운다
  python3 track_drones.py layout                   작은 창 1(왼쪽) = drone-01, 2(가운데) = drone-02 촬영 카메라,
                                                   3(오른쪽) = 숨김 (미니맵 자리). 카메라는 짐벌로 흔들림을 없앤다
                                                   (settings.json 수정·백업, AirSim 다시 켜야 적용)
  python3 track_drones.py layout --check           settings.json 이 이 배치인지 보기만 한다 (맞으면 0, 아니면 1)

작은 창 (layout)
  촬영 카메라 search_cam 을 두 기체에 새로 붙인다: 기체 앞쪽 조금 아래, 30도 숙임(--pitch), 640x360.
  짐벌(Gimbal Stabilization 1)이 카메라의 숙임·옆기울기를 세계 기준으로 고정해, 기체가 가감속으로 기울어도
  화면이 위아래로 흔들리지 않는다. 방향(요)은 기체를 따른다 (AirSim 1.8.1 APIPCamera::Tick 으로 확인).
  작은 창의 자리와 테두리 색은 AirSim 안에 고정돼 있다 (창 1 왼쪽 초록, 2 가운데 연노랑, 3 오른쪽 보라.
  settings.json 에는 어느 카메라를 띄울지만 있다). 그래서 드론 색을 테두리 색에 맞췄다: drone-01 = 초록(창 1),
  drone-02 = 연노랑(창 2). 미니맵 핀·지나간 길도 같은 색. 미니맵은 숨긴 창 3 자리(오른쪽 아래)에 뜬다.

표시 (drone-01 초록, drone-02 연노랑. 섹터 선은 북 빨강·남 파랑·동 주황·서 하늘이라 드론 색과 겹치지 않는다)
  지나간 길   가는 선으로 남는다 (--trail-thickness, 기본 6). 섹터 경계(map_probe.py draw)와 같은 좌표 변환이라
              겹쳐 보인다. map_probe.py draw 를 다시 하면 함께 지워진다 (run_live.sh 는 띄울 때마다 새로 그린다)
  기둥·이름표  기본은 안 그린다 (드론이 가려진다). --beacon 50 이면 드론 위로 솟은 기둥, --labels 면 이름표
              (BAT 배터리·LINK 링크 품질 = 미들웨어에 넘긴 텔레메트리 기록의 최신 값. 3초 넘게 없으면 "no data N s").
              배터리·링크는 미니맵(minimap.py) 아래 줄에서 본다

AirSim 시점 (AirSim 1.8.1 CameraDirector 소스·설정 문서로 확인)
  F(1인칭)·B(뒤따라가기)·\\(지상 관찰자)는 'FPV 기체' 한 대만 따라간다. 기본은 settings.json 의 첫 기체
  (Copter1 = drone-01)이고, 실행 중에 바꾸는 키는 없다. 기체 설정 IsFpvVehicle 로 정하고 AirSim 을 다시 켠다.
  M 은 수동 카메라 (화살표·W/A/S/D 로 이동). 두 드론을 한 화면에 보려면 M 으로 섹터 위 멀리에서 보면 된다.
  다른 드론의 카메라는 SubWindows 로 작은 창에 띄울 수 있다 (1·2·3 키로 켜고 끔).

위치는 simGetGroundTruthEnvironment 의 위경도(AirSim 이 ArduPilot 에 GPS 로 주는 값)를 원점 기준 NED 로 바꿔 쓴다.
simGetVehiclePose 는 기체마다 자기 출발점 기준이라 두 기체를 한 좌표에 그리려면 이쪽이 맞다.
"""
import argparse
import json
import math
import os
import shutil
import sys
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "fault_injection.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)
sys.path.insert(1, _HERE)

import map_probe as mp                              # noqa: E402  (Rpc, 설정 읽기, 좌표 변환)
from fault_injection import stream_log_path         # noqa: E402

# AirSim 작은 창 테두리 색과 같게 (창 1 = drone-01 초록, 창 2 = drone-02 연노랑. AirSim 안에 고정된 색).
# 미니맵 핀·지나간 길도 이 색을 쓴다
DRONE_COLORS = {"drone-01": [0.35, 0.87, 0.35, 1.0],   # 초록
                "drone-02": [0.97, 0.93, 0.60, 1.0]}   # 연노랑
OTHER_COLOR = [1.0, 1.0, 1.0, 1.0]
STALE_S = 3.0          # 미들웨어 telemetry_timeout_s 기본값과 같은 기준

SEARCH_CAMERA = "search_cam"       # 작은 창 촬영 카메라 이름 (settings.json Vehicles.<기체>.Cameras 에 만든다)
SEARCH_CAMERA_POSE = {"X": 0.30, "Y": 0.0, "Z": 0.10, "Roll": 0.0, "Yaw": 0.0}   # 기체 앞쪽 조금 아래 (m, NED)


def search_camera_setting(pitch=-30.0, gimbal=True, width=640, height=360):
    """작은 창에 띄울 촬영 카메라 설정.

    AirSim 1.8.1 은 Vehicles.<기체>.Cameras 에 적은 카메라를 새로 만들어 기체에 붙인다
    (PawnSimApi::createCamerasFromSettings). 그래서 위치·각도를 모두 적어야 한다 (빠지면 NaN 위치).
    짐벌은 숙임(Pitch)·옆기울기(Roll)를 세계 기준으로 고정한다 (APIPCamera::Tick). 요는 비워서 기체 방향을 따른다.
    """
    cam = {"CaptureSettings": [{"ImageType": 0, "Width": width, "Height": height, "FOV_Degrees": 90}],
           **SEARCH_CAMERA_POSE, "Pitch": float(pitch)}
    if gimbal:
        cam["Gimbal"] = {"Stabilization": 1, "Pitch": float(pitch), "Roll": 0.0}
    return cam


SPARE_CAMERA = "back_center"       # 숨긴 창 3 자리. 보이는 창과 다른 카메라여야 한다 (아무도 안 쓰는 뒤 카메라)


def layout_subwindows(vehicle1, vehicle2):
    """작은 창 1(왼쪽) = drone-01, 2(가운데) = drone-02 촬영 카메라, 3(오른쪽) = 숨김 (미니맵 자리).

    창마다 카메라가 달라야 한다. AirSim 1.8.1 SimHUD::updateWidgetSubwindowVisibility 는 창 1→2→3 순서로
    그 창 카메라의 촬영을 켜거나(보임) 끈다(숨김). 숨긴 창이 보이는 창과 같은 카메라면 그 촬영을 꺼서 보이는 창이
    멈춘다 (예전 배치에서 3 키로 창 3 을 켜야만 창 1 이 움직이던 증상). 그래서 숨긴 창 3 은 뒤 카메라를 가리킨다.
    """
    return [
        {"WindowID": 0, "CameraName": SEARCH_CAMERA, "ImageType": 0, "VehicleName": vehicle1, "Visible": True},
        {"WindowID": 1, "CameraName": SEARCH_CAMERA, "ImageType": 0, "VehicleName": vehicle2, "Visible": True},
        {"WindowID": 2, "CameraName": SPARE_CAMERA, "ImageType": 0, "VehicleName": vehicle1, "Visible": False},
    ]


def layout_problems(data, vehicle1, vehicle2):
    """settings.json 내용이 layout 배치와 다른 점 목록 (같으면 빈 목록). 카메라 각도는 따지지 않는다."""
    problems = []
    vehicles = data.get("Vehicles") or {}
    for name in (vehicle1, vehicle2):
        if SEARCH_CAMERA not in ((vehicles.get(name) or {}).get("Cameras") or {}):
            problems.append(f"{name} 에 촬영 카메라 {SEARCH_CAMERA} 가 없다")
    keys = ("CameraName", "ImageType", "VehicleName", "Visible")
    have = {w.get("WindowID"): w for w in (data.get("SubWindows") or [])}
    for want in layout_subwindows(vehicle1, vehicle2):
        got = have.get(want["WindowID"]) or {}
        if any(got.get(k) != want[k] for k in keys):
            problems.append(f"작은 창 {want['WindowID'] + 1}: 지금 "
                            + (", ".join(f"{k}={got.get(k)}" for k in keys) if got else "설정 없음")
                            + " / 맞는 값 " + ", ".join(f"{k}={want[k]}" for k in keys))
    return problems


def drone_mapping(vehicle_names, override=None):
    """AirSim 기체 이름 -> 드론 ID. 기본은 설정 순서대로 drone-01, drone-02 (start_hetero_sim.sh 구성)."""
    mapping = {name: f"drone-{i + 1:02d}" for i, name in enumerate(vehicle_names)}
    if override:
        for pair in override.split(","):
            if "=" not in pair:
                sys.exit(f"[오류] --map 형식은 Copter1=drone-01,Copter2=drone-02 이다: {pair}")
            vehicle, drone = (s.strip() for s in pair.split("=", 1))
            mapping[vehicle] = drone
    return mapping


def latest_samples(path, max_bytes=128 * 1024):
    """텔레메트리 기록 끝부분에서 드론별 마지막 기록 {drone: (기록 시각 epoch, telemetry)}."""
    out = {}
    if not path:
        return out
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            raw = f.read().decode("utf-8", errors="replace")
    except OSError:
        return out
    for line in reversed(raw.splitlines()):
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        name = rec.get("drone")
        if name in out:
            continue
        try:
            ts = datetime.fromisoformat(rec.get("logged_at")).timestamp()
        except (TypeError, ValueError):
            continue
        out[name] = (ts, rec.get("telemetry") or {})
    return out


def label_for(drone, sample, now):
    if sample is None:
        return drone
    ts, t = sample
    age = now - ts
    if age > STALE_S:
        return f"{drone}  no data {age:.0f}s"
    parts = [drone]
    if t.get("battery_remaining_pct") is not None:
        parts.append(f"BAT {t['battery_remaining_pct']:.0f}%")
    if t.get("link_quality_pct") is not None:
        parts.append(f"LINK {t['link_quality_pct']:.0f}%")
    return "  ".join(parts)


def run(args):
    origin, vehicles, settings_port, settings_path = mp.load_settings()
    port = args.port or settings_port
    try:
        rpc = mp.Rpc(args.host, port, timeout=10)
    except OSError as e:
        sys.exit(mp.CONNECT_HELP.format(host=args.host, port=port, err=e))
    try:
        listed = list(rpc.call("listVehicles") or [])
    except Exception as e:  # noqa: BLE001 - 목록이 없으면 설정 파일의 이름을 그대로 쓴다
        print(f"listVehicles 실패 (설정 파일의 이름을 쓴다): {e}")
        listed = []
    names = [n for n in vehicles if not listed or n in listed] or listed
    if not names:
        sys.exit("표시할 기체가 없다 (settings.json 의 Vehicles 와 AirSim listVehicles 가 비어 있음)")
    mapping = drone_mapping(names, args.map)
    stream = args.stream or stream_log_path()
    lat0, lon0, alt0 = origin
    print(f"AirSim {args.host}:{port} ({settings_path or '기본 설정'}), 원점 {lat0:.6f}, {lon0:.6f}, 해발 {alt0:g}m")
    for name in names:
        print(f"  {name} -> {mapping[name]} (색 {DRONE_COLORS.get(mapping[name], OTHER_COLOR)[:3]})")
    if args.labels:
        print(f"이름표 값: {stream or '없음 (SKYBUDDY_TELEMETRY_LOG=0)'}")
    print("표시 중 - Ctrl+C 로 끝낸다")

    def to_nedu(gp):
        lat, lon = gp["latitude"], gp["longitude"]
        north = (lat - lat0) * mp.M_PER_DEG
        east = (lon - lon0) * mp.M_PER_DEG * math.cos(math.radians(lat))
        return north, east, float(gp["altitude"]) - alt0

    last_point = {}
    samples, samples_at = {}, 0.0
    end = time.time() + args.duration if args.duration else None
    life = args.period * 1.6          # 기둥·이름표는 잠깐만 보이게 해서 드론을 따라 움직이게 한다
    try:
        while end is None or time.time() < end:
            now = time.time()
            if args.labels and now - samples_at >= 1.0:
                samples, samples_at = latest_samples(stream), now
            for name in names:
                drone = mapping[name]
                color = DRONE_COLORS.get(drone, OTHER_COLOR)
                env = rpc.call("simGetGroundTruthEnvironment", name)
                n, e, u = to_nedu(env["geo_point"])
                prev = last_point.get(name)
                if prev is None or math.dist(prev, (n, e, u)) >= args.min_step:
                    if prev is not None:
                        rpc.call("simPlotLineList", [mp.vec(*prev), mp.vec(n, e, u)], color,
                                 args.trail_thickness, -1.0, True)
                    last_point[name] = (n, e, u)
                if args.beacon > 0:
                    rpc.call("simPlotLineList", [mp.vec(n, e, u + 2.0), mp.vec(n, e, u + args.beacon)],
                             color, args.beacon_thickness, life, False)
                if args.labels:
                    rpc.call("simPlotStrings", [label_for(drone, samples.get(drone), now)],
                             [mp.vec(n, e, u + max(args.beacon, 0.0) + 4.0)], args.label_scale, color, life)
            time.sleep(max(0.0, args.period - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    except (ConnectionError, OSError, RuntimeError) as e:
        print(f"AirSim 연결이 끊겨 표시를 멈춘다: {e}")
        return 1
    print("표시를 멈췄다 (지나간 길은 map_probe.py draw 를 다시 하거나 draw --clear 로 지운다)")
    return 0


def load_windows_settings():
    """AirSim 이 실제로 읽는 Windows 쪽 settings.json: (경로, 전체, Vehicles)."""
    path = mp.find_settings()
    if not path or not path.startswith("/mnt/"):
        sys.exit("AirSim 이 읽는 Windows 쪽 settings.json 을 찾지 못했다 "
                 "(/mnt/c/Users/*/OneDrive/Documents/AirSim 또는 /mnt/c/Users/*/Documents/AirSim). "
                 "AIRSIM_SETTINGS=<경로> 로 지정할 것")
    with open(path, encoding="utf-8-sig") as f:          # Windows 메모장이 붙이는 BOM 도 읽는다
        data = json.load(f)
    vehicles = data.get("Vehicles") or {}
    if not vehicles:
        sys.exit(f"{path} 에 Vehicles 가 없다")
    return path, data, vehicles


def save_settings(path, data):
    """원본을 백업하고 바꾼다. 백업 경로를 돌려준다."""
    backup = f"{path}.bak_{datetime.now():%Y%m%d_%H%M%S}"
    shutil.copy2(path, backup)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="ascii") as f:          # AirSim 문서 권고: ASCII 로 저장
        json.dump(data, f, indent=2, ensure_ascii=True)
        f.write("\n")
    os.replace(tmp, path)
    return backup


def layout(args):
    """작은 창 1 = drone-01, 2 = drone-02 촬영 카메라 (짐벌로 흔들림 없음), 3 = 숨김(미니맵 자리). 원본은 백업한다.

    --check 면 바꾸지 않고 지금 settings.json 이 이 배치인지만 본다 (맞으면 0, 아니면 1).
    """
    path, data, vehicles = load_windows_settings()
    mapping = drone_mapping(list(vehicles), args.map)
    by_drone = {d: v for v, d in mapping.items()}
    v1, v2 = by_drone.get("drone-01"), by_drone.get("drone-02")
    if not v1 or not v2:
        sys.exit("[오류] drone-01·drone-02 에 해당하는 기체가 없다. 기체: "
                 + ", ".join(f"{v}({mapping[v]})" for v in vehicles))
    if args.check:
        problems = layout_problems(data, v1, v2)
        for p in problems:
            print(f"  다름: {p}")
        print(f"{path}: " + ("작은 창 배치 맞음" if not problems else "작은 창 배치가 다르다 (track_drones.py layout)"))
        return 0 if not problems else 1
    for name in (v1, v2):
        cameras = vehicles[name].setdefault("Cameras", {})
        cameras[SEARCH_CAMERA] = search_camera_setting(args.pitch, gimbal=not args.no_gimbal)
    data["SubWindows"] = layout_subwindows(v1, v2)
    backup = save_settings(path, data)
    print(f"바꿈: {path}")
    print(f"백업: {backup}")
    print(f"  작은 창 1 (왼쪽)  : {v1} (drone-01) 촬영 카메라 - 초록 테두리 = drone-01 색")
    print(f"  작은 창 2 (가운데): {v2} (drone-02) 촬영 카메라 - 연노랑 테두리 = drone-02 색")
    print(f"  작은 창 3 (오른쪽): 숨김 ({v1} {SPARE_CAMERA}) - 미니맵(minimap.py)이 그 자리에 뜬다")
    print(f"  카메라: {args.pitch:g}도 숙임, " + ("짐벌로 숙임·옆기울기 고정 (흔들림 없음)" if not args.no_gimbal
                                            else "짐벌 없음 (기체를 따라 기운다)"))
    print("AirSim 을 완전히 껐다 켜야 적용된다 (그다음 start_hetero_sim.sh 도 다시). 작은 창은 1·2 키로 켜고 끈다")
    print(f"되돌리기: cp '{backup}' '{path}'")
    return 0


def view(args):
    """settings.json 에서 시점이 따라갈 기체(IsFpvVehicle)와 작은 창(SubWindows)을 바꾼다. 원본은 백업한다."""
    path, data, vehicles = load_windows_settings()
    mapping = drone_mapping(list(vehicles), args.map)
    by_drone = {d: v for v, d in mapping.items()}

    def vehicle_of(key):
        name = by_drone.get(key) or (key if key in vehicles else None)
        if name is None:
            sys.exit(f"[오류] '{key}' 에 해당하는 기체가 없다. 기체: "
                     + ", ".join(f"{v}({mapping[v]})" for v in vehicles))
        return name

    follow = vehicle_of(args.follow)
    for name, setting in vehicles.items():
        setting["IsFpvVehicle"] = (name == follow)
    if args.subwindow:
        windows = [w for w in data.get("SubWindows", []) if w.get("WindowID") != 0]
        if args.subwindow != "none":
            windows.insert(0, {"WindowID": 0, "CameraName": "0", "ImageType": 0,
                               "VehicleName": vehicle_of(args.subwindow), "Visible": True})
        if windows:
            data["SubWindows"] = windows
        else:
            data.pop("SubWindows", None)
    backup = save_settings(path, data)
    print(f"바꿈: {path}")
    print(f"백업: {backup}")
    print(f"  B·F 시점이 따라갈 기체: {follow} ({mapping[follow]})")
    if args.subwindow and args.subwindow != "none":
        sub = vehicle_of(args.subwindow)
        print(f"  작은 창 1: {sub} ({mapping[sub]}) 앞 카메라 - AirSim 에서 1 키로 켜고 끔")
    elif args.subwindow == "none":
        print("  작은 창 1: 지움")
    print("AirSim 을 다시 켜야 적용된다 (그다음 start_hetero_sim.sh 도 다시)")
    print(f"되돌리기: cp '{backup}' '{path}'")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="track_drones.py", description="AirSim 화면에 드론 위치 표시",
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    parser.add_argument("--duration", type=float, default=0, help="이 시간(초) 뒤 끝냄. 0 이면 Ctrl+C 까지")
    # AirSim 은 SITL 과 lock-step 이라 API 호출·영구 선이 많을수록 화면이 끊긴다 (2026-11-17 렉).
    # 그래서 0.5초마다 읽고, 5m 움직일 때마다만 선을 잇는다 (예전 0.3초·2m 의 절반 이하 호출).
    parser.add_argument("--period", type=float, default=0.5, help="갱신 주기 초 (기본 0.5)")
    parser.add_argument("--beacon", type=float, default=0.0,
                        help="드론 위로 솟는 기둥 높이 m (기본 0 = 안 그림. 예전 모양은 50)")
    parser.add_argument("--beacon-thickness", type=float, default=10.0, help="기둥 두께 (기본 10)")
    parser.add_argument("--trail-thickness", type=float, default=6.0, help="지나간 길 두께 (기본 6. 예전 20)")
    parser.add_argument("--labels", action="store_true", help="드론 위에 이름표(배터리·링크) 글씨를 띄운다 (기본 안 띄움)")
    parser.add_argument("--label-scale", type=float, default=8.0, help="이름표 글자 크기 (기본 8)")
    parser.add_argument("--min-step", type=float, default=5.0, help="이만큼(m) 움직일 때마다 길을 잇는다 (기본 5)")
    parser.add_argument("--map", help="기체 이름 -> 드론 ID (기본: 설정 순서대로 drone-01, drone-02). "
                                      "예: Copter1=drone-01,Copter2=drone-02")
    parser.add_argument("--stream", help="이름표에 쓸 텔레메트리 기록 (기본: SKYBUDDY_TELEMETRY_LOG)")
    parser.add_argument("--host", default=mp.DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=None, help="AirSim API 포트 (기본: settings.json 의 ApiServerPort)")
    sub = parser.add_subparsers(dest="command")
    v = sub.add_parser("view", help="시점이 따라갈 드론·작은 창 바꾸기 (settings.json 수정, AirSim 재시작 필요)")
    v.add_argument("--follow", required=True, help="B·F 시점이 따라갈 드론 (drone-01 / drone-02 또는 기체 이름)")
    v.add_argument("--subwindow", help="작은 창에 앞 카메라를 띄울 드론. none 이면 작은 창 설정을 지운다")
    lay = sub.add_parser("layout", help="작은 창 1=drone-01, 2=drone-02 촬영 카메라(흔들림 없음), 3=숨김(미니맵 자리) "
                                        "(settings.json 수정, AirSim 재시작 필요)")
    lay.add_argument("--pitch", type=float, default=-30.0, help="카메라를 아래로 숙인 각도 (기본 -30. -90 이면 바로 아래)")
    lay.add_argument("--no-gimbal", action="store_true", help="짐벌을 끈다 (기체가 기울면 화면도 같이 기운다)")
    lay.add_argument("--check", action="store_true", help="바꾸지 않고 지금 settings.json 이 이 배치인지만 본다 (맞으면 0)")
    args = parser.parse_args(argv)
    if args.command == "view":
        view(args)
        return 0
    if args.command == "layout":
        return layout(args)
    if args.period <= 0:
        parser.error("--period 는 0보다 커야 한다")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
