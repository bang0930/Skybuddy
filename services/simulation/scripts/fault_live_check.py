"""실제 SITL 연결에서 장애 주입이 동작하는지 확인한다. 비행하지 않는다 (명령을 하나도 보내지 않음).

verify_faults.py 는 가짜 연결로 검사했다. 여기서는 같은 항목을 진짜 pymavlink 연결과
진짜 ROS 2 토픽으로 확인한다. 텔레메트리만 읽고 시동·이륙·이동 명령은 보내지 않는다.

전제
  - AirSim + start_hetero_sim.sh 로 SITL 2대가 떠 있을 것 (드론은 지상)
  - 미들웨어 서버는 꺼 둘 것 (UDP 14561 을 둘이 같이 쓸 수 없다)
  - 시험용 제어 파일을 따로 쓰므로 /tmp/skybuddy_faults.json 은 건드리지 않는다

  source /opt/ros/humble/setup.bash
  source ~/ardu_ws/install/setup.bash
  python3 fault_live_check.py          (약 40초)
"""
import os
import shutil
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
APP_DIR = _HERE if os.path.exists(os.path.join(_HERE, "kpi_mission.py")) \
          else os.path.join(os.path.dirname(_HERE), "app")
sys.path.insert(0, APP_DIR)
SCRIPTS_DIR = _HERE if os.path.exists(os.path.join(_HERE, "fault.py")) \
              else os.path.join(os.path.dirname(_HERE), "scripts")
sys.path.insert(1, SCRIPTS_DIR)

WORK = tempfile.mkdtemp(prefix="skybuddy_fault_live_")
os.environ["SKYBUDDY_FAULT_FILE"] = os.path.join(WORK, "faults.json")
os.environ["SKYBUDDY_FAULT_LOG"] = os.path.join(WORK, "events.jsonl")
os.environ["SKYBUDDY_FAULT_POLL_S"] = "0"
os.environ.pop("SKYBUDDY_FAULT_INJECTION", None)
os.environ.pop("MIDDLEWARE_DRONES_CONFIG", None)

import fault  # noqa: E402

ok = True


def check(label, condition, detail=""):
    global ok
    mark = "OK  " if condition else "실패"
    print(f"  [{mark}] {label}" + (f"  -> {detail}" if detail and not condition else ""))
    if not condition:
        ok = False


def cli(*argv):
    fault.main(list(argv))


def count_samples(drone, seconds, timeout=0.5):
    n, end = 0, time.time() + seconds
    while time.time() < end:
        if drone.get_telemetry(timeout=timeout) is not None:
            n += 1
    return n


def exercise(name, drone, hidden_check=None, raw_battery=None):
    """raw_battery: 장애 주입을 거치지 않은 지금 배터리 값을 돌려주는 함수 (해제 확인용)."""
    t = drone.get_telemetry(timeout=3)
    check("실제 텔레메트리 수신", t is not None, "SITL 준비가 끝났는지 확인")
    if t is None:
        return
    real_battery = t["battery_remaining_pct"]
    print(f"    실제 값: 배터리 {real_battery}, 위치 {t['lat']:.6f}, {t['lon']:.6f}, "
          f"고도 {t['relative_alt_m']}m")
    normal = count_samples(drone, 2.0)

    cli("battery", name, "20")
    t = drone.get_telemetry(timeout=3)
    check("battery 20 -> 실제 연결에서도 battery_remaining_pct 20.0",
          t is not None and t["battery_remaining_pct"] == 20.0, str(t and t["battery_remaining_pct"]))

    cli("link", name, "down", "--failsafe", "none")    # 이 검사는 명령을 하나도 보내지 않는다
    time.sleep(0.3)
    t = drone.get_telemetry(timeout=1.5)
    check("down -> 실제 메시지가 계속 와도 get_telemetry 는 None", t is None, str(t))
    if hidden_check is not None:
        hidden_check()
    check("fault_status() 가 down", drone.fault_status()["link"]["state"] == "down")

    cli("clear", name, "--kind", "link")
    t = drone.get_telemetry(timeout=3)
    check("복구 -> 다시 수신 (배터리 주입은 유지)",
          t is not None and t["battery_remaining_pct"] == 20.0, str(t))

    cli("link", name, "degraded", "--interval", "1.0")
    degraded = count_samples(drone, 3.0)
    check(f"degraded 1초 -> 3초에 샘플 2~4개 (정상은 2초에 {normal}개)", 2 <= degraded <= 4,
          f"{degraded}개")

    cli("clear", name)
    t = drone.get_telemetry(timeout=3)
    # 처음 읽을 때는 배터리 토픽이 아직 안 왔을 수 있다(AP_DDS 실측: 첫 샘플 None, 나중에 100).
    # 그래서 '처음 값'이 아니라 지금 기체가 보내는 원래 값과 비교한다.
    real_now = raw_battery() if raw_battery is not None else real_battery
    check("해제 -> 주입값이 아니라 기체가 보내는 원래 배터리 값",
          t is not None and t["battery_remaining_pct"] == real_now,
          f"{t and t['battery_remaining_pct']} (원래 값 {real_now})")


def main():
    try:
        from mavlink_drone import MavlinkDrone
        print("=== MAVLink (drone-01, udpin:127.0.0.1:14561) ===")
        try:
            mav = MavlinkDrone("udpin:127.0.0.1:14561", drone_id="drone-01").connect(timeout=15)
        except OSError as e:
            check("연결", False, f"포트를 쓰는 프로그램이 있다 - 미들웨어 서버를 끄고 다시: {e}")
        except RuntimeError as e:
            check("연결", False, f"하트비트가 안 온다 - start_hetero_sim.sh 가 끝났는지 확인: {e}")
        else:
            def mav_hidden():
                hb = mav.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.5)
                check("down -> 실제 하트비트도 어댑터에 안 보임 (진짜 pymavlink 연결 객체)", hb is None)

            def mav_battery():
                ss = mav.master.messages.get("SYS_STATUS")
                return ss.battery_remaining if ss is not None and ss.battery_remaining != -1 else None
            try:
                exercise("drone-01", mav, mav_hidden, mav_battery)
            finally:
                mav.close()

        try:
            from ros2_drone import Ros2Drone
        except ImportError as e:
            print(f"\n=== AP_DDS 건너뜀: {e} (ROS 2 source 2줄 먼저) ===")
        else:
            print("\n=== AP_DDS (drone-02, ros2:ap) ===")
            dds = Ros2Drone("ros2:ap", drone_id="drone-02")
            try:
                dds.connect(timeout=20)
            except RuntimeError as e:
                check("연결", False, str(e))
            else:
                def dds_battery():
                    with dds._lock:
                        bat = dds._battery
                    return round(bat.percentage * 100, 1) if bat is not None else None
                exercise("drone-02", dds, raw_battery=dds_battery)
            finally:
                dds.close()
    finally:
        shutil.rmtree(WORK, ignore_errors=True)

    print("\n" + ("전부 통과." if ok else "실패 항목이 있다. 위를 확인할 것."))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
