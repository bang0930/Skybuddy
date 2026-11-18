"""ArduPilot 파라미터 파일(.parm)을 MAVLink 로 기체에 넣고, 실제로 바뀌었는지 읽어서 확인한다.

  python3 apply_params.py 14561 config_reference/mavlink_drone.parm      (개발 환경)
  python3 apply_params.py 14561 ../config/mavlink_drone.parm             (팀 저장소 scripts/ 에서)

start_hetero_sim.sh 가 drone1(ArduCopter 4.3, MAVLink) 에 쓴다. drone1 은 eeprom 이 남는 인스턴스라
sim_vehicle.py --add-param-file(기본값)로는 값이 바뀌지 않는다. drone2(4.7.1, DDS) 는 부팅마다 eeprom 을
지우므로 dds_drone.parm 을 --add-param-file 로 넣는다.

파일 형식은 sim_vehicle.py 와 같다: 한 줄에 "이름 값", # 뒤는 주석.
같은 포트를 미들웨어(MavlinkDrone)가 쓰고 있으면 붙지 못한다. 미들웨어를 띄우기 전에 실행한다.
"""
import sys
import time

from pymavlink import mavutil


def read_parm(path):
    """[(이름, 값)]. 주석·빈 줄은 건너뛴다."""
    params = []
    with open(path, encoding="utf-8") as f:
        for number, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 2:
                raise ValueError(f"{path}:{number}: '이름 값' 형식이 아니다: {raw.strip()}")
            params.append((parts[0], float(parts[1])))
    return params


def wait_value(master, name, wait_s=3.0):
    """기체가 보내는 PARAM_VALUE 중 name 의 값. 없으면 None."""
    deadline = time.time() + wait_s
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id.strip("\x00") == name:
            return msg.param_value
    return None


def apply(master, params, wait_s=3.0):
    """하나씩 지금 값을 읽고, 보내고, PARAM_VALUE 로 확인한다. 반영 안 된 이름 목록을 돌려준다."""
    failed = []
    for name, value in params:
        master.mav.param_request_read_send(master.target_system, master.target_component,
                                           name.encode("ascii"), -1)
        before = wait_value(master, name, wait_s)
        master.mav.param_set_send(master.target_system, master.target_component,
                                  name.encode("ascii"), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        confirmed = wait_value(master, name, wait_s)
        if confirmed is None:
            print(f"  {name:16s} -> 확인 실패 (이 버전에 없는 이름일 수 있다)")
            failed.append(name)
        elif abs(confirmed - value) > 1e-3 * max(1.0, abs(value)):
            print(f"  {name:16s} -> 요청 {value:g} / 실제 {confirmed:g}  <<< 불일치")
            failed.append(name)
        elif before is not None and abs(before - confirmed) > 1e-3 * max(1.0, abs(value)):
            print(f"  {name:16s} = {confirmed:g}  (전 {before:g})")
        else:
            print(f"  {name:16s} = {confirmed:g}")
    return failed


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__)
        return 2
    port, files = argv[0], argv[1:]
    try:
        params = [p for path in files for p in read_parm(path)]
    except (OSError, ValueError) as e:
        print(f"[오류] {e}")
        return 2
    if not params:
        print("넣을 파라미터가 없다")
        return 0
    print(f"포트 {port} 에 {len(params)}개 적용: {', '.join(files)}")
    master = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    try:
        if master.wait_heartbeat(timeout=30) is None:
            print("[오류] 30초 안에 하트비트가 없다 (SITL 이 떠 있는지, 미들웨어가 같은 포트를 쓰고 있지 않은지 확인)")
            return 1
        failed = apply(master, params)
    finally:
        master.close()
    if failed:
        print(f"[주의] {len(failed)}개 반영 안 됨: {', '.join(failed)}")
        return 1
    print("모두 적용 확인됨")
    return 0


if __name__ == "__main__":
    sys.exit(main())
