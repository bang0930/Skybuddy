"""기체 진동(자세 루프 발진)을 잡기 위한 감쇠 튜닝을 적용한다.

왜 필요한가:
  두 기체가 완전히 같은 파라미터를 쓰는데 한 대만 roll 축에서 지속 진동이 났다
  (vib_z 1.5 vs 0.05, 구간 이동 시간 35초 vs 7초). 파라미터·펌웨어·디렉토리는
  동일함을 diff로 확인했으므로, 차이는 그 기체가 AirSim에서 받는 센서 데이터의
  지연/지터다. 지연이 크면 같은 게인이라도 위상 여유가 모자라 발진한다.
  대응은 그 기체의 자세 루프 게인을 낮추고 필터를 세게 거는 것.

프리셋:
  default : ArduPilot 공식 AirSim 기본값으로 되돌린다
  damped  : 게인을 낮추고 필터 차단주파수를 내린 감쇠 설정
  soft    : damped 보다 더 보수적. damped 로도 안 잡힐 때.

사용법:
  python3 ~/skybuddy/tune_damping.py 14561 damped     # 드론1에 damped 적용
  python3 ~/skybuddy/tune_damping.py 14571 damped     # 드론2에 damped 적용
  python3 ~/skybuddy/tune_damping.py 14571 default    # 원복

적용 후 확인:
  python3 ~/skybuddy/watch_vibration.py 14571
"""
import sys
import time
from pymavlink import mavutil

PRESETS = {
    # ArduPilot 공식 airsim-quadX.parm (Copter-4.3) 값
    "default": {
        "ATC_RAT_RLL_P": 0.25,
        "ATC_RAT_RLL_I": 0.25,
        "ATC_RAT_RLL_D": 0.003,
        "ATC_RAT_PIT_P": 0.25,
        "ATC_RAT_PIT_I": 0.25,
        "ATC_RAT_PIT_D": 0.003,
        "ATC_RAT_RLL_FLTD": 50,
        "ATC_RAT_RLL_FLTT": 50,
        "ATC_RAT_PIT_FLTD": 50,
        "ATC_RAT_PIT_FLTT": 50,
        "ATC_ANG_RLL_P": 4.5,
        "ATC_ANG_PIT_P": 4.5,
    },
    # 게인 약 45% 감소 + 필터 차단주파수 50Hz -> 15Hz.
    # D항을 특히 많이 줄인다. D는 센서 지연에 가장 민감해서 발진의 주범이다.
    "damped": {
        "ATC_RAT_RLL_P": 0.135,
        "ATC_RAT_RLL_I": 0.135,
        "ATC_RAT_RLL_D": 0.0008,
        "ATC_RAT_PIT_P": 0.135,
        "ATC_RAT_PIT_I": 0.135,
        "ATC_RAT_PIT_D": 0.0008,
        "ATC_RAT_RLL_FLTD": 15,
        "ATC_RAT_RLL_FLTT": 15,
        "ATC_RAT_PIT_FLTD": 15,
        "ATC_RAT_PIT_FLTT": 15,
        "ATC_ANG_RLL_P": 3.0,
        "ATC_ANG_PIT_P": 3.0,
    },
    # damped 로도 안 잡힐 때. 반응은 더 느려지지만 발진 여유는 가장 크다.
    "soft": {
        "ATC_RAT_RLL_P": 0.08,
        "ATC_RAT_RLL_I": 0.08,
        "ATC_RAT_RLL_D": 0.0,
        "ATC_RAT_PIT_P": 0.08,
        "ATC_RAT_PIT_I": 0.08,
        "ATC_RAT_PIT_D": 0.0,
        "ATC_RAT_RLL_FLTD": 10,
        "ATC_RAT_RLL_FLTT": 10,
        "ATC_RAT_PIT_FLTD": 10,
        "ATC_RAT_PIT_FLTT": 10,
        "ATC_ANG_RLL_P": 2.0,
        "ATC_ANG_PIT_P": 2.0,
    },
}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]
    preset_name = sys.argv[2] if len(sys.argv) > 2 else "damped"

    if preset_name not in PRESETS:
        print(f"알 수 없는 프리셋: {preset_name}")
        print(f"사용 가능: {', '.join(PRESETS)}")
        sys.exit(1)

    params = PRESETS[preset_name]
    print(f"포트 {port} 에 '{preset_name}' 프리셋을 적용합니다 ({len(params)}개 항목)")

    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}')
    print("하트비트 대기 중...")
    master.wait_heartbeat()
    print(f"연결됨 (system={master.target_system})\n")

    failed = []
    for name, value in params.items():
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('ascii'), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        # 반영됐는지 실제로 읽어서 확인한다. 조용히 무시되는 걸 막기 위함.
        confirmed = None
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.param_id.strip('\x00') == name:
                confirmed = msg.param_value
                break

        if confirmed is None:
            print(f"  {name:20s} -> 확인 실패 (이 버전에 없는 파라미터일 수 있음)")
            failed.append(name)
        elif abs(confirmed - value) > 1e-4:
            print(f"  {name:20s} -> 요청 {value} / 실제 {confirmed}  <<< 불일치")
            failed.append(name)
        else:
            print(f"  {name:20s} = {confirmed}")

    print()
    if failed:
        print(f"[주의] {len(failed)}개 항목이 반영되지 않았습니다: {', '.join(failed)}")
    else:
        print("모든 항목 적용 확인됨.")
    print("\n효과 확인:")
    print(f"  python3 ~/skybuddy/watch_vibration.py {port}")
    master.close()


if __name__ == "__main__":
    main()
