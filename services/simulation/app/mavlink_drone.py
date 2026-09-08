"""MAVLink로 통신하는 드론 어댑터.

DroneInterface 계약(drone_interface.py)의 MAVLink 구현체다.
같은 계약을 DDS로 구현한 것이 ros2_drone.py이며, 둘은 서로 바꿔 끼울 수 있다.

    from mavlink_drone import MavlinkDrone

    drone = MavlinkDrone("udpin:127.0.0.1:14561").connect()
    drone.takeoff(30)                       # 시동 + 이륙 + 고도 도달까지
    drone.goto(lat, lon, 30)                # 목표만 던지고 즉시 리턴
    while ...:                              # 도달 판정은 호출 측이 직접
        t = drone.get_telemetry()
    drone.land()
    drone.close()

전제:
    - ArduPilot SITL이 실행 중이고 MAVLink UDP 포트가 열려 있을 것
    - pymavlink 설치 (requirements.txt)

연결 문자열은 pymavlink 형식을 그대로 쓴다.
    udpin:127.0.0.1:14561    이 프로세스가 수신 대기 (SITL이 보내오는 쪽)
    udpout:127.0.0.1:14550   이 프로세스가 송신
"""
from pymavlink import mavutil
import time

from drone_interface import DroneInterface


class MavlinkDrone(DroneInterface):
    """ArduPilot과 MAVLink로 통신하는 어댑터.

    상위 계층(kpi_mission.py, 미들웨어)은 이 클래스를 직접 알 필요가 없다.
    DroneInterface의 7개 메서드만 호출하면 되고, 어느 프로토콜이 붙었는지는
    kpi_mission.make_drone() 팩토리만 안다.

    force_arm=True면 ArduPilot에 강제 시동 매직넘버(21196)를 함께 보낸다.
    AirSim의 시뮬레이션 GPS가 실제 prearm 기준을 통과하지 못하기 때문이며,
    실기체로 옮길 때는 반드시 꺼야 한다.
    """

    def __init__(self, connection_string, force_arm=True):
        self.connection_string = connection_string
        self.force_arm = force_arm
        self.master = None

    # ---------------------------------------------------------------- 연결

    def connect(self, timeout=30):
        """하트비트를 받아 상대 시스템 ID를 확정하고 self를 반환한다.

        MAVLink는 명령마다 대상 system/component ID가 필요한데, 이 값은
        하트비트를 한 번 받아봐야 알 수 있다. 그래서 연결 = 하트비트 대기다.
        """
        self.master = mavutil.mavlink_connection(self.connection_string)
        msg = self.master.wait_heartbeat(timeout=timeout)
        if msg is None:
            raise RuntimeError("하트비트 수신 실패")
        self.master.target_system = msg.get_srcSystem()
        self.master.target_component = msg.get_srcComponent()
        print(f"[연결됨] system={self.master.target_system} component={self.master.target_component}")
        return self

    def close(self):
        if self.master:
            self.master.close()

    # -------------------------------------------------------- 내부 헬퍼

    def set_mode(self, mode_name, timeout=10):
        """비행 모드를 바꾸고 하트비트로 실제 반영을 확인한다. 성공하면 True.

        모드 전환은 "요청"이라 거부될 수 있다. 특히 부팅 직후에는 EKF가 위치를
        확정하기 전이라 GUIDED 전환이 거부된다. 그래서 timeout 안에서 계속
        재요청하고, 끝내 안 되면 조용히 넘어가지 않고 False를 리턴한다.
        """
        mode_id = self.master.mode_mapping()[mode_name]
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb is not None and hb.custom_mode == mode_id:
                return True
        return False

    def arm(self):
        """모터 시동. 성공하면 True, 실패하면 이유를 출력하고 False.

        실패 시 STATUSTEXT에서 사유를 찾아 출력한다. ArduPilot은 시동 거부 이유를
        ACK가 아니라 별도 텍스트 메시지로 보내기 때문에, 이걸 안 읽으면
        "왜 안 되는지 모른 채 재시도만" 하게 된다.
        """
        param2 = 21196 if self.force_arm else 0   # ArduPilot 강제 시동 매직넘버
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, param2, 0, 0, 0, 0, 0
        )
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack is not None and ack.result == 0:
            return True

        reason = None
        deadline = time.time() + 1.0
        while time.time() < deadline:
            status = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
            if status is not None:
                reason = status.text
                break
        print(f"[ARM 실패] result={ack.result if ack else 'no ACK'} reason={reason}")
        return False

    # -------------------------------------------------------- 비행 명령

    def takeoff(self, altitude_m):
        """STABILIZE 리셋 → 시동 → GUIDED 전환 → 이륙. 실패하면 예외를 던진다.

        순서가 이렇게 복잡한 데는 각각 이유가 있다.

        1) STABILIZE로 리셋하는 이유
           LAND 모드에서는 시동이 거부된다("LAND mode not armable").
           이전 미션이 착륙까지 마쳤어도 모드는 LAND에 남아 있으므로 항상 리셋한다.

        2) 시동을 GUIDED 전환보다 먼저 하는 이유
           GUIDED 전환은 EKF 위치 확정을 기다려야 해서 최대 30초까지 걸린다.
           먼저 시동을 걸어두면 그 대기 시간 동안 모터 스풀업이 함께 진행된다.

        3) GUIDED 전환 후 1초 쉬는 이유
           하트비트가 GUIDED를 보고해도 ArduCopter 내부의 이착륙 서브모드가
           자리잡기까지 몇 스케줄러 틱이 더 필요하다. 이 간격이 너무 좁으면
           NAV_TAKEOFF가 "이륙 중"으로 등록되기 전에 스로틀만 올라가서
           update_land_detector()의 안전장치(PANIC: flow_of_ctrl)가 오작동한다.

        4) NAV_TAKEOFF를 재시도하는 이유
           ArduCopter의 do_user_takeoff()는 아래를 모두 만족해야 수락한다.
             - 모터 스풀 상태가 THROTTLE_UNLIMITED (시동 직후엔 GROUND_IDLE)
             - land_complete가 true (아직 지상에 있음)
             - position_ok() (EKF 위치 추정 확정)
           SITL 2기 + AirSim + DDS Agent + ROS 2가 CPU를 나눠 쓰면 스풀업이
           실제 시간으로 더 오래 걸려서, 한 번만 보내면 result=4(FAILED)로 거부된다.
        """
        self.set_mode('STABILIZE')
        if not self.arm():
            raise RuntimeError("ARM 실패")

        guided_wait_start = time.time()
        if not self.set_mode('GUIDED', timeout=30):
            raise RuntimeError("GUIDED 전환 실패 (포지션 추정이 아직 준비 안 됐을 수 있음)")
        print(f"[GUIDED 전환] {time.time() - guided_wait_start:.1f}초 소요")

        time.sleep(1.0)

        deadline = time.time() + 20
        last_result = None
        while time.time() < deadline:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, altitude_m
            )
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if ack is not None and ack.result == 0:
                return
            last_result = ack.result if ack is not None else 'no ACK'
            time.sleep(1.0)

        reason = None
        status = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if status is not None:
            reason = status.text
        raise RuntimeError(f"NAV_TAKEOFF 실패 (20초 재시도, result={last_result}, reason={reason})")

    def goto(self, lat, lon, alt_m):
        """목표 지점을 지정하고 즉시 리턴한다 (fire-and-forget).

        도달 여부는 호출 측이 get_telemetry()를 폴링해서 판단해야 한다.
        MAVLink에는 "도착했다"는 통보가 없기 때문이다.

        alt_m은 홈(이륙 지점) 기준 상대 고도다. 해수면 기준이 아니다.
        type_mask 0b0000111111111000은 "속도·가속도·yaw는 무시하고 위치만 쓴다"는 뜻이다
        (하위 3비트 = 위경도·고도 사용, 나머지 비트 = 해당 항목 무시).
        """
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), alt_m,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def land(self, timeout=60):
        """착륙 명령 후 시동이 실제로 꺼질 때까지 기다린다. 성공하면 True.

        명령만 보내고 리턴하면 다음 작업이 "아직 떠 있고 시동이 걸린" 기체 위에서
        시작해서 엉뚱하게 실패한다. 그래서 하트비트의 armed 플래그가 내려가는 것까지 확인한다.
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb is None:
                continue
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                return True
        return False

    def disarm(self, timeout=5):
        """착륙을 거치지 않고 곧바로 시동만 끈다 (안전장치). 성공하면 True.

        이착륙 시퀀스 도중 실패했을 때 쓴다. 모터가 시동 상태로 남으면 여러 대를
        운용할 때 다음 기체가 뜨는 동안 이 기체의 스로틀도 살아 있는 상태가 되어
        AirSim/ArduPilot 쪽 크래시를 유발할 수 있다.
        """
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196, 0, 0, 0, 0, 0
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb is None:
                continue
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                return True
        return False

    # -------------------------------------------------------- 텔레메트리

    def get_telemetry(self, timeout=2):
        """DroneInterface.TELEMETRY_FIELDS 스키마의 dict를 반환한다. 없으면 None.

        GLOBAL_POSITION_INT가 도착할 때까지 블로킹한다(약 4Hz). 호출 측이 while
        루프로 폴링하는 구조라, 논블로킹이면 CPU를 전부 먹고 로그가 폭증한다.

        GPS_RAW_INT / SYS_STATUS / VIBRATION은 각자 다른 주기로 흘러오는 별도
        스트림이다. 매번 새로 도착하지 않아도 pymavlink가 마지막 값을 캐시해두므로
        master.messages에서 꺼내 쓴다.

        빈 문자열("")은 "이번엔 값이 없다"는 뜻이다. 필드 자체는 항상 존재한다.
        """
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            return None

        gps = self.master.messages.get('GPS_RAW_INT')
        sys_status = self.master.messages.get('SYS_STATUS')
        # 진동은 눈에 보이는 "부들거림"을 숫자로 남기기 위한 항목이다.
        # ArduPilot 기준 30 이상이면 문제, clipping(가속도계 포화 횟수)은 0이어야 정상.
        # AP_DDS에는 이에 해당하는 토픽이 없어서 DDS 드론은 이 필드가 빈 값으로 나온다.
        vib = self.master.messages.get('VIBRATION')

        return {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "relative_alt_m": msg.relative_alt / 1000.0,
            "vx": msg.vx / 100.0,
            "vy": msg.vy / 100.0,
            "vz": msg.vz / 100.0,
            "heading_deg": (msg.hdg / 100.0) if msg.hdg != 65535 else "",   # 65535 = 값 없음
            "gps_fix_type": gps.fix_type if gps else "",
            "satellites_visible": gps.satellites_visible if gps else "",
            "gps_eph": (gps.eph / 100.0) if gps else "",
            "battery_voltage_v": (sys_status.voltage_battery / 1000.0)
                if sys_status and sys_status.voltage_battery != 65535 else "",
            "battery_remaining_pct": sys_status.battery_remaining
                if sys_status and sys_status.battery_remaining != -1 else "",
            "vibration_x": round(vib.vibration_x, 3) if vib else "",
            "vibration_y": round(vib.vibration_y, 3) if vib else "",
            "vibration_z": round(vib.vibration_z, 3) if vib else "",
            "clipping": (vib.clipping_0 + vib.clipping_1 + vib.clipping_2) if vib else "",
            "timestamp": time.time(),
        }


# 이전 이름. 파일명이 drone_link.py였을 때의 클래스 이름을 쓰는 코드를 위해 남겨둔다.
DroneLink = MavlinkDrone
