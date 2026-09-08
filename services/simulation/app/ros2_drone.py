"""ROS 2(DDS)로 말하는 드론 어댑터.

drone_link.py(MAVLink)와 완전히 동일한 인터페이스를 제공하므로,
kpi_mission.py 의 미션 로직은 어느 쪽이 붙었는지 알 필요가 없다.

전제:
  - ArduPilot 4.7.1 이상을 --enable-DDS 로 빌드해서 실행 중
  - micro-XRCE-DDS Agent 가 포트 2019 에서 실행 중
  - 실행 전 ROS 환경을 소싱할 것:
      source /opt/ros/humble/setup.bash
      source ~/ardu_ws/install/setup.bash

[반드시 지켜야 하는 3가지 - 전부 조용히 실패한다]
 1) 구독 QoS 는 BEST_EFFORT 여야 한다. ArduPilot 이 BEST_EFFORT 로 발행하는데
    rclpy 기본값은 RELIABLE 이라 매칭이 안 되고 메시지가 하나도 안 온다.
 2) /ap/cmd_gps_pose 는 header.frame_id 가 "map" 이어야 한다.
    AP_DDS_ExternalControl.cpp 의 핸들러 첫 줄이 frame_id 를 MAP_FRAME 과 비교하고,
    다르면 로그도 없이 return false 로 빠진다. (실측: 243회 발행했으나 무반응)
 3) 위치 명령만으로는 이륙하지 않는다. GUIDED 위치제어 루프가 매 주기
    is_disarmed_or_landed() 를 먼저 검사해서, 착륙 상태면 지상 대기 처리로 빠진다.
    반드시 /ap/experimental/takeoff 서비스를 써야 한다.
"""
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ardupilot_msgs.msg import GlobalPosition, Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix
from std_srvs.srv import Trigger

from drone_interface import DroneInterface

# ArduCopter 비행 모드 번호
COPTER_MODE_GUIDED = 4
COPTER_MODE_LAND = 9

# ardupilot_msgs/msg/GlobalPosition
FRAME_GLOBAL_REL_ALT = 6      # 홈 기준 상대 고도 -> Location::AltFrame::ABOVE_HOME
MAP_FRAME = "map"             # AP_DDS_Frames.h 의 MAP_FRAME 과 반드시 일치해야 함
# 속도/가속도/yaw 를 전부 무시하고 위치만 쓴다 (8+16+32+64+128+256+1024+2048)
TYPE_MASK_POSITION_ONLY = 3576

# 목표 재발행 주기. GUIDED 는 마지막 목표를 유지하지만, 수동 검증을 -r 2 로 했으므로
# 검증된 조건을 그대로 재현한다.
TARGET_REPUBLISH_HZ = 2.0

# 텔레메트리 최소 간격. DDS 원본은 약 28Hz 로 MAVLink(약 4Hz)보다 7배 빠르다.
# 그대로 받으면 (a) 두 드론의 CSV 행 수가 7배 차이나 비교가 어색해지고
# (b) SITL 2개 + AirSim + Agent + ROS 2 로 이미 빠듯한 CPU 를 더 먹는다.
# 그래서 MAVLink 쪽에 맞춰 샘플링한다. 원본 주기가 빠르다는 건 README 에 기록해뒀다.
DEFAULT_TELEMETRY_INTERVAL_S = 0.25

_rclpy_lock = threading.Lock()
_rclpy_users = 0


def _acquire_rclpy():
    """드론 2대를 스레드로 동시에 돌릴 때 rclpy.init() 이 두 번 불리면 안 된다."""
    global _rclpy_users
    with _rclpy_lock:
        if _rclpy_users == 0 and not rclpy.ok():
            rclpy.init()
        _rclpy_users += 1


def _release_rclpy():
    global _rclpy_users
    with _rclpy_lock:
        _rclpy_users -= 1
        if _rclpy_users <= 0:
            _rclpy_users = 0
            if rclpy.ok():
                rclpy.shutdown()


class Ros2Drone(DroneInterface):
    """AP_DDS 를 DroneInterface 계약에 맞춰 감싼 어댑터."""

    def __init__(self, connection_string="ros2:ap", drone_id="drone2",
                 telemetry_interval_s=DEFAULT_TELEMETRY_INTERVAL_S):
        # kpi_mission.py 는 connection_string 하나만 넘긴다. MAVLink 의
        # "udpin:127.0.0.1:14561" 자리에 "ros2:ap" 를 넣어 네임스페이스를 전달한다.
        self.connection_string = connection_string
        self.ns = connection_string.split(":", 1)[1] if ":" in connection_string else "ap"
        self.ns = self.ns.strip("/") or "ap"
        self.drone_id = drone_id
        self.telemetry_interval_s = telemetry_interval_s

        self.node = None
        self._executor = None
        self._spin_thread = None
        self._lock = threading.Lock()

        # 최신 수신값 캐시
        self._geopose = None
        self._geopose_seq = 0
        self._twist = None
        self._navsat = None
        self._battery = None
        self._status = None

        self._home_alt_amsl = None   # 이륙 전 지상 고도(AMSL). 상대고도 환산 기준.
        self._target = None          # 재발행할 현재 목표
        self._last_telemetry_at = 0.0

    # ---------------------------------------------------------------- 연결

    def connect(self, timeout=30):
        _acquire_rclpy()
        node_name = f"skybuddy_{self.drone_id}_{int(time.time() * 1000) % 100000}"
        self.node = Node(node_name)

        # ArduPilot 은 BEST_EFFORT 로 발행한다. RELIABLE 로 구독하면 매칭이 안 돼
        # 메시지가 하나도 안 온다 (에러도 안 난다).
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        n = self.ns

        self.node.create_subscription(GeoPoseStamped, f"/{n}/geopose/filtered",
                                      self._on_geopose, qos)
        self.node.create_subscription(TwistStamped, f"/{n}/twist/filtered",
                                      self._on_twist, qos)
        self.node.create_subscription(NavSatFix, f"/{n}/navsat/navsat0",
                                      self._on_navsat, qos)
        self.node.create_subscription(BatteryState, f"/{n}/battery",
                                      self._on_battery, qos)
        self.node.create_subscription(Status, f"/{n}/status",
                                      self._on_status, qos)

        self._cmd_pub = self.node.create_publisher(GlobalPosition, f"/{n}/cmd_gps_pose", 10)

        self._cli_mode = self.node.create_client(ModeSwitch, f"/{n}/mode_switch")
        self._cli_arm = self.node.create_client(ArmMotors, f"/{n}/arm_motors")
        self._cli_takeoff = self.node.create_client(Takeoff, f"/{n}/experimental/takeoff")
        self._cli_prearm = self.node.create_client(Trigger, f"/{n}/prearm_check")

        self.node.create_timer(1.0 / TARGET_REPUBLISH_HZ, self._republish_target)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        # 위치가 들어오기 시작할 때까지 대기
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._geopose is not None:
                break
            time.sleep(0.2)
        else:
            self.close()
            raise RuntimeError(f"{timeout}초 안에 /{n}/geopose/filtered 수신 실패 "
                               f"(Agent 와 SITL 이 떠 있는지 확인할 것)")

        # 이륙 전 AMSL 을 홈 고도로 잡는다. DDS geopose 는 AMSL 이고 MAVLink
        # relative_alt 는 홈 기준 상대고도라, 이 기준점이 없으면 두 드론의 고도를
        # 비교할 수 없다. (산악 지형에서는 이 차이가 수십 m 로 벌어진다)
        self._home_alt_amsl = self._geopose.pose.position.altitude
        print(f"[연결됨] ROS 2 namespace=/{self.ns} "
              f"home_alt_amsl={self._home_alt_amsl:.2f}m")
        return self

    def close(self):
        self._target = None
        if self._executor is not None:
            self._executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2)
            self._spin_thread = None
        _release_rclpy()

    # ------------------------------------------------------------ 콜백들

    def _on_geopose(self, msg):
        with self._lock:
            self._geopose = msg
            self._geopose_seq += 1

    def _on_twist(self, msg):
        with self._lock:
            self._twist = msg

    def _on_navsat(self, msg):
        with self._lock:
            self._navsat = msg

    def _on_battery(self, msg):
        with self._lock:
            self._battery = msg

    def _on_status(self, msg):
        with self._lock:
            self._status = msg

    # ------------------------------------------------------------ 서비스 호출

    def _call(self, client, request, timeout=10, what="service"):
        """executor 가 별도 스레드에서 돌고 있으므로 future 를 폴링해서 기다린다.
        여기서 spin_until_future_complete 를 쓰면 executor 와 충돌한다."""
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{what} 서비스를 찾을 수 없음 ({client.srv_name})")
        future = client.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)
        raise RuntimeError(f"{what} 응답 시간 초과")

    def _set_mode(self, mode):
        req = ModeSwitch.Request()
        req.mode = mode
        resp = self._call(self._cli_mode, req, what="mode_switch")
        return bool(resp.status) and resp.curr_mode == mode

    def _armed(self):
        with self._lock:
            return bool(self._status.armed) if self._status is not None else None

    # ------------------------------------------------------------ 비행 명령

    def takeoff(self, altitude_m):
        # prearm 은 실패해도 진행한다 - 이유를 로그로 남기는 게 목적이다.
        try:
            pre = self._call(self._cli_prearm, Trigger.Request(), what="prearm_check")
            if not pre.success:
                print(f"[prearm 경고] {pre.message}")
        except RuntimeError as e:
            print(f"[prearm 확인 불가] {e}")

        if not self._set_mode(COPTER_MODE_GUIDED):
            raise RuntimeError("GUIDED 전환 실패")

        req = ArmMotors.Request()
        req.arm = True
        if not self._call(self._cli_arm, req, what="arm_motors").result:
            raise RuntimeError("ARM 실패")

        req = Takeoff.Request()
        req.alt = float(altitude_m)
        if not self._call(self._cli_takeoff, req, what="takeoff").status:
            raise RuntimeError("이륙 명령 거부됨")

        # 목표 고도의 95% 도달까지 대기 (MAVLink 쪽 동작과 맞춤)
        deadline = time.time() + max(60, altitude_m * 4)
        while time.time() < deadline:
            t = self.get_telemetry(timeout=2)
            if t is not None and t["relative_alt_m"] >= altitude_m * 0.95:
                return True
        raise RuntimeError(f"이륙 후 고도 도달 실패 (목표 {altitude_m}m)")

    def goto(self, lat, lon, alt_m):
        msg = GlobalPosition()
        msg.header.frame_id = MAP_FRAME     # 이거 빠지면 조용히 무시된다
        msg.coordinate_frame = FRAME_GLOBAL_REL_ALT
        msg.type_mask = TYPE_MASK_POSITION_ONLY
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt_m)
        self._target = msg
        self._cmd_pub.publish(msg)          # 타이머를 기다리지 않고 즉시 한 번

    def _republish_target(self):
        if self._target is not None:
            self._target.header.stamp = self.node.get_clock().now().to_msg()
            self._cmd_pub.publish(self._target)

    def land(self, timeout=60):
        self._target = None                 # 목표 재발행을 멈춘다
        if not self._set_mode(COPTER_MODE_LAND):
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._armed() is False:
                return True
            time.sleep(0.5)
        return False

    def disarm(self, timeout=5):
        self._target = None
        try:
            req = ArmMotors.Request()
            req.arm = False
            self._call(self._cli_arm, req, timeout=timeout, what="arm_motors(disarm)")
        except RuntimeError:
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._armed() is False:
                return True
            time.sleep(0.3)
        return False

    # ------------------------------------------------------------ 텔레메트리

    def get_telemetry(self, timeout=2):
        # 새 위치 메시지가 올 때까지 기다린다. 논블로킹으로 두면 호출 측 while 루프가
        # CPU 를 전부 먹고 CSV 행이 수십만 개로 불어난다.
        with self._lock:
            start_seq = self._geopose_seq
        deadline = time.time() + timeout
        while time.time() < deadline:
            elapsed = time.time() - self._last_telemetry_at
            with self._lock:
                fresh = self._geopose_seq != start_seq
            if fresh and elapsed >= self.telemetry_interval_s:
                break
            time.sleep(0.02)
        else:
            return None

        self._last_telemetry_at = time.time()
        with self._lock:
            gp, tw, nav, bat = self._geopose, self._twist, self._navsat, self._battery

        if gp is None:
            return None

        pos = gp.pose.position
        rel_alt = pos.altitude - (self._home_alt_amsl or pos.altitude)

        # ENU -> NED. ROS(REP-103)는 x=동, y=북, z=위 / MAVLink 는 x=북, y=동, z=아래.
        if tw is not None:
            vx = tw.twist.linear.y
            vy = tw.twist.linear.x
            vz = -tw.twist.linear.z
        else:
            vx = vy = vz = ""

        # ENU yaw(동쪽 기준 반시계) -> 나침반 방위(진북 기준 시계)
        q = gp.pose.orientation
        yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        heading = (90.0 - math.degrees(yaw_enu)) % 360.0

        # NavSatFix.status: -1=NO_FIX, 0=FIX, 1=SBAS, 2=GBAS
        # MAVLink fix_type 스케일(3=3D Fix)로 옮긴다. kpi_mission 이 <3 을 저하로 본다.
        if nav is not None:
            gps_fix_type = 3 if nav.status.status >= 0 else 1
            # 공분산에서 수평 정확도(m)를 뽑는다. MAVLink eph(cm/100=m)와 같은 단위.
            eph = (round(math.sqrt(max(nav.position_covariance[0], 0.0)), 2)
                   if nav.position_covariance_type != 0 else "")
        else:
            gps_fix_type = ""
            eph = ""

        return {
            "lat": pos.latitude,
            "lon": pos.longitude,
            "relative_alt_m": round(rel_alt, 3),
            "vx": round(vx, 3) if vx != "" else "",
            "vy": round(vy, 3) if vy != "" else "",
            "vz": round(vz, 3) if vz != "" else "",
            "heading_deg": round(heading, 2),
            "gps_fix_type": gps_fix_type,
            # NavSatFix 에는 위성 수 필드가 없다. 프로토콜 간 기능 격차 - 숨기지 않는다.
            "satellites_visible": "",
            "gps_eph": eph,
            "battery_voltage_v": round(bat.voltage, 2) if bat is not None else "",
            "battery_remaining_pct": round(bat.percentage * 100, 1) if bat is not None else "",
            # AP_DDS 에는 진동/클리핑 토픽이 아예 없다. MAVLink 만 제공하는 지표.
            "vibration_x": "",
            "vibration_y": "",
            "vibration_z": "",
            "clipping": "",
            "timestamp": time.time(),
        }


if __name__ == "__main__":
    # 단독 동작 확인: 이륙 -> 북쪽 20m -> 복귀 -> 착륙
    d = Ros2Drone("ros2:ap", drone_id="dds_test").connect()
    try:
        home = d.get_telemetry()
        print(f"홈: {home['lat']:.6f}, {home['lon']:.6f}")
        d.takeoff(30)
        print("이륙 완료")

        target_lat = home["lat"] + 20 / 111320
        d.goto(target_lat, home["lon"], 30)
        deadline = time.time() + 60
        while time.time() < deadline:
            t = d.get_telemetry()
            if t is None:
                continue
            dist = d.distance_m(t["lat"], t["lon"], target_lat, home["lon"])
            print(f"  거리 {dist:5.1f}m  고도 {t['relative_alt_m']:5.1f}m  "
                  f"방위 {t['heading_deg']:5.1f}도")
            if dist < 3:
                print("도달")
                break

        d.goto(home["lat"], home["lon"], 30)
        time.sleep(20)
        print("착륙 확인됨" if d.land(timeout=120) else "[경고] 착륙 확인 실패")
    finally:
        d.close()
