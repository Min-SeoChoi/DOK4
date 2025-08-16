#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import re
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode, VehicleCommand, VehicleStatus,
    TrajectorySetpoint, VehicleAttitudeSetpoint,
    VehicleLocalPosition, VehicleAttitude, VehicleGlobalPosition,
    VehicleCommandAck
)

EARTH_RADIUS = 6371000.0  # meters

# === 각도 래핑 유틸 (yaw 틀어짐 방지) ===
def wrap_rad(a):
    return (a + np.pi) % (2*np.pi) - np.pi  # [-pi, pi]

def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0      # [-180, 180]

# ───────────── 여기를 너 좌표로 수정 ─────────────
ALTITUDE_MODE = "ned"   # "ned" 또는 "amsl"
WAYPOINTS_INLINE = [
    {"name":"WP0","lat":35.051662,"lon":128.039209,"alt": -0.0},    # 이륙 기준점
    {"name":"WP1","lat":35.051674,"lon":128.038923,"alt": -35.0},
    {"name":"WP2","lat":35.051845,"lon":128.037568,"alt": -35.0},
    {"name":"WP3","lat":35.053682,"lon":128.035687,"alt": -50.0},
    {"name":"WP4","lat":35.050028,"lon":128.034947,"alt": -50.0},
    {"name":"WP5","lat":35.050692,"lon":128.037352,"alt": -35.0},
    {"name":"WP6","lat":35.051808,"lon":128.039227,"alt": -35.0},   # 조난자 상공
    {"name":"WP7","lat":35.051551,"lon":128.039171,"alt": -0.0},    # 조난자 하기 지점
]

# === 이륙/도착/호버 설정 ===
TAKEOFF_ALTITUDE_NED = -35.0  # 제자리 수직 상승 목표(NED z, 음수 = 위로 상승)
THRESHOLD_RANGE      = 5.0    # 웨이포인트 도착 판정 반경 (m)
XY_ACCEPT = THRESHOLD_RANGE
Z_ACCEPT  = THRESHOLD_RANGE

ALT_REACHED_EPS   = 1.0       # 이륙 고도 도달 허용오차 [m]
TAKEOFF_HOLD_SEC  = 2.0       # 이륙 고도 도달 후 호버 유지 시간 [s]

# ★ 새로 추가: 웨이포인트 도착 시 동작
WP_HOVER_SEC      = 5.0       # 각 웨이포인트에서 5초 호버
ALIGN_HOLD_SEC    = 1.0       # 기수 정렬 시간(제자리에서 yaw 맞추는 시간)
# ────────────────────────────────────────────────

class OffboardControl(Node):
    def __init__(self):
        super().__init__("Offboard_control")

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Topics
        self.pub_ob_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_att_sp  = self.create_publisher(VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint", qos)
        self.pub_traj_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd     = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)

        self.sub_lpos    = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.cb_lpos, qos)
        self.sub_att     = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.cb_att, qos)
        # status_v1 과 status 둘 다 구독 (환경 차이 대비)
        self.sub_status1 = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.cb_status, qos)
        self.sub_status2 = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.cb_status, qos)
        self.sub_gpos    = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.cb_gpos, qos)
        # 명령 ACK 확인
        self.sub_ack     = self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.cb_ack, qos)

        # Waypoints 준비 (AMSL -> NED 변환 옵션)
        self.wgs84_wps = self._prepare_wps(WAYPOINTS_INLINE, ALTITUDE_MODE)

        # 기준점 설정 (WP0 우선)
        ref = self.wgs84_wps["WP0"] if "WP0" in self.wgs84_wps else self.wgs84_wps[sorted(self.wgs84_wps.keys())[0]]
        self.ref_lat = math.radians(ref["lat"])
        self.ref_lon = math.radians(ref["lon"])
        self.ref_sin = math.sin(self.ref_lat)
        self.ref_cos = math.cos(self.ref_lat)

        # WGS84 -> NED 변환 테이블 생성
        self.ned_wps = {}
        for name, wp in self.wgs84_wps.items():
            x, y = self._wgs84_to_ned(wp["lat"], wp["lon"])
            self.ned_wps[name] = {"x":x, "y":y, "z":wp["alt"]}

        self.get_logger().info(f"NED WPs: {self.ned_wps}")

        # === 초기 yaw(첫 구간: WP0→WP1) 계산 ===
        names_sorted = self._sorted_wp_names()
        self.names_sorted = names_sorted
        self.init_yaw_deg = self._yaw_deg_to(self.ned_wps[names_sorted[0]], self.ned_wps[names_sorted[1]])
        self.get_logger().info(f"초기 yaw(deg): {self.init_yaw_deg:.1f}")

        # 상태 변수
        self.state = "NOT_READY"
        self.offboard_setpoint_counter = 0  # 워밍업/시퀀스용
        self.segment_idx = 1  # 현재 타깃 웨이포인트 인덱스 (WP0->WP1부터)

        # 이륙 고도 도달 후 호버 유지 카운터
        self.takeoff_hold_ticks = 0

        # 웨이포인트 도착 후 5초 호버/정렬용
        self.wp_hold_ticks   = 0
        self.align_hold_ticks = 0
        self.current_wp_name = None  # 방금 도착한 웨이포인트 이름

        # 최신 수신값 보관
        self.lpos = VehicleLocalPosition()
        self.att  = VehicleAttitude()
        self.stat = VehicleStatus()
        self.gpos = VehicleGlobalPosition()

        # 타이머
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.on_timer)

    # ───────────── Waypoint 준비 ─────────────
    def _prepare_wps(self, lst, altitude_mode):
        d = {}
        for item in lst:
            alt = float(item["alt"])
            if altitude_mode.lower() == "amsl":
                alt = -alt  # AMSL(+위) → NED z(아래가 +)
            d[item["name"]] = {"lat": float(item["lat"]), "lon": float(item["lon"]), "alt": alt}
        return d

    # ───────────── 콜백 ─────────────
    def cb_lpos(self, msg): self.lpos = msg
    def cb_att (self, msg): self.att  = msg
    def cb_status(self, msg): self.stat = msg
    def cb_gpos(self, msg): self.gpos = msg
    def cb_ack(self, msg: VehicleCommandAck):
        self.get_logger().info(f"ACK cmd={int(msg.command)} result={int(msg.result)} "
                               f"sys={msg.target_system} comp={msg.target_component}")

    # ───────────── 유틸 ─────────────
    def _wgs84_to_ned(self, lat_deg, lon_deg):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        s = math.sin(lat); c = math.cos(lat)
        cdl = math.cos(lon - self.ref_lon)
        arg = self.ref_sin*s + self.ref_cos*c*cdl
        arg = max(min(arg, 1.0), -1.0)
        cang = math.acos(arg)
        k = cang / math.sin(cang) if abs(cang) > 1e-6 else 1.0
        x = k * (self.ref_cos*s - self.ref_sin*c*cdl) * EARTH_RADIUS
        y = k * c * math.sin(lon - self.ref_lon) * EARTH_RADIUS
        return x, y

    def _dist_ned(self, x1,y1,z1, x2,y2,z2):
        dx, dy, dz = x2-x1, y2-y1, z2-z1
        dxy = math.hypot(dx, dy)
        return math.hypot(dxy, abs(dz)), dxy, abs(dz)

    def _reached(self, wp):
        _, dxy, dz = self._dist_ned(self.lpos.x, self.lpos.y, self.lpos.z, wp["x"], wp["y"], wp["z"])
        return (dxy <= XY_ACCEPT) and (dz <= Z_ACCEPT)

    def _yaw_deg_to(self, a, b):
        dx, dy = b["x"]-a["x"], b["y"]-a["y"]
        return math.degrees(math.atan2(dy, dx))

    def _sorted_wp_names(self):
        # "WP12" → 12 기준 정렬, 아니면 0
        def key_func(name):
            m = re.search(r'(\d+)$', name)
            return int(m.group(1)) if m else 0
        return sorted(self.ned_wps.keys(), key=key_func)

    # ───────────── 명령 ─────────────
    def _pub_ob_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.pub_ob_mode.publish(msg)

    def _cmd(self, command, **kw):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kw.get("param1", 0.0)
        msg.param2 = kw.get("param2", 0.0)
        msg.param3 = kw.get("param3", 0.0)
        msg.param4 = kw.get("param4", 0.0)
        msg.param5 = kw.get("param5", 0.0)
        msg.param6 = kw.get("param6", 0.0)
        msg.param7 = kw.get("param7", 0.0)
        msg.target_system = 1; msg.target_component = 1
        msg.source_system = 1; msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.pub_cmd.publish(msg)

    def _arm(self): self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    def _offboard(self): self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    def _takeoff(self, alt=50.0): self._cmd(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=float(alt))  # 옵션
    def _land(self): self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _pub_pos_sp(self, x,y,z, yaw_deg):
        if self.stat.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self._offboard()
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [np.nan, np.nan, np.nan]
        msg.acceleration = [np.nan, np.nan, np.nan]
        # yaw는 wrap해서 전송 (clip 금지)
        msg.yaw = float(wrap_rad(np.deg2rad(wrap_deg(yaw_deg))))
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.pub_traj_sp.publish(msg)

    # ───────────── 타이머(FSM) ─────────────
    def on_timer(self):
        self._pub_ob_heartbeat()

        names = self.names_sorted
        if len(names) < 2:
            self.get_logger().error("웨이포인트가 2개 미만입니다.")
            return

        if self.state == "NOT_READY":
            # 1) 워밍업: 현재 위치 유지 + 초기 yaw(첫 구간 방향)
            self.offboard_setpoint_counter += 1
            self._pub_pos_sp(self.lpos.x, self.lpos.y, self.lpos.z, self.init_yaw_deg)

            # 2) ARM
            if self.offboard_setpoint_counter == 20:
                self._arm()
                self.get_logger().info("ARM 명령 전송")

            # 3) OFFBOARD 전환
            if self.offboard_setpoint_counter == 30:
                self._offboard()
                self.get_logger().info("OFFBOARD 모드 전환")

            # 4) 제자리 수직 상승: x,y 고정, z만 목표(-35 m)로, yaw는 초기값 유지
            if self.offboard_setpoint_counter >= 40:
                self._pub_pos_sp(self.lpos.x, self.lpos.y, TAKEOFF_ALTITUDE_NED, self.init_yaw_deg)

                # 이륙 고도 도달 여부(절대오차) 확인
                if abs(self.lpos.z - TAKEOFF_ALTITUDE_NED) <= ALT_REACHED_EPS:
                    # 목표 범위에 들어오면 호버 유지 카운트
                    self.takeoff_hold_ticks += 1
                    if (self.takeoff_hold_ticks * self.dt) >= TAKEOFF_HOLD_SEC:
                        # 충분히 버텼으면 FLYING 전환
                        self.state = "FLYING"
                        self.segment_idx = 1
                        self.get_logger().info("이륙 고도 도달 및 호버 완료 → FLYING 진입")
                else:
                    # 아직 범위 밖이면 카운터 리셋
                    self.takeoff_hold_ticks = 0

        elif self.state == "FLYING":
            # 현재 목표 웨이포인트
            curr = self.ned_wps[names[self.segment_idx - 1]]
            nxt  = self.ned_wps[names[self.segment_idx]]

            # 이동 (leg 방향으로 yaw)
            yaw_deg = self._yaw_deg_to(curr, nxt)
            self._pub_pos_sp(nxt["x"], nxt["y"], nxt["z"], yaw_deg)

            # 도착 판정
            if self._reached(nxt):
                self.get_logger().info(f"{names[self.segment_idx]} 도착")
                # 도착한 지점에서 5초 호버 단계로 전환
                self.state = "WP_HOVER"
                self.current_wp_name = names[self.segment_idx]  # 방금 도착한 이름
                self.wp_hold_ticks = 0

        elif self.state == "WP_HOVER":
            # 방금 도착한 웨이포인트 위치로 고정해서 5초 호버
            wp = self.ned_wps[self.current_wp_name]
            # yaw는 유지(변경 없이), 위치만 고정
            self._pub_pos_sp(wp["x"], wp["y"], wp["z"], self.init_yaw_deg)  # yaw는 유지할 값이 없으면 초기 yaw로 고정

            self.wp_hold_ticks += 1
            if (self.wp_hold_ticks * self.dt) >= WP_HOVER_SEC:
                # 다음 단계: ALIGN (다음 웨이포인트 방향으로 기수 정렬)
                # 마지막 웨이포인트인 경우에는 정렬 생략하고 착륙
                if self.segment_idx >= (len(names) - 1):
                    self.state = "LANDING"
                    self.get_logger().info("마지막 웨이포인트 호버 완료 → 착륙")
                else:
                    self.state = "WP_ALIGN"
                    self.align_hold_ticks = 0
                    self.get_logger().info("호버 완료 → 기수 정렬 단계 진입")

        elif self.state == "WP_ALIGN":
            # 현재 WP에서 제자리, yaw만 다음 WP 방향으로 ALIGN_HOLD_SEC 만큼 유지
            idx_now = self.segment_idx  # current_wp_name == names[idx_now]
            wp_now  = self.ned_wps[self.current_wp_name]
            wp_next = self.ned_wps[self.names_sorted[self.segment_idx + 1]]
            yaw_next_deg = self._yaw_deg_to(wp_now, wp_next)

            # 위치는 고정, yaw만 다음 방향
            self._pub_pos_sp(wp_now["x"], wp_now["y"], wp_now["z"], yaw_next_deg)

            self.align_hold_ticks += 1
            if (self.align_hold_ticks * self.dt) >= ALIGN_HOLD_SEC:
                # 정렬 완료 → 다음 세그먼트로 이동 시작
                self.segment_idx += 1
                if self.segment_idx >= len(self.names_sorted):
                    self.state = "LANDING"
                else:
                    self.state = "FLYING"
                self.get_logger().info(f"기수 정렬 완료 → WP{self.segment_idx}로 출발")

        elif self.state == "LANDING":
            self._land()
            self.get_logger().info("착륙 명령 전송")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
