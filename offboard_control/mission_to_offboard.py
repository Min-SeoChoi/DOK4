#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from px4_msgs.msg import (
    VehicleGlobalPosition,
    VehicleLocalPosition,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus
)

EARTH_RADIUS = 6371000.0  # meters


def haversine_m(lat1, lon1, lat2, lon2):
    """WGS84 두 점 사이의 호버사인 거리[m]"""
    la1 = math.radians(lat1)
    lo1 = math.radians(lon1)
    la2 = math.radians(lat2)
    lo2 = math.radians(lon2)
    dlat = la2 - la1
    dlon = lo2 - lo1
    a = math.sin(dlat / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c


class MissionToOffboard(Node):
    def __init__(self):
        super().__init__('mission_to_offboard')

        # -------- 파라미터 (원하면 ros2 run 시 --ros-args -p 로 덮어쓰기 가능) --------
        self.declare_parameter('target_lat', 35.051808)
        self.declare_parameter('target_lon', 128.039227)
        self.declare_parameter('threshold_m', 5.0)  # 마지막 WP 도달 판정 거리
        self.declare_parameter('stream_hz', 50.0)    # Offboard setpoint 송출 주기(≥20Hz 권장)

        self.TARGET_LAT = float(self.get_parameter('target_lat').value)
        self.TARGET_LON = float(self.get_parameter('target_lon').value)
        self.THRESHOLD_M = float(self.get_parameter('threshold_m').value)
        self.HZ = float(self.get_parameter('stream_hz').value)

        # -------- 상태 변수 --------
        self.current_lat = None
        self.current_lon = None
        self.local_x = None
        self.local_y = None
        self.local_z = None
        self.hold_alt_ned = None  # 전환 이후 유지할 NED 고도(z>0은 아래)
        self.offboard_requested = False
        self.offboard_active = False

        # -------- 구독자: 센서/상태 토픽은 SensorData QoS(BEST_EFFORT)로! --------
        self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.on_gpos,
            qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_lpos,
            qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.on_status,
            qos_profile_sensor_data
        )

        # -------- 퍼블리셔: 입력 계열은 기본 QoS(RELIBLE)로 OK --------
        self.pub_ofb = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # -------- 타이머: setpoint keepalive + 도달 체크 --------
        self.timer = self.create_timer(1.0 / self.HZ, self.on_timer)

        self.get_logger().info("Mission→Offboard takeover node started.")

    # ======================= 콜백 =======================
    def on_gpos(self, msg: VehicleGlobalPosition):
        self.current_lat = msg.lat
        self.current_lon = msg.lon

    def on_lpos(self, msg: VehicleLocalPosition):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z
        # 전환 후 hold용 기준 고도는 최초 한 번 기록
        if self.hold_alt_ned is None and (msg.z == msg.z):  # NaN 체크 (NaN은 자기 자신과 다름)
            self.hold_alt_ned = msg.z

    def on_status(self, msg: VehicleStatus):
        self.offboard_active = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    # ======================= 주기 처리 =======================
    def on_timer(self):
        # 항상 OffboardControlMode + TrajectorySetpoint keepalive 송출(전환 허용 조건)
        self.publish_offboard_keepalive()

        # 글로벌 위치 없으면 판단 불가
        if self.current_lat is None or self.current_lon is None:
            return

        # 마지막 WP 도달 체크
        dist_m = haversine_m(self.current_lat, self.current_lon, self.TARGET_LAT, self.TARGET_LON)

        if (not self.offboard_requested) and dist_m <= self.THRESHOLD_M:
            self.get_logger().info(f"Reached final WP (~{dist_m:.1f} m). Switching to OFFBOARD…")
            self.request_offboard_mode()
            self.offboard_requested = True

        # 오프보드 활성 상태면 위치 hold (원하면 여기서 목표 경로를 생성해도 됨)
        if self.offboard_active and (self.local_x is not None) and (self.local_y is not None):
            z_hold = self.hold_alt_ned if self.hold_alt_ned is not None else -5.0
            self.publish_position_hold(self.local_x, self.local_y, z_hold)

    # ======================= 퍼블리시 유틸 =======================
    def _now_us(self) -> int:
        # PX4 메시지는 µs 단위 timestamp 사용
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_keepalive(self):
        # 1) OffboardControlMode: position 제어 사용
        ofb = OffboardControlMode()
        ofb.timestamp = self._now_us()
        ofb.position = True
        ofb.velocity = False
        ofb.acceleration = False
        ofb.attitude = False
        ofb.body_rate = False
        self.pub_ofb.publish(ofb)

        # 2) TrajectorySetpoint keepalive
        ts = TrajectorySetpoint()
        ts.timestamp = self._now_us()
        # 아직 구속하지 않음(NaN) → 전환 허용을 위한 keepalive 목적
        ts.position = [float('nan'), float('nan'), float('nan')]
        ts.velocity = [0.0, 0.0, 0.0]
        ts.acceleration = [float('nan'), float('nan'), float('nan')]
        ts.yaw = float('nan')
        ts.yawspeed = float('nan')
        self.pub_traj.publish(ts)

    def publish_position_hold(self, x: float, y: float, z: float):
        ts = TrajectorySetpoint()
        ts.timestamp = self._now_us()
        ts.position = [float(x), float(y), float(z)]
        ts.velocity = [0.0, 0.0, 0.0]
        ts.acceleration = [float('nan'), float('nan'), float('nan')]
        ts.yaw = float('nan')       # 현재 yaw 유지
        ts.yawspeed = float('nan')
        self.pub_traj.publish(ts)

    def request_offboard_mode(self):
        # MAV_CMD_DO_SET_MODE: base_mode=1(CUSTOM), custom_mode=6(OFFBOARD)
        cmd = VehicleCommand()
        cmd.timestamp = self._now_us()
        cmd.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.pub_cmd.publish(cmd)
        self.get_logger().info("VehicleCommand sent: request OFFBOARD.")

def main():
    rclpy.init()
    node = MissionToOffboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
