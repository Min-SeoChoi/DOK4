#!/usr/bin/env python3
"""
본 코드에서는,
1. Arm
2. 이륙고도 5m로 이륙
3. 이륙 완료 시 5초 호버
4. 전진 5m
5. 전진 완료 시 5초 호버
6. 착륙
순으로 동작합니다.
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_mili')

        # QoS: Best Effort for PX4 → ROS2 브리지 호환
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # 퍼블리셔
        self.pub_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp   = self.create_publisher(
            TrajectorySetpoint,   '/fmu/in/trajectory_setpoint',    qos)
        self.pub_cmd  = self.create_publisher(
            VehicleCommand,       '/fmu/in/vehicle_command',         qos)

        # 구독자
        self.sub_pos    = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.pos_cb, qos)
        self.sub_status = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.status_cb, qos)

        # 내부 상태 변수
        self.current_x = 0.0
        self.current_z = 0.0
        self.nav_state = None

        self.state       = 0    # 0=takeoff,1=hover1,2=forward,3=hover2,4=land
        self.loop_count  = 0    # OFFBOARD 준비용 카운터
        self.hover_count = 0    # 호버링 카운터

        # 목표값
        self.takeoff_alt = -5.0   # NED Z = -5m
        self.forward_dist = 5.0   # x = +5m

        # 100 Hz 제어 루프
        self.timer = self.create_timer(0.01, self.loop)

    def pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_z = msg.z

    def status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state

    def send_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.position  = True
        self.pub_mode.publish(msg)

    def send_sp(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.position  = [float(x), float(y), float(z)]
        msg.yaw       = float(yaw)
        self.pub_sp.publish(msg)

    def send_cmd(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp       = int(self.get_clock().now().nanoseconds // 1000)
        msg.command         = command
        msg.param1          = params.get('param1', 0.0)
        msg.param2          = params.get('param2', 0.0)
        msg.param3          = params.get('param3', 0.0)
        msg.param4          = params.get('param4', 0.0)
        msg.param5          = params.get('param5', 0.0)
        msg.param6          = params.get('param6', 0.0)
        msg.param7          = params.get('param7', 0.0)
        msg.target_system   = 1
        msg.target_component= 1
        msg.source_system   = 1
        msg.source_component= 1
        msg.from_external   = True
        self.pub_cmd.publish(msg)

    def loop(self):
        # 1) OFFBOARD 준비 (10 사이클 동안 heartbeat + SP)
        if self.loop_count < 10:
            self.send_mode()
            self.send_sp(0.0, 0.0, 0.0)
            self.loop_count += 1
            return

        # 2) 한 번만 Arm + Offboard 모드 전환
        if self.loop_count == 10:
            # Arm
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            # Offboard 모드 (base_mode=1, custom_mode=6)
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Armed & switched to OFFBOARD')
        # 모드 유지용 heartbeat
        self.send_mode()

        # 상태 머신
        if self.state == 0:
            # 이륙 5m
            self.send_sp(0.0, 0.0, self.takeoff_alt)
            if self.current_z <= (self.takeoff_alt + 0.5) and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.state = 1
                self.hover_count = 0
                self.get_logger().info('Reached 5m, start hover 5s')

        elif self.state == 1:
            # 호버 5초 (100Hz × 5s = 500 사이클)
            self.send_sp(0.0, 0.0, self.takeoff_alt)
            self.hover_count += 1
            if self.hover_count >= 500:
                self.state = 2
                self.get_logger().info('Hover1 complete, moving forward')

        elif self.state == 2:
            # 전진 5m
            self.send_sp(self.forward_dist, 0.0, self.takeoff_alt)
            if self.current_x >= (self.forward_dist - 0.5):
                self.state = 3
                self.hover_count = 0
                self.get_logger().info('Reached forward 5m, start hover 5s')

        elif self.state == 3:
            # 호버 5초
            self.send_sp(self.forward_dist, 0.0, self.takeoff_alt)
            self.hover_count += 1
            if self.hover_count >= 500:
                self.state = 4
                self.get_logger().info('Hover2 complete, landing')

        elif self.state == 4:
            # 착륙
            self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info('Land command sent')
            self.timer.cancel()

        self.loop_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
