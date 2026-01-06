#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
import math

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        # Configure QoS profile for PX4 (Best Effort is required for UDP)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_status_subscriber_ = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber_ = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # Variables
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)
        
        self.mission_step = 0
        # Waypoints [x, y, z] in meters (NED frame)
        # Z is negative for Up
        self.waypoints = [
            [0.0, 0.0, -10.0],   # Takeoff
            [5.0, 5.0, -10.0],   # Fly to Building 2 (Base Station)
            [-4.0, 2.0, -10.0],  # Fly to Building 3 (Outpost)
            [0.0, 0.0, -10.0],   # Return Home
            [0.0, 0.0, 0.0]      # Land
        ]
        self.error_threshold = 0.5 # Distance acceptance radius

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to Land mode")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57 # Face North (roughly)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def distance_to_target(self, target):
        curr_x = self.vehicle_local_position.x
        curr_y = self.vehicle_local_position.y
        curr_z = self.vehicle_local_position.z
        
        dx = target[0] - curr_x
        dy = target[1] - curr_y
        dz = target[2] - curr_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def cmdloop_callback(self):
        # Always publish offboard control mode while in offboard
        self.publish_offboard_control_mode()

        # State Machine
        if self.mission_step == 0:
            # Wait for simulation to settle, then Arm
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.mission_step = 1
            else:
                self.engage_offboard_mode()
                self.arm()
                
        elif self.mission_step > 0 and self.mission_step <= len(self.waypoints):
            # Navigation Logic
            target = self.waypoints[self.mission_step - 1]
            self.publish_trajectory_setpoint(target[0], target[1], target[2])
            
            dist = self.distance_to_target(target)
            if dist < self.error_threshold:
                self.get_logger().info(f"Reached Waypoint {self.mission_step}")
                self.mission_step += 1
                
        elif self.mission_step > len(self.waypoints):
            # Land
            self.land()
            self.mission_step += 1 # Stop repeating land command

def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()