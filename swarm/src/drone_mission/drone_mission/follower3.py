#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from pymavlink import mavutil
import time
import math


# =====================================================
# MAVLINK CONNECTION (FOLLOWER 3)
# =====================================================
# Connects to ArduPilot SITL instance 3 via UDP.
# By your port convention:
#   leader    -> 14550
#   follower1 -> 14560
#   follower2 -> 14570
#   follower3 -> 14580
# =====================================================

connection = mavutil.mavlink_connection('udp:127.0.0.1:14580')
print("Connecting Follower3...")
connection.wait_heartbeat()
print("Follower3 connected")


# =====================================================
# FORMATION / CONTROL PARAMETERS
# =====================================================
# Follower3 uses a larger offset than follower1/follower2:
#   - further behind the leader
#   - further to the LEFT of the leader
#
# NED frame reminder:
#   +X = North, +Y = East, +Z = Down
# Therefore, a negative Z means "up".
# =====================================================

FIXED_ALTITUDE = -10     # Hold altitude at ~10m above (NED negative Z)
DIST_BACK = 10.0         # Desired distance behind the leader (meters)
DIST_SIDE = 10.0         # Desired lateral offset to the LEFT (meters)
K = 0.35                 # Proportional gain for position error correction
V_MAX = 5.0              # Max horizontal speed (m/s)

leader_x = None
leader_y = None

# Synchronization flag: the follower will NOT move until START is received
formation_active = False


# =====================================================
# MAVLINK HELPER FUNCTIONS
# =====================================================

def set_mode(mode: str):
    """
    Sets the vehicle mode (e.g., GUIDED, LAND) and waits for confirmation.
    """
    mode_id = connection.mode_mapping()[mode]
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    while True:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True)
        if mavutil.mode_string_v10(msg) == mode:
            print(f"Mode {mode} confirmed")
            break


def arm_vehicle():
    """
    Arms the vehicle and waits until the armed flag is detected.
    """
    while True:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Follower3 ARMED")
            break


def takeoff():
    """
    Sends a takeoff command to reach ~10 meters.
    Takeoff altitude is given as a positive value (ArduPilot command convention).
    """
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 10
    )
    time.sleep(8)
    print("Follower3 airborne")


def get_local_position():
    """
    Reads LOCAL_POSITION_NED.
    Returns (x, y) in meters.
    """
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if msg:
        return msg.x, msg.y
    return None, None


def send_velocity(vx: float, vy: float):
    """
    Sends a horizontal velocity command (vx, vy) in LOCAL_NED frame,
    saturating the speed to V_MAX.
    """
    speed = math.sqrt(vx * vx + vy * vy)
    if speed > V_MAX:
        scale = V_MAX / speed
        vx *= scale
        vy *= scale

    # Send multiple times to "hold" the command for a short period
    for _ in range(5):
        connection.mav.set_position_target_local_ned_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Use velocity (vx, vy, vz); ignore position
            0, 0, FIXED_ALTITUDE,
            vx, vy, 0,
            0, 0, 0,
            0, 0
        )
        time.sleep(0.1)


def land_callback(msg: String):
    """
    Emergency landing handler: if the swarm receives a 'land' command,
    this follower switches to LAND mode.
    """
    if msg.data.strip().lower() == "land":
        print("LAND command received")
        connection.set_mode('LAND')


# =====================================================
# ROS2 CALLBACKS (DATA FROM LEADER + SYNC)
# =====================================================

def leader_position_callback(msg: Point):
    """
    Updates the leader's last known local position (x, y).
    """
    global leader_x, leader_y
    leader_x = msg.x
    leader_y = msg.y


def start_callback(msg: String):
    """
    Enables formation control only after leader publishes START.
    This prevents followers from moving before every drone is ready.
    """
    global formation_active
    if msg.data.strip().lower() == "start":
        formation_active = True
        print("[SYNC] Follower3 START received → formation active")


def leader_velocity_callback(msg: Point):
    """
    Main formation controller (Follower3):

    Same approach as follower1, but with larger offsets:
      - behind leader by DIST_BACK
      - left of leader by DIST_SIDE

    Inputs:
      - Leader velocity (vx_leader, vy_leader)
      - Leader position (leader_x, leader_y)
      - Follower current position (follower_x, follower_y)

    Strategy:
      1) Compute leader direction of motion (unit vector).
      2) Compute perpendicular vector to the LEFT (unit).
      3) Desired follower point:
           behind leader by DIST_BACK
           left of leader by DIST_SIDE
      4) Compute position error between desired point and follower.
      5) Command velocity = leader_velocity (feedforward) + K * error (P control)
    """
    global formation_active

    # Do not follow until the formation has been synchronized
    if not formation_active:
        return

    follower_x, follower_y = get_local_position()
    if leader_x is None or follower_x is None:
        return

    vx_leader = msg.x
    vy_leader = msg.y

    speed = math.sqrt(vx_leader * vx_leader + vy_leader * vy_leader)
    if speed < 0.1:
        return

    # Unit direction of leader motion
    dir_x = vx_leader / speed
    dir_y = vy_leader / speed

    # Left-hand perpendicular unit vector
    perp_x = -dir_y
    perp_y = dir_x

    # Desired position relative to leader
    desired_x = leader_x - dir_x * DIST_BACK + perp_x * DIST_SIDE
    desired_y = leader_y - dir_y * DIST_BACK + perp_y * DIST_SIDE

    # Position error (desired - current)
    error_x = desired_x - follower_x
    error_y = desired_y - follower_y

    # Feedforward (leader velocity) + proportional correction
    vx_cmd = vx_leader + K * error_x
    vy_cmd = vy_leader + K * error_y

    send_velocity(vx_cmd, vy_cmd)


# =====================================================
# MAIN
# =====================================================

def main():
    """
    Follower3 node lifecycle:
      1) MAVLink: set GUIDED, arm, takeoff
      2) ROS2: publish TF + Path for RViz
      3) ROS2: synchronize via READY/START
      4) ROS2: follow leader velocity + formation geometry (LEFT + larger offset)
      5) ROS2: respond to swarm LAND command

    Note:
      This script also sends a zero-velocity command before START,
      ensuring the follower stays still even if some stale command exists.
    """
    global formation_active

    # ---------- TAKEOFF SEQUENCE ----------
    set_mode('GUIDED')
    time.sleep(2)

    arm_vehicle()
    time.sleep(2)

    takeoff()

    # ---------- ROS2 INITIALIZATION ----------
    rclpy.init()
    node = rclpy.create_node('follower3_control')

    # RViz visualization publishers
    path_pub = node.create_publisher(Path, '/follower3_path', 10)
    tf_broadcaster = TransformBroadcaster(node)

    # Synchronization pubs/subs
    ready_pub = node.create_publisher(String, '/drone_ready', 10)
    node.create_subscription(String, '/start_formation', start_callback, 10)

    # Send READY burst
    ready_msg = String()
    ready_msg.data = "follower3"
    for _ in range(10):
        ready_pub.publish(ready_msg)
        time.sleep(0.05)
    print("[SYNC] READY sent: follower3")

    # Path message (history of poses)
    path_msg = Path()
    path_msg.header.frame_id = "world"

    # ---------- TF + PATH PUBLISHER ----------
    def publish_tf_and_path():
        """
        Publishes:
          - TF transform for follower3/base_link
          - nav_msgs/Path for trajectory visualization in RViz

        Safety behavior:
          If formation is not active yet, send zero velocity to keep the drone still.
        """
        if not formation_active:
            send_velocity(0.0, 0.0)

        x, y = get_local_position()
        if x is None:
            return

        now = node.get_clock().now().to_msg()

        # TF transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = "follower3/base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 10.0
        t.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(t)

        # PoseStamped for Path
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 10.0
        pose.pose.orientation.w = 1.0

        path_msg.header.stamp = now
        path_msg.poses.append(pose)

        # Keep only the last N poses
        if len(path_msg.poses) > 300:
            path_msg.poses.pop(0)

        path_pub.publish(path_msg)

    # ---------- SUBSCRIPTIONS ----------
    node.create_subscription(Point, '/leader_local_position',
                             leader_position_callback, 10)

    node.create_subscription(Point, '/leader_velocity',
                             leader_velocity_callback, 10)

    node.create_subscription(String, '/swarm_land',
                             land_callback, 10)

    # ---------- TIMER ----------
    node.create_timer(0.2, publish_tf_and_path)

    print("Follower3 ready (TF + Path + Sync)")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
