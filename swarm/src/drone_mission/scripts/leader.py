#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String, Float32
from pymavlink import mavutil
import time
import math


# =====================================================
# MAVLINK CONNECTION
# =====================================================
# Connects to ArduPilot SITL through UDP.
# Instance 0 (leader) uses port 14550.
# =====================================================

connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Connecting Leader...")
connection.wait_heartbeat()
print("Leader connected")


# =====================================================
# CONTROL PARAMETERS
# =====================================================

FIXED_ALTITUDE = -10          # NED frame (negative Z = upward)
Kp_pos = 0.4                  # Proportional position controller gain
V_MAX = 4.0                   # Maximum horizontal velocity (m/s)
STOP_RADIUS = 0.3             # Distance threshold to stop (m)

LOW_BATTERY_THRESHOLD = 20.0  # Percentage threshold for emergency landing
emergency_landing = False


# =====================================================
# GLOBAL VARIABLES
# =====================================================

prev_x = None
prev_y = None
prev_time = None

target_x = None
target_y = None

ready_set = set()             # Synchronization barrier
formation_active = False


# =====================================================
# MAVLINK FUNCTIONS
# =====================================================

def set_mode(mode):
    """
    Sets vehicle flight mode and waits until confirmed.
    """
    connection.set_mode(mode)
    while True:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True)
        if mavutil.mode_string_v10(msg) == mode:
            print(f"Mode {mode} confirmed")
            break


def arm_vehicle():
    """
    Arms the drone and waits until safety flag confirms.
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
            print("Leader ARMED")
            break


def takeoff():
    """
    Sends takeoff command to reach 10 meters altitude.
    """
    print("Taking off...")

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        10
    )

    time.sleep(8)
    print("Leader airborne")


def land_vehicle():
    """
    Switches vehicle to LAND mode.
    """
    print("LOW BATTERY → Landing formation")

    connection.set_mode('LAND')

    while True:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True)
        if mavutil.mode_string_v10(msg) == 'LAND':
            print("LAND mode confirmed")
            break


def get_local_position():
    """
    Reads LOCAL_POSITION_NED message.
    Returns (x, y) position in meters.
    """
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if msg:
        return msg.x, msg.y
    return None, None


def send_velocity(vx, vy):
    """
    Sends velocity command in LOCAL_NED frame.
    Velocity is saturated to V_MAX.
    """

    speed = math.sqrt(vx * vx + vy * vy)
    if speed > V_MAX:
        scale = V_MAX / speed
        vx *= scale
        vy *= scale

    for _ in range(5):
        connection.mav.set_position_target_local_ned_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Use velocity only
            0, 0, FIXED_ALTITUDE,
            vx, vy, 0,
            0, 0, 0,
            0, 0
        )
        time.sleep(0.1)


def push_visual_target_to_ardupilot():
    """
    Sends position target to ArduPilot (visual guidance line).
    Only used if a valid target exists.
    """
    if target_x is None:
        return

    connection.mav.set_position_target_local_ned_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,  # Position only
        target_x,
        target_y,
        FIXED_ALTITUDE,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def control_to_target():
    """
    Proportional position controller toward target.
    Only active after formation synchronization.
    """
    if not formation_active:
        return

    if target_x is None:
        return

    x, y = get_local_position()
    if x is None:
        return

    ex = target_x - x
    ey = target_y - y

    dist = math.sqrt(ex * ex + ey * ey)

    if dist < STOP_RADIUS:
        send_velocity(0.0, 0.0)
        return

    vx = Kp_pos * ex
    vy = Kp_pos * ey
    send_velocity(vx, vy)


# =====================================================
# ROS2 NODE
# =====================================================

def main():

    global prev_x, prev_y, prev_time
    global target_x, target_y
    global ready_set, formation_active
    global emergency_landing

    # ---------- MAVLINK INITIALIZATION ----------

    set_mode('GUIDED')
    time.sleep(2)

    arm_vehicle()
    time.sleep(2)

    takeoff()

    # ---------- ROS2 INITIALIZATION ----------

    rclpy.init()
    node = rclpy.create_node('leader_node')

    # Publishers
    pos_pub = node.create_publisher(Point, '/leader_local_position', 10)
    vel_pub = node.create_publisher(Point, '/leader_velocity', 10)
    path_pub = node.create_publisher(Path, '/leader_path', 10)
    ready_pub = node.create_publisher(String, '/drone_ready', 10)
    start_pub = node.create_publisher(String, '/start_formation', 10)
    battery_pub = node.create_publisher(Float32, '/leader_battery', 10)
    land_pub = node.create_publisher(String, '/swarm_land', 10)

    tf_broadcaster = TransformBroadcaster(node)

    path_msg = Path()
    path_msg.header.frame_id = "world"

    # =====================================================
    # SYNCHRONIZATION BARRIER
    # =====================================================

    def publish_ready_burst():
        msg = String()
        msg.data = "leader"
        for _ in range(10):
            ready_pub.publish(msg)
            time.sleep(0.05)
        print("[SYNC] READY sent: leader")

    publish_ready_burst()
    ready_set.add("leader")

    def ready_callback(msg: String):
        global formation_active, ready_set, target_x, target_y

        ready_set.add(msg.data)
        print(f"[SYNC] READY received from: {msg.data} | set={ready_set}")

        if (not formation_active and
            {"leader", "follower1", "follower2", "follower3", "follower4"}.issubset(ready_set)):

            formation_active = True
            start_pub.publish(String(data="start"))
            print("[SYNC] All drones ready → START published")

            x, y = get_local_position()
            if x is not None:
                target_x = x
                target_y = y
                push_visual_target_to_ardupilot()

    node.create_subscription(String, '/drone_ready', ready_callback, 10)

    # =====================================================
    # DATA PUBLISHING (POSITION, TF, PATH, VELOCITY)
    # =====================================================

    def publish_data():
        global prev_x, prev_y, prev_time

        x, y = get_local_position()
        if x is None:
            return

        now_ros = node.get_clock().now().to_msg()
        now_time = time.time()

        # ---- Publish leader position ----
        pos_msg = Point()
        pos_msg.x = x
        pos_msg.y = y
        pos_pub.publish(pos_msg)

        # ---- Publish TF transform ----
        t = TransformStamped()
        t.header.stamp = now_ros
        t.header.frame_id = "world"
        t.child_frame_id = "leader/base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 10.0
        t.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(t)

        # ---- Publish Path ----
        pose = PoseStamped()
        pose.header.stamp = now_ros
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 10.0
        pose.pose.orientation.w = 1.0

        path_msg.header.stamp = now_ros
        path_msg.poses.append(pose)

        if len(path_msg.poses) > 300:
            path_msg.poses.pop(0)

        path_pub.publish(path_msg)

        # ---- Compute and publish velocity (only if formation active) ----
        if formation_active and prev_x is not None and prev_time is not None:

            dt = now_time - prev_time
            if dt > 0:
                vx = (x - prev_x) / dt
                vy = (y - prev_y) / dt

                vel_msg = Point()
                vel_msg.x = vx
                vel_msg.y = vy
                vel_pub.publish(vel_msg)

        prev_x = x
        prev_y = y
        prev_time = now_time
    
    # =====================================================
    # TARGET POSITION CALLBACK
    # =====================================================

    def position_callback(msg):
        global target_x, target_y

        if not formation_active:
            print("Target ignored: formation not active yet")
            return

        target_x = msg.x
        target_y = msg.y
        print(f"New LOCAL target received: x={target_x}, y={target_y}")

        push_visual_target_to_ardupilot()

    node.create_subscription(Point, '/target_position', position_callback, 10)

    # =====================================================
    # BATTERY MONITOR
    # =====================================================

    def publish_battery():
        global emergency_landing, formation_active

        msg = connection.recv_match(type='BATTERY_STATUS', blocking=False)

        if msg:
            battery = float(msg.battery_remaining)

            battery_msg = Float32()
            battery_msg.data = battery
            battery_pub.publish(battery_msg)

            node.get_logger().info(f'Leader Battery: {battery}%')

            if battery <= LOW_BATTERY_THRESHOLD and not emergency_landing:

                emergency_landing = True
                formation_active = False

                land_vehicle()

                land_msg = String()
                land_msg.data = "land"
                land_pub.publish(land_msg)

                print("LAND command sent to formation")

    # =====================================================
    # PERIODIC TIMERS
    # =====================================================

    node.create_timer(0.2, publish_data)
    node.create_timer(0.1, control_to_target)
    node.create_timer(5.0, publish_battery)

    print("Leader ready with TF + Path + SYNC + Velocity Control")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


# =====================================================
# MAIN ENTRY
# =====================================================

if __name__ == '__main__':
    main()
