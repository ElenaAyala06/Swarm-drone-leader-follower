import os
import math
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction


def launch_setup(context, *args, **kwargs):

    print("\n========== SPAWN SWARM FORMATION ==========\n")

    # =====================================================
    # USER INPUT: Leader initial GPS position
    # =====================================================
    # The user manually introduces the leader's latitude,
    # longitude and altitude. Followers will be positioned
    # relative to this reference point.
    # =====================================================

    lat = float(input("Leader latitude: "))
    lon = float(input("Leader longitude: "))
    alt = float(input("Altitude (meters): "))
    heading = 0  # Initial yaw angle in degrees

    # =====================================================
    # CONVERSION FUNCTIONS
    # =====================================================
    # These functions convert meters into degrees of
    # latitude and longitude.
    #
    # Latitude:
    #   1 degree ≈ 111,320 meters
    #
    # Longitude:
    #   1 degree ≈ 111,320 * cos(latitude)
    #
    # This allows positioning drones using metric offsets.
    # =====================================================

    def meters_to_lat(meters):
        return meters / 111320.0

    def meters_to_lon(meters, latitude):
        return meters / (111320.0 * math.cos(math.radians(latitude)))

    # =====================================================
    # FORMATION DEFINITION
    # =====================================================
    # Dictionary format:
    #   instance_id : (north_offset_m, east_offset_m)
    #
    # Instance 0 -> Leader
    # Other instances -> Followers positioned behind and
    # diagonally relative to leader.
    # =====================================================

    formations = {
        0: (0, 0),        # Leader
        1: (-5, 5),       # Follower 1
        2: (-5, -5),      # Follower 2
        3: (-10, 10),     # Follower 3
        4: (-10, -10),    # Follower 4
    }

    processes = []

    # =====================================================
    # START MULTIPLE SITL INSTANCES
    # =====================================================
    # For each drone instance:
    #   - Compute new GPS position
    #   - Launch ArduPilot SITL with custom location
    #   - Assign unique UDP output port
    #
    # TimerAction is used to stagger startup times in
    # order to avoid port conflicts and race conditions.
    # =====================================================

    for instance, (north_m, east_m) in formations.items():

        new_lat = lat + meters_to_lat(north_m)
        new_lon = lon + meters_to_lon(east_m, lat)

        sitl = ExecuteProcess(
            cmd=[
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '-f', 'quad',
                f'-I{instance}',
                '--custom-location',
                f"{new_lat},{new_lon},{alt},{heading}",
                '--map',
                '--console',
                f'--out=udp:127.0.0.1:{14550 + instance*10}'
            ],
            output='screen'
        )

        # Each instance starts 4 seconds after the previous one
        processes.append(
            TimerAction(period=2.0 + instance*4.0, actions=[sitl])
        )

    # =====================================================
    # START PYTHON CONTROL SCRIPTS
    # =====================================================
    # These scripts handle:
    #   - Leader autonomous behavior
    #   - Followers formation control
    #
    # They are launched with delay to ensure that SITL
    # instances are fully initialized before connecting.
    # =====================================================

    leader_node = ExecuteProcess(
        cmd=['python3', '/home/elena/swarm/src/drone_mission/scripts/leader.py'],
        output='screen'
    )

    follower1_node = ExecuteProcess(
        cmd=['python3', '/home/elena/swarm/src/drone_mission/scripts/follower1.py'],
        output='screen'
    )

    follower2_node = ExecuteProcess(
        cmd=['python3', '/home/elena/swarm/src/drone_mission/scripts/follower2.py'],
        output='screen'
    )

    follower3_node = ExecuteProcess(
        cmd=['python3', '/home/elena/swarm/src/drone_mission/scripts/follower3.py'],
        output='screen'
    )

    follower4_node = ExecuteProcess(
        cmd=['python3', '/home/elena/swarm/src/drone_mission/scripts/follower4.py'],
        output='screen'
    )

    processes += [

        TimerAction(period=30.0, actions=[leader_node]),
        TimerAction(period=35.0, actions=[follower1_node]),
        TimerAction(period=40.0, actions=[follower2_node]),
        TimerAction(period=45.0, actions=[follower3_node]),
        TimerAction(period=50.0, actions=[follower4_node]),
    ]

    # =====================================================
    # START RVIZ VISUALIZATION
    # =====================================================
    # RViz is launched after all drones and controllers
    # are active, allowing real-time visualization of:
    #   - Drone positions
    #   - TF frames
    #   - Paths
    #   - Formation structure
    # =====================================================

    rviz = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            '/home/elena/swarm/src/drone_mission/scripts/swarm_rviz.rviz'
        ],
        output='screen'
    )

    processes.append(
        TimerAction(period=60.0, actions=[rviz])
    )

    return processes


# =====================================================
# MAIN LAUNCH DESCRIPTION
# =====================================================
# OpaqueFunction allows interactive input (GPS values)
# before launching processes.
# =====================================================

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
