from setuptools import setup

# =====================================================
# PACKAGE NAME
# =====================================================
# This must match:
#   - The folder name inside src/
#   - The name defined in package.xml
# =====================================================

package_name = 'drone_mission'

setup(

    # =====================================================
    # BASIC PACKAGE INFORMATION
    # =====================================================

    name=package_name,
    version='0.0.0',  # Initial development version

    # =====================================================
    # PYTHON PACKAGES TO INSTALL
    # =====================================================
    # This tells setuptools which Python modules
    # should be installed when running:
    #
    #   colcon build
    #
    # It assumes there is a folder:
    #   drone_mission/
    # containing __init__.py
    # =====================================================

    packages=[package_name],

    # =====================================================
    # NON-PYTHON FILES TO INSTALL
    # =====================================================
    # data_files installs additional resources into the
    # ROS2 workspace install directory.
    #
    # This includes:
    #   - package.xml
    #   - launch files
    #   - resource index entry
    #
    # These are required for ROS2 to properly detect
    # and launch the package.
    # =====================================================

    data_files=[

        # Registers the package in ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Installs package.xml
        ('share/' + package_name, ['package.xml']),

        # Installs launch file
        ('share/' + package_name + '/launch',
         ['launch/swarm.launch.py']),
    ],

    # =====================================================
    # DEPENDENCIES
    # =====================================================
    # setuptools is required to build the Python package.
    # Additional Python dependencies (e.g., pymavlink)
    # should be added here if needed.
    # =====================================================

    install_requires=['setuptools'],

    zip_safe=True,

    # =====================================================
    # MAINTAINER INFORMATION
    # =====================================================

    maintainer='elena',
    maintainer_email='elena@todo.todo',

    # =====================================================
    # PACKAGE DESCRIPTION
    # =====================================================
    # Short explanation of the project purpose.
    # =====================================================

    description='ROS2-based multi-drone mission using PyMavLink and ArduPilot SITL',

    license='Apache-2.0',

    # =====================================================
    # ENTRY POINTS
    # =====================================================
    # Defines executable commands that can be run with:
    #
    #   ros2 run drone_mission <executable_name>
    #
    # Format:
    #   executable_name = python_module:function
    #
    # Example:
    #   ros2 run drone_mission leader
    #
    # will call:
    #   drone_mission/leader.py -> main()
    # =====================================================

    entry_points={
        'console_scripts': [

            # Leader drone control node
            'leader = drone_mission.leader:main',

            # Follower drones control nodes
            'follower1 = drone_mission.follower1:main',
            'follower2 = drone_mission.follower2:main',
            'follower3 = drone_mission.follower3:main',
            'follower4 = drone_mission.follower4:main',
        ],
    },
)
