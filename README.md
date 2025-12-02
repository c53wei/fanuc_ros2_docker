# Workspace Setup in VSCODE

Using VSCode VNC viewer for RViz which is compatible with non-linux machines like Windows and Mac. 

1. With the folder open in VSCode, use the command palette to Reopen in Container. 
2. Use the provided Dockerfile and devcontainer.json to build and run your container.
3. Once running, you should be able to access the virtual desktop at `localhost:6060`

## Launching MoveIt with Mock Hardware
`ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx20ia_l use_mock:=true`

## Launching MoveIt with RViz on a Real Robot
`ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx20ia_l robot_ip:="192.168.1.100"`

# Raster Playground - MoveIt2 + ROS2 helper

This repository contains a simple development script `src/raster_playground.py` that demonstrates planning and executing a pose on a robot MoveIt group (example: CRX20ia_L).

Important notes
- This script expects a working ROS2 environment and MoveIt2 (move_group) running for your robot.
- The Python MoveIt bindings (`moveit_commander`) must be installed on the host.

Usage
1. Source your ROS2 and MoveIt2 workspaces (example):

   # Bash example (adjust to your install)
   source /opt/ros/humble/setup.bash 
   source ~/ws_fanuc/install/setup.bash

2. Run move_group for your robot (from your MoveIt2 config / launch files).
   
   * See above instructions to either launch with mock or real hardware.

3. Run the script:

   python3 src/raster_playground.py

4. Change parameters at runtime with `ros2 param set` (for example change reference frame):

   ros2 param set /raster_playground reference_frame "world"

Parameters
- `group_name`: MoveIt move group name (default: `manipulator`)
- `reference_frame`: Frame in which the target pose is expressed (default: `world`)
- `target_x`, `target_y`, `target_z`: Target position in meters
- `target_roll`, `target_pitch`, `target_yaw`: Target orientation (radians)

Notes & next steps
- This is a lightweight development helper — to integrate into a robot system, convert it into a ROS2 package with a proper `package.xml`, `setup.py` and launch files.
- If moveit_commander bindings are not installed, install MoveIt2 and the Python wrappers for your ROS2 distribution.
