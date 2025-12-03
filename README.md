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

## Running the example controller node

Usage
1. Source your ROS2 and MoveIt2 workspaces (example): 


   `source /opt/ros/humble/setup.bash && source ~/ws_fanuc/install/setup.bash`

2. Build the package from the directory that contains the node if it isn't built


   `colcon build --symlink-install --packages-select my_robot_controller`

3. Run move_group for your robot (from your MoveIt2 config / launch files).
   
   `ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx20ia_l use_mock:=true`

   OR

   `ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx20ia_l robot_ip:="192.168.1.100"`
   DO NOT RUN WITH PHYSICAL HARDWARE UNLESS YOU ARE SURE THE AREA IS CLEAR!!


4. Run the service:
   `ros2 run my_robot_controller move_to_joint`

## Troubleshooting


```bash
# Source your ROS and workspace overlays in the same shell
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verify the executable is registered and run it
ros2 pkg executables my_robot_controller
ros2 run my_robot_controller move_to_joint
```

Notes:

- The node will contact MoveIt2's `move_group` and your robot controllers. Make sure `move_group` is running (for example with the mock launch shown above) before running the node.
- If Python cannot import the package at runtime you may need to source the workspace install in the shell used to run `ros2 run` (the `source install/setup.bash` step above). In development environments you can also add that source line to your `~/.bashrc` so new shells include the overlay automatically.
- As a last resort for quick testing you can add the package `src` folder to `PYTHONPATH` before running (not recommended long-term):

```bash
export PYTHONPATH=$(pwd)/src/my_robot_controller:$PYTHONPATH
ros2 run my_robot_controller move_to_joint
```


Important notes
- This script expects a working ROS2 environment and MoveIt2 (move_group) running for your robot.

