import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import threading
import numpy as np
# geometry_msgs.msg is no longer needed since we are using move_to_pose

# From SRDF/MoveIt config package
PLANNING_GROUP_NAME = "manipulator" 
JOINT_NAMES = [
    "J1", "J2", "J3", 
    "J4", "J5", "J6"
] 
BASE_LINK_NAME = "base_link"        
END_EFFECTOR_NAME = "flange" # Corrected end-effector name
# -----------------------

# --- RASTER PATH PARAMETERS ---
HEIGHT = 0.50 # Z-height from base_link in meters
RASTER_SIZE = 0.50 # Side length of the square area (50 cm x 50 cm) in meters
LINE_COUNT = 5 # Number of back-and-forth passes

# Calculate spacing between lines
LINE_SPACING = RASTER_SIZE / (LINE_COUNT - 1) if LINE_COUNT > 1 else RASTER_SIZE

# Starting X position (in front of the base)
START_X = 0.50 
# Starting Y position (start of the raster area)
START_Y = -RASTER_SIZE / 2.0 
START_Z = HEIGHT

# Fixed orientation (identity quaternion: [x, y, z, w])
TARGET_ORIENTATION = [0.0, 0.70710678, 0.0, 0.70710678]
# ------------------------------

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.get_logger().info('Initializing MoveIt2 client...')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK_NAME,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=PLANNING_GROUP_NAME,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )
        
        # Add a small delay for MoveIt state initialization
        self.get_logger().info('Waiting for MoveIt2 to settle...')
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        
        # Start the motion execution in a new thread
        threading.Thread(target=self.execute_motion).start()

    def generate_raster_waypoints(self):
        """Generates a list of [position, orientation] lists for the raster path."""
        waypoints = []
        
        # 1. Approach the starting point above the path
        waypoints.append(([START_X, START_Y, START_Z], TARGET_ORIENTATION))
        
        for i in range(LINE_COUNT):
            current_y = START_Y + i * LINE_SPACING
            
            # Determine the start and end X coordinates for the sweeping motion
            if i % 2 == 0:
                # Even line (0, 2, 4): move along +X
                x_start = START_X
                x_end = START_X + RASTER_SIZE
            else:
                # Odd line (1, 3): move along -X
                x_start = START_X + RASTER_SIZE
                x_end = START_X

            # Add the sweeping endpoint
            waypoints.append(([x_end, current_y, START_Z], TARGET_ORIENTATION))
            
            # Add the transition point to the next line's start Y position
            if i < LINE_COUNT - 1:
                next_y = START_Y + (i + 1) * LINE_SPACING
                # This point moves in Y, staying at the current X_end position
                waypoints.append(([x_end, next_y, START_Z], TARGET_ORIENTATION))

        return waypoints


    def execute_motion(self):
        
        # 1. Generate Waypoints (list of [position, orientation] lists)
        waypoints = self.generate_raster_waypoints()
        
        self.get_logger().info(f"Attempting to move through {len(waypoints)} sequential Cartesian poses...")

        # 2. Execute Sequential Cartesian Path
        for i, (position, orientation) in enumerate(waypoints):
            self.get_logger().info(f"Moving to waypoint {i+1}/{len(waypoints)}: {position}")

            # Use move_to_pose for each segment
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=orientation,
                frame_id=BASE_LINK_NAME,
                # CRITICAL: Forces Cartesian planning for this single segment
                cartesian=True, 
                tolerance_position=0.01,
                tolerance_orientation=0.5, # Relaxed tolerance for easier planning
            )
            
            # Wait after *every* move to guarantee the sequential nature of the path
            self.moveit2.wait_until_executed()
            
        self.get_logger().info("✅ Raster motion completed successfully.")
        
         # Allow cleanup and shutdown
        try:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))
        except Exception:
            pass
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    
    executor = rclpy.executors.MultiThreadedExecutor(2) 
    executor.add_node(node)
    
  # Use a multithreaded executor to allow the node to receive results 
    # from the Move Group Action server while it's waiting.
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error during execution: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': 
    main()