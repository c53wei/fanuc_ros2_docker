import sys
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import threading

# From SRDF/MoveIt config package
PLANNING_GROUP_NAME = "manipulator"
JOINT_NAMES = [
    "J1", "J2", "J3",
    "J4", "J5", "J6"
]
BASE_LINK_NAME = "base_link"
END_EFFECTOR_NAME = "tool0"

TARGET_JOINT_POSITIONS = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]


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
            # Use MutuallyExclusiveCallbackGroup to allow MoveIt2's internal 
            # threads to run while the main node spins.
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )

        # Start the action in a new thread to prevent blocking the ROS spin
        threading.Thread(target=self.execute_motion).start()

    def execute_motion(self):
        self.get_logger().info(f"Attempting to move to joint goal: {TARGET_JOINT_POSITIONS}")

        # Plan and execute the motion in a single call
        # This will send a goal to the Move Group node
        # Plan and execute using pymoveit2 MoveIt2 API. The signature for
        # move_to_configuration accepts joint_positions (and optional joint_names,
        # tolerance/weight). Remove unsupported kwargs.
        self.moveit2.move_to_configuration(
            joint_positions=TARGET_JOINT_POSITIONS,
        )

        # Wait until the planning and execution is complete
        self.moveit2.wait_until_executed()

        self.get_logger().info("✅ Motion completed successfully.")

        # Allow cleanup and shutdown
        try:
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))
        except Exception:
            pass
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()

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
