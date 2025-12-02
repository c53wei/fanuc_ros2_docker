#!/usr/bin/env python3
"""
raster_playground.py

Simple ROS2 + MoveIt2 helper to plan and execute a pose on a robot group (e.g. CRX20ia_L).

Features:
- Uses MoveIt Python API (moveit_commander) to create RobotCommander and MoveGroupCommander.
- Allows changing the pose reference frame at runtime via a ROS2 parameter ("reference_frame").
- Reads a target pose from ROS2 parameters (x,y,z,roll,pitch,yaw) and plans+executes it.

Assumptions / notes:
- MoveIt2 and the appropriate Python integration (moveit_commander) are installed on the host.
- A running ROS2 daemon and MoveIt move_group for the robot are available.
- The move group name (parameter `group_name`) matches the MoveIt configuration.

This script is intended as a development playground and should be adapted into a ROS2 package
and a launch file for production use.
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import os
import glob
import re

try:
	# Prefer the ROS2 MoveIt Python bindings if available
	import pymoveit2 as pm2
	from geometry_msgs.msg import PoseStamped
	_HAS_PYMOVEIT2 = True
	_HAS_MOVEIT_COMMANDER = False
except Exception:
	pm2 = None
	try:
		import moveit_commander
		from geometry_msgs.msg import PoseStamped
		_HAS_PYMOVEIT2 = False
		_HAS_MOVEIT_COMMANDER = True
	except Exception:
		# Neither binding is available; keep variables None so we can log helpful errors
		pm2 = None
		moveit_commander = None
		PoseStamped = None
		_HAS_PYMOVEIT2 = False
		_HAS_MOVEIT_COMMANDER = False


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
	"""Convert roll, pitch, yaw to quaternion.
	Returns (x, y, z, w)
	"""
	cy = math.cos(yaw * 0.5)
	sy = math.sin(yaw * 0.5)
	cp = math.cos(pitch * 0.5)
	sp = math.sin(pitch * 0.5)
	cr = math.cos(roll * 0.5)
	sr = math.sin(roll * 0.5)

	w = cr * cp * cy + sr * sp * sy
	x = sr * cp * cy - cr * sp * sy
	y = cr * sp * cy + sr * cp * sy
	z = cr * cp * sy - sr * sp * cy
	return x, y, z, w


class RasterPlayground(Node):
	def __init__(self):
		super().__init__('raster_playground')

		# Parameters (can be changed via ros2 param set)
		self.declare_parameter('group_name', 'manipulator')
		self.declare_parameter('reference_frame', 'world')
		self.declare_parameter('target_x', 0.5)
		self.declare_parameter('target_y', 0.0)
		self.declare_parameter('target_z', 0.5)
		self.declare_parameter('target_roll', 0.0)
		self.declare_parameter('target_pitch', 0.0)
		self.declare_parameter('target_yaw', 0.0)

		self.group_name = self.get_parameter('group_name').get_parameter_value().string_value
		self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value

		# Initialize MoveIt interface depending on available bindings
		self.move_group = None
		self.moveit2 = None

		# Additional parameters for pymoveit2
		self.declare_parameter('base_link', 'base_link')
		self.declare_parameter('end_effector', 'ee_link')
		# joint_names can be provided as a list parameter
		self.declare_parameter('joint_names', [])

		if _HAS_PYMOVEIT2 and pm2 is not None:
			# Build MoveIt2 wrapper instance. pymoveit2 expects a node and joint/base/ee names.
			# Read joint_names parameter; try to support list or string forms
			param = self.get_parameter('joint_names')
			try:
				joint_names = param.get_parameter_value().string_array_value
			except Exception:
				# fallback: try to read as string and split
				val = param.get_parameter_value().string_value
				if val:
					# allow comma-separated or space-separated
					joint_names = [s.strip() for s in re.split('[,\s]+', val) if s.strip()]
				else:
					joint_names = []
			# rclpy Parameter list access above may be empty; handle both string and list styles
			if isinstance(joint_names, str):
				# single string, split by commas
				joint_names = [s.strip() for s in joint_names.split(',') if s.strip()]

			base_link = self.get_parameter('base_link').get_parameter_value().string_value
			end_effector = self.get_parameter('end_effector').get_parameter_value().string_value

			if not joint_names:
				self.get_logger().warning('pymoveit2 is available but `joint_names` parameter is empty; attempting to discover joint names from workspace URDF/XACRO files...')
				# Try to discover joint names automatically from common workspace locations
				discovered = self._discover_joint_names()
				if discovered:
					joint_names = discovered
					self.get_logger().info(f'Auto-discovered {len(joint_names)} joint names (using these for pymoveit2): {joint_names}')
				else:
					self.get_logger().warning('No joint names could be discovered automatically; please set the `joint_names` parameter to a correct ordered list for your robot.')

			try:
				# Create MoveIt2 helper. The constructor signature is (node, joint_names, base_link_name, end_effector_name, group_name=...)
				self.moveit2 = pm2.MoveIt2(self, joint_names, base_link, end_effector, group_name=self.group_name)
				self.get_logger().info(f'Initialized pymoveit2 MoveIt2 for group "{self.group_name}" with base "{base_link}" and ee "{end_effector}"')
			except Exception as e:
				self.get_logger().error(f'Failed to initialize pymoveit2 MoveIt2: {e}')
				self.moveit2 = None

		elif _HAS_MOVEIT_COMMANDER and moveit_commander is not None:
			# Legacy moveit_commander (if someone installed a compatibility layer)
			moveit_commander.roscpp_initialize(sys.argv)
			try:
				self.robot = moveit_commander.RobotCommander()
				self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
				# Apply the initial reference frame
				try:
					self.move_group.set_pose_reference_frame(self.reference_frame)
					self.get_logger().info(f'Set initial pose reference frame to: {self.reference_frame}')
				except Exception as e:
					self.get_logger().warning(f'Could not set reference frame on MoveGroupCommander: {e}')
			except Exception as e:
				self.get_logger().error(f'Failed to initialize MoveIt commanders: {e}')
				self.move_group = None
		else:
			self.get_logger().error('No supported MoveIt Python bindings found (pymoveit2 or moveit_commander). Install ros-humble-pymoveit2 or appropriate bindings.')

		# Register parameter change callback to update the reference frame at runtime
		self.add_on_set_parameters_callback(self._on_set_parameters)

	def _on_set_parameters(self, params):
		"""Handle runtime updates to parameters; specifically reference_frame."""
		for param in params:
			if param.name == 'reference_frame' and param.type_ == Parameter.Type.STRING:
				new_frame = param.value
				self.get_logger().info(f'Request to change reference frame to: {new_frame}')
				if self.move_group is not None:
					try:
						self.move_group.set_pose_reference_frame(new_frame)
						self.get_logger().info(f'Pose reference frame updated to: {new_frame}')
						self.reference_frame = new_frame
					except Exception as e:
						self.get_logger().error(f'Failed to set new reference frame: {e}')
						return rclpy.parameter.SetParametersResult(successful=False)
		return rclpy.parameter.SetParametersResult(successful=True)

	def _discover_joint_names(self, search_roots=None):
		"""Search for URDF/XACRO files under common workspace locations and extract joint names.

		This is a best-effort helper: XACRO files that rely on macro expansion may not contain explicit
		<joint> tags, so discovery may be incomplete. Returns a list of unique joint names in file order.
		"""
		if search_roots is None:
			search_roots = [
				os.path.expanduser('~/ws_fanuc/install'),
				os.path.expanduser('~/ws_fanuc/src'),
				'/root/ws_fanuc/install',
				'/root/ws_fanuc/src',
				'/opt/ros/humble/share',
			]

		found_joints = []
		seen = set()

		# Patterns to search for
		file_patterns = ['**/*.urdf', '**/*.xacro', '**/*.urdf.xacro']

		for root in search_roots:
			if not os.path.isdir(root):
				continue
			for pat in file_patterns:
				glob_path = os.path.join(root, pat)
				for path in glob.glob(glob_path, recursive=True):
					try:
						with open(path, 'r', encoding='utf-8', errors='ignore') as f:
							content = f.read()
							# find <joint ... name="..." ...>
							for m in re.finditer(r'<joint\s+[^>]*name=["\']([^"\']+)["\']', content):
								jn = m.group(1)
								if jn not in seen:
									seen.add(jn)
									found_joints.append(jn)
					except Exception:
						# ignore files we can't read
						continue

		# If no joints found, try a rough fallback: search for 'joint name=' across the whole share path
		if not found_joints:
			for root in search_roots:
				for dirpath, dirnames, filenames in os.walk(root):
					for fname in filenames:
						if fname.endswith(('.urdf', '.xacro', '.urdf.xacro')):
							path = os.path.join(dirpath, fname)
							try:
								with open(path, 'r', encoding='utf-8', errors='ignore') as f:
									content = f.read()
									for m in re.finditer(r'joint\s+name=["\']([^"\']+)["\']', content):
										jn = m.group(1)
										if jn not in seen:
											seen.add(jn)
											found_joints.append(jn)
							except Exception:
								continue

		return found_joints

	def plan_and_execute(self):
		if self.moveit2 is None and self.move_group is None:
			self.get_logger().error('No MoveIt interface initialized (pymoveit2 or moveit_commander). Aborting plan.')
			return False

		# Read the target pose parameters
		x = float(self.get_parameter('target_x').get_parameter_value().double_value)
		y = float(self.get_parameter('target_y').get_parameter_value().double_value)
		z = float(self.get_parameter('target_z').get_parameter_value().double_value)
		roll = float(self.get_parameter('target_roll').get_parameter_value().double_value)
		pitch = float(self.get_parameter('target_pitch').get_parameter_value().double_value)
		yaw = float(self.get_parameter('target_yaw').get_parameter_value().double_value)

		# Construct PoseStamped
		header_frame = self.reference_frame
		if PoseStamped is None:
			self.get_logger().error('geometry_msgs not available. Install ROS2 geometry messages.')
			return False

		target = PoseStamped()
		target.header.frame_id = header_frame
		target.header.stamp = self.get_clock().now().to_msg()
		target.pose.position.x = x
		target.pose.position.y = y
		target.pose.position.z = z
		qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
		target.pose.orientation.x = qx
		target.pose.orientation.y = qy
		target.pose.orientation.z = qz
		target.pose.orientation.w = qw

		self.get_logger().info(f'Planning to pose in frame "{header_frame}": x={x} y={y} z={z} rpy=({roll},{pitch},{yaw})')

		# Use pymoveit2 if available
		try:
			if self.moveit2 is not None:
				# move_to_pose will plan and execute; wait for completion
				self.moveit2.move_to_pose(target, frame_id=header_frame)
				ok = self.moveit2.wait_until_executed()
				if ok:
					self.get_logger().info('pymoveit2: Plan and execution succeeded')
					return True
				else:
					self.get_logger().error('pymoveit2: Execution failed or timed out')
					return False

			# Fallback to moveit_commander style API
			if self.move_group is not None:
				try:
					self.move_group.set_start_state_to_current_state()
				except Exception:
					pass
				try:
					self.move_group.set_pose_target(target)
				except Exception:
					# older wrappers may require different args
					try:
						self.move_group.set_pose_target(target.pose)
					except Exception:
						pass

				plan = None
				try:
					plan = self.move_group.plan()
				except Exception:
					pass

				executed = False
				if plan:
					try:
						if hasattr(self.move_group, 'execute'):
							self.move_group.execute(plan, wait=True)
							executed = True
						elif hasattr(self.move_group, 'go'):
							self.move_group.go(wait=True)
							executed = True
					except Exception as e:
						self.get_logger().warning(f'Execution attempt after plan failed: {e}')

				if not executed:
					try:
						self.get_logger().info('Attempting move_group.go() as fallback')
						self.move_group.go(wait=True)
						executed = True
					except Exception as e:
						self.get_logger().error(f'Fallback go() failed: {e}')

				try:
					self.move_group.clear_pose_targets()
				except Exception:
					pass

				if executed:
					self.get_logger().info('Plan executed (or go() returned).')
					return True
				else:
					self.get_logger().error('Planning/execution did not succeed.')
					return False

		except Exception as e:
			self.get_logger().error(f'Unexpected error during plan_and_execute: {e}')
			return False


def main(args=None):
	rclpy.init(args=args)

	node = RasterPlayground()
	if node.move_group is None:
		# Early exit if MoveIt not available
		node.get_logger().info('Exiting because MoveIt is not initialized (see errors above).')
		rclpy.shutdown()
		return

	# Run a single plan+execute cycle. For continuous control or services, integrate this into
	# a parameter-change or service-based workflow.
	success = node.plan_and_execute()

	# Keep node alive briefly to allow parameter changes (user can Ctrl-C to stop)
	if success:
		node.get_logger().info('Completed one plan_and_execute run. Spinning for parameter updates. Ctrl-C to exit.')
	else:
		node.get_logger().info('Plan failed. Spinning for parameter updates. Use ros2 param set to change parameters and try again.')

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass

	# Clean up
	try:
		moveit_commander.roscpp_shutdown()
	except Exception:
		pass
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
