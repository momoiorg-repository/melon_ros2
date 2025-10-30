"""
Copyright (c) 2025 SSatoya

This file is based on the following file from the PyMoveIt2 project:
  https://github.com/AndrejOrsula/pymoveit2/blob/main/examples/ex_pose_goal.py

SPDX-License-Identifier: Apache-2.0

Modifications by SSatoya:
- Adapted for the Melon robot configuration

Description:
 This script demonstrates how to send a pose goal to MoveIt 2 using pymoveit2,
 with parameters for Cartesian planning and optional asynchronous execution.

Parameters (passed via ROS 2 parameters):
- position: [x, y, z] in meters (relative to the base link)
- quat_xyzw: [x, y, z, w] quaternion representing orientation
- cartesian: bool, if True, use Cartesian planning
- synchronous: bool, if True, waits until execution finishes
- cancel_after_secs: float, cancels asynchronous execution after given seconds

Example usage:
1) Move synchronously with default parameters:
   ros2 run melon_demo manipulation_demo --ros-args \
    -p position:="[1.0, 0.0, 1.0]" \
    -p quat_xyzw:="[1.0, 0.0, 0.0, 0.0]" \
    -p cartesian:=False

2) Move asynchronously and cancel after 1 second:
   ros2 run melon_demo manipulation_demo --ros-args \
    -p position:="[1.0, 0.0, 1.0]" \
    -p quat_xyzw:="[1.0, 0.0, 0.0, 0.0]" \
    -p cartesian:=False \
    -p synchronous:=False \
    -p cancel_after_secs:=1.0
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from . import melon as robot  # Import Melon robot configuration

def main():
    # Initialize the ROS 2 node
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [1.0, 0.0, 1.0])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    # If True, wait for the operation to complete before proceeding (synchronous execution)
    node.declare_parameter("synchronous", True)
    # If non-positive, don't cancel. Only used if synchronous is False
    node.declare_parameter("cancel_after_secs", 0.0)
    # Planner ID (Path Planning Algorithm)
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    # Declare parameters for cartesian planning
    node.declare_parameter("cartesian", False)  # 
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    # Set the robot joint names, base link, end effector names, and MoveIt group name (import from melon.py).
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    # 0.5 means 50% of the maximum velocity/acceleration
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = (
        node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    )
    cartesian_fraction_threshold = (
        node.get_parameter("cartesian_fraction_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_jump_threshold = (
        node.get_parameter("cartesian_jump_threshold")
        .get_parameter_value()
        .double_value
    )
    cartesian_avoid_collisions = (
        node.get_parameter("cartesian_avoid_collisions")
        .get_parameter_value()
        .bool_value
    )

    # Set parameters for cartesian planning
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        cartesian=cartesian,
        cartesian_max_step=cartesian_max_step,
        cartesian_fraction_threshold=cartesian_fraction_threshold,
    )

    # Synchronous or asynchronous execution processing
    if synchronous:
        # Note: the same functionality can be achieved by setting
        # `synchronous:=false` and `cancel_after_secs` to a negative value.
        # In this case, we wait until the execution is complete here.
        moveit2.wait_until_executed()
    else:
        # Wait for the request to get accepted (i.e., for execution to start)
        print("Current State: " + str(moveit2.query_state()))
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        # Get the future
        print("Current State: " + str(moveit2.query_state()))
        future = moveit2.get_execution_future()

        # Cancel the goal
        if cancel_after_secs > 0.0:
            # Sleep for the specified time
            sleep_time = node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            # Cancel the goal
            print("Cancelling goal")
            moveit2.cancel_execution()

        # Wait until the future is done
        while not future.done():
            rate.sleep()

        # Print the result
        print("Result status: " + str(future.result().status))
        print("Result error code: " + str(future.result().result.error_code))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
