import sys
import bosdyn.client
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, blocking_sit
import msvcrt
import time

import bosdyn.client.util
from bosdyn.api import basic_command_pb2
from bosdyn.api import geometry_pb2 as geo
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b)
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_for_trajectory_cmd, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient

# ---------------------
sdk = bosdyn.client.create_standard_sdk('understanding-spot')
robot = sdk.create_robot('192.168.80.3')
robot.authenticate('user', 'hhe262iz9zez')

# ---------------------
lease_client = robot.ensure_client('lease')
##lease = lease_client.return_lease(lease)
lease = lease_client.acquire()
lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)

# ---------------------
robot.power_on(timeout_sec=20)
robot.time_sync.wait_for_sync()

# ---------------------
# Setup clients for the robot state and robot command services.
robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

# ---------------------
"""Command the robot to go to an offset position using a trajectory command."""

def relative_move(dx, dy, dyaw, frame_name, robot_command_client, robot_state_client, stairs=False):
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    # Command the robot to go to the goal point in the specified frame. The command will stop at the
    # new position.
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    end_time = 10.0
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)

    return True

# ---------------------
while True:
    user_input = msvcrt.getch().decode("utf-8")

    if user_input.upper() == 'Q':
        lease_client.return_lease(lease)
        break
    elif user_input.upper() == 'O':
        blocking_stand(robot_command_client, timeout_sec=10)
    elif user_input.upper() == 'L':
        blocking_sit(robot_command_client, timeout_sec=10)
    elif user_input.upper() == 'A':
        relative_move(0, 0.5, 0, ODOM_FRAME_NAME, robot_command_client, robot_state_client, stairs=False)
    elif user_input.upper() == 'D':
        relative_move(0, -0.5, 0, ODOM_FRAME_NAME, robot_command_client, robot_state_client, stairs=False)
    elif user_input.upper() == 'S':
        relative_move(-0.5, 0, 0, ODOM_FRAME_NAME, robot_command_client, robot_state_client, stairs=False)
    elif user_input.upper() == 'W':
        relative_move(0.5, 0, 0, ODOM_FRAME_NAME, robot_command_client, robot_state_client, stairs=False)
    elif user_input.upper() == 'U':
        relative_move(0, 0, 0.3, ODOM_FRAME_NAME, robot_command_client, robot_state_client, stairs=False)
    elif user_input.upper() == 'I':
        relative_move(0, 0, -0.3, ODOM_FRAME_NAME, robot_command_client, robot_state_client, stairs=False)
    else:
        print("No input provided.")