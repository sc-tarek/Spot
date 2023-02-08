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

# Imports for live view
import logging
import cv2
import numpy as np
from scipy import ndimage

from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.time_sync import TimedOutError

# ---------------------
sdk = bosdyn.client.create_standard_sdk('understanding-spot')
robot = sdk.create_robot('192.168.80.3')
robot.authenticate('user', 'hhe262iz9zez')

# ---------------------
lease_client = robot.ensure_client('lease')
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
"""Live image display

This code creates an object sdk of the bosdyn.client.standard_sdk class, which is used to communicate with a robot at "localhost." 
The code then creates a robot object using the create_robot method of sdk, authenticates the robot, synchronizes its time with the directory, and creates an image_client object for communicating with the image service.

The code prepares a list of requests to capture images from the image_sources list. 
Each request specifies a source and the desired quality percent of the captured image.

Finally, the code creates a window using OpenCV's cv2.namedWindow function for each image_source in image_sources. 
The window will display the image captured from the corresponding source.
"""

_LOGGER = logging.getLogger(__name__)

VALUE_FOR_Q_KEYSTROKE = 113
VALUE_FOR_ESC_KEYSTROKE = 27

ROTATION_ANGLE = {
    'back_fisheye_image': 0,
    'frontleft_fisheye_image': -78,
    'frontright_fisheye_image': -102,
    'left_fisheye_image': 0,
    'right_fisheye_image': 180
}

def image_to_opencv(image, auto_rotate=True):
    """Convert an image proto message to an openCV image."""
    num_channels = 1  # Assume a default of 1 byte encodings.
    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        dtype = np.uint16
        extension = ".png"
    else:
        dtype = np.uint8
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            num_channels = 3
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            num_channels = 4
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            num_channels = 1
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
            num_channels = 1
            dtype = np.uint16
        extension = ".jpg"

    img = np.frombuffer(image.shot.image.data, dtype=dtype)
    if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
        try:
            # Attempt to reshape array into a RGB rows X cols shape.
            img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_channels))
        except ValueError:
            # Unable to reshape the image data, trying a regular decode.
            img = cv2.imdecode(img, -1)
    else:
        img = cv2.imdecode(img, -1)

    if auto_rotate:
        img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name])

    return img, extension


def reset_image_client(robot):
    """Recreate the ImageClient from the robot object."""
    del robot.service_clients_by_name['image']
    del robot.channels_by_authority['api.spot.robot']
    return robot.ensure_client('image')


# def main():
    # Parameters
image_sources = ["frontleft_fisheye_image", "frontright_fisheye_image "]
#image_service = ImageClient.default_service_name
jpeg_quality_percent = 90
capture_delay = 10
disable_full_screen = True
auto_rotate = False

image_client = robot.ensure_client(ImageClient.default_service_name)
requests = [
    build_image_request(source, quality_percent=jpeg_quality_percent)
    for source in image_sources
]

for image_source in image_sources:
    cv2.namedWindow(image_source, cv2.WINDOW_NORMAL)
    if len(image_sources) > 1 or disable_full_screen == True:
        cv2.setWindowProperty(image_source, cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE)
    else:
        cv2.setWindowProperty(image_source, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

keystroke = None
timeout_count_before_reset = 0
while keystroke != VALUE_FOR_Q_KEYSTROKE and keystroke != VALUE_FOR_ESC_KEYSTROKE:
    try:
        images_future = image_client.get_image_async(requests, timeout=0.5)
        while not images_future.done():
            keystroke = cv2.waitKey(25)
            print(keystroke)
            if keystroke == VALUE_FOR_ESC_KEYSTROKE or keystroke == VALUE_FOR_Q_KEYSTROKE:
                sys.exit(1)
        images = images_future.result()
    except TimedOutError as time_err:
        if timeout_count_before_reset == 5:
            # To attempt to handle bad comms and continue the live image stream, try recreating the
            # image client after having an RPC timeout 5 times.
            _LOGGER.info("Resetting image client after 5+ timeout errors.")
            image_client = reset_image_client(robot)
            timeout_count_before_reset = 0
        else:
            timeout_count_before_reset += 1
    except Exception as err:
        _LOGGER.warning(err)
        continue
    for i in range(len(images)):
        image, _ = image_to_opencv(images[i], auto_rotate)
        cv2.imshow(images[i].source.name, image)
    keystroke = cv2.waitKey(capture_delay)


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
