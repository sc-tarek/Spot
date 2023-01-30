import sys
import bosdyn.client

sdk = bosdyn.client.create_standard_sdk('understanding-spot')
robot = sdk.create_robot('192.168.80.3')
robot.authenticate('user', 'hhe262iz9zez')

lease_client = robot.ensure_client('lease')
#lease = lease_client.return_lease(lease)
lease = lease_client.acquire()
lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)

robot.power_on(timeout_sec=20)
robot.is_powered_on()

robot.time_sync.wait_for_sync()

from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, blocking_sit, blocking_selfright
command_client = robot.ensure_client(RobotCommandClient.default_service_name)

while True:
    user_input = input("Enter W to stand, S to sit or 'Q' to quit: ")

    if user_input.upper() == 'Q':
        break
    elif user_input.upper() == 'W':
        blocking_stand(command_client, timeout_sec=10)
    elif user_input.upper() == 'S':
        blocking_sit(command_client, timeout_sec=10)
    else:
        print("No input provided.")