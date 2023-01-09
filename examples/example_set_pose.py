from easyUR import UR
import math
import time
robot=UR()
current_position, current_orientation = robot.get_pose()

print("current robot position :",current_position)
print("current robot orientation :",current_orientation)

next_position = [current_position[0]+0.05, current_position[1], current_position[2]]
robot.set_pose(next_position, current_orientation)
robot.set_pose(current_position, current_orientation, async = True)
time.sleep(4)

'''
robot.set_ee_speed(0.1)
robot.set_ee_acceleration(0.5)
robot.set_pose(next_position, current_orientation)
robot.set_pose(current_position, current_orientation)

robot.set_ee_speed(0.5)
robot.set_ee_acceleration(1.5)
robot.set_pose(next_position, current_orientation)
robot.set_pose(current_position, current_orientation)
'''
joint_poses = robot.get_joint_positions()
print("Current joint positions", joint_poses)

next_q = joint_poses
next_q[5] = next_q[5] + math.radians(20)
robot.set_joint_positions(next_q, async=True)
time.sleep(4)
next_q[5] = next_q[5] - math.radians(20)
robot.set_joint_positions(next_q)
