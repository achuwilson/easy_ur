from easyUR import UR
import math
import time
import numpy as np

robot=UR()



#################### EE POSITION CONTROL ######################
current_position, current_orientation = robot.get_pose()

print("current robot position :",current_position)
print("current robot orientation :",current_orientation)
#increment x axis
next_position = [current_position[0]+0.05, current_position[1], current_position[2]]
#move to position
robot.set_pose(next_position, current_orientation)
#move asyncronously -  it returns control immediately and doesnt wait until
#you reach the position -  you can use
robot.set_pose(current_position, current_orientation, async = True)
time.sleep(1.0)
#during asynchronous move, you can stop robot motion in between
robot.stop()

#set end effector speed and acceleration to a low value and move
#values are in m/s and m/s2
print("LOW SPEED EE MOVE")
robot.set_ee_speed(0.05)
robot.set_ee_acceleration(0.2)
robot.set_pose(next_position, current_orientation)
robot.set_pose(current_position, current_orientation)

#set end effector speed and acceleration to a high value and move
print("HIGH SPEED EE MOVE")
robot.set_ee_speed(0.5)
robot.set_ee_acceleration(1.5)
robot.set_pose(next_position, current_orientation)
robot.set_pose(current_position, current_orientation)

##set end effector speed and acceleration to a low value
robot.set_ee_speed(0.1)
robot.set_ee_acceleration(0.5)

################## JOINT CONTROL ##########################
print("JOINT CONTROL MODE")
#get the joint positions
joint_poses = robot.get_joint_positions()
print("Current joint positions", joint_poses)
#increment the last axis
next_q = joint_poses
next_q[5] = next_q[5] + math.radians(20)
#set joint positions
robot.set_joint_positions(next_q)
next_q[5] = next_q[5] - math.radians(20)
robot.set_joint_positions(next_q, async=True)
time.sleep(1.0)
#during asynchronous move, you can stop robot motion in between
robot.stop()

#joint speed control
print("LOW SPEED JOINT MOVE")
#values are in radians/s and radians/s2
robot.set_joint_speed(0.1)
robot.set_joint_acceleration(0.5)
#increment the last axis
next_q = joint_poses
next_q[5] = next_q[5] + math.radians(20)
#set joint positions
robot.set_joint_positions(next_q)
next_q[5] = next_q[5] - math.radians(20)
robot.set_joint_positions(next_q, async=True)
time.sleep(1.0)
#during asynchronous move, you can stop robot motion in between
robot.stop()



################## FREEDRIVE MODE ##############################

robot.freedrive(True)
print("FREEDRIVE MODE ON")
time.sleep(0.5)
for i in range(1000):
	time.sleep(0.01)
	print(robot.get_pose()[0])
	#print(robot.get_joint_positions())

robot.freedrive(False)
print("FREEDRIVE MODE OFF")

################################ SERVOING #################################
print("SERVO MODE ON")
#get current pos
current_position, current_orientation = robot.get_pose()
next_position = [current_position[0]+0.1, current_position[1], current_position[2]]

dist = np.linalg.norm(np.array(current_position)-np.array(next_position))
num_points = int(dist/0.00055)
lin_path = np.linspace(current_position,next_position,num_points)

step_count=0
t_delay=0.05
print("Num_points Servo= ", num_points)
while step_count<num_points:
    robot.servo(lin_path[step_count],current_orientation)
    time.sleep(t_delay)
    step_count=step_count+1
    print("servo move ", step_count)
    if(step_count==150):
        time.sleep(t_delay)# use delay before breaking
        print("breaking")
        break

############################### Moving Back #################################
time.sleep(2)
print("moving to start pos")
robot.set_pose(current_position, current_orientation)

