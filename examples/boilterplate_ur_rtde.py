
import rospy
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import WrenchStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import os
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point32

from std_srvs.srv import Empty

from std_srvs.srv import Empty

from easy_ur.srv import *

rospy.init_node("easy_ur_example")


pos_msg  = PoseStamped()
cur_pos = PoseStamped()
cur_wrench = WrenchStamped()
pose_pub = rospy.Publisher('target_frame',PoseStamped, queue_size = 1)

#set_speed = rospy.Publisher('set_speed',Float32, queue_size = 1)
pos_msg.header.frame_id = "base_link"
pos_seq = 0

wrench_pub = rospy.Publisher('target_wrench',WrenchStamped, queue_size = 1)
wrench_msg = WrenchStamped()
wrench_msg.header.frame_id = "base_link"
wrench_seq = 0

cur_pos = PoseStamped()
cur_wrench = WrenchStamped()

#ur robot stuff
rospy.wait_for_service('/ur_stop')
stop_service = rospy.ServiceProxy('/ur_stop', Trigger)
trigger =  TriggerRequest()
def stop_ur():
    stop_service(trigger)

def set_ur_speed(speed):
	rospy.wait_for_service('/ur_speed')
	try:
		service = rospy.ServiceProxy('/ur_speed', SetSpeed)
		service(speed)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def set_ur_complianceAxes(req):
	rospy.wait_for_service('/ur_compliance_axes')
	try:
		service = rospy.ServiceProxy('/ur_compliance_axes', SetComplianceAxes)
		service(req[0],req[1],req[2],req[3],req[4],req[5])
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def set_ur_complianceLimits(req):
	rospy.wait_for_service('/ur_compliance_limits')
	try:
		service = rospy.ServiceProxy('/ur_compliance_limits', SetComplianceLimits)
		service(req[0],req[1],req[2],req[3],req[4],req[5])
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def apply_wrench(wrench):
    global wrench_seq
    wrench_msg.wrench.force.x,wrench_msg.wrench.force.y,wrench_msg.wrench.force.z, wrench_msg.wrench.torque.x,wrench_msg.wrench.torque.y,wrench_msg.wrench.torque.z = wrench
    wrench_msg.header.seq = wrench_seq
    wrench_seq+=1
    wrench_msg.header.stamp = rospy.get_rostime()
    wrench_pub.publish(wrench_msg)

def move_pos(pos, orn , isBlocking =True):
    global pos_seq
    pos_msg.pose.position.x,pos_msg.pose.position.y,pos_msg.pose.position.z = pos
    pos_msg.pose.orientation.x,pos_msg.pose.orientation.y,pos_msg.pose.orientation.z,pos_msg.pose.orientation.w =orn
    pos_msg.header.seq = pos_seq
    pos_seq+=1
    pos_msg.header.stamp = rospy.get_rostime()
    pose_pub.publish(pos_msg)
    #invert the goal orientation quaternion by negating the w component
    orn_goal = [pos_msg.pose.orientation.x,pos_msg.pose.orientation.y,pos_msg.pose.orientation.z,-pos_msg.pose.orientation.w]
    if (isBlocking == True):
            while not rospy.is_shutdown():
                #compute the position error
                dist_err  = math.sqrt((pos_msg.pose.position.x - cur_pos.pose.position.x)**2 +(pos_msg.pose.position.y - cur_pos.pose.position.y)**2 +(pos_msg.pose.position.z - cur_pos.pose.position.z)**2)
                #compute the orientation error
                # invert the goal orientation quaternion by negating the w component
                #pos_msg.pose.orientation.w = -pos_msg.pose.orientation.w
                curn_orn = [cur_pos.pose.orientation.x,cur_pos.pose.orientation.y,cur_pos.pose.orientation.z,cur_pos.pose.orientation.w]
                orn_err = tf.transformations.quaternion_multiply(curn_orn,orn_goal)# cur_pos.pose.orientation)
                orn_err_eul = euler_from_quaternion(orn_err)
                orn_err_sum = abs(math.degrees(orn_err_eul[0])) + abs(math.degrees(orn_err_eul[1])) + abs(math.degrees(orn_err_eul[2]))
                #print("ERR ",dist_err, orn_err_sum)
                rospy.sleep(0.002)
                if(dist_err<=0.001) and orn_err_sum<=1.0: # 1mm and 1 degree
                    break#print("Errors ",dist_err)

def callback_pos(msg):
    cur_pos.pose = msg.pose

POS1 = [0.156,-0.682,0.12]
POS2 = [0.156,-0.682,0.026]
POS3 = [0.176,-0.682,0.026]
ROT1 = quaternion_from_euler(math.radians(180), 0, math.radians(-90))

def run_pos_test():
    set_ur_speed(0.1)
    move_pos(POS1, ROT1, isBlocking=True)
    move_pos(POS2, ROT1, isBlocking=True)
    move_pos(POS1, ROT1, isBlocking=True)
    set_ur_speed(0.005)
    move_pos(POS1, ROT1, isBlocking=True)
    move_pos(POS2, ROT1, isBlocking=True)
    move_pos(POS1, ROT1, isBlocking=False)
    rospy.sleep(1)
    stop_ur()

def run_wrench_test():
    set_ur_complianceAxes([0,1,0,0,0,0])
    apply_wrench([10,10,0,0,0,0])
    rospy.sleep(3)
    apply_wrench([-10,-10,0,0,0,0])
    rospy.sleep(3)
    stop_ur()

if __name__ == '__main__':
    rospy.Subscriber('/ur_pose', PoseStamped, callback_pos)

    rospy.sleep(0.5)
    run_wrench_test()
    run_pos_test()
    #run_wrench_test()
    #run_pos_test()
