import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose
import math
import numpy as np
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool, Float64MultiArray
from geometry_msgs.msg import Point32
from std_srvs.srv import Empty
from easy_ur.srv import *
import time

import threading
import signal

class UR:
    def __init__(self):
        rospy.init_node("easy_ur")


        self.cur_pos = PoseStamped()
        self.cur_joint_pos =None
        self.servo_msg  = PoseStamped()

        self.servo_pub = rospy.Publisher('/ur_servo_cmd',PoseStamped, queue_size = 1)
        self.servo_msg.header.frame_id = "base_link"
        self.servo_seq = 0

        rospy.sleep(0.1)

        rospy.Subscriber("/ur_joints", Float64MultiArray, self.callback_joint_pos)
        rospy.Subscriber('/ur_pose', PoseStamped, self.callback_ee_pos)
        rospy.sleep(0.1)

        self.exit_event = threading.Event()
        signal.signal(signal.SIGINT, self.signal_handler)
        th = threading.Thread(target=self.thread_function)
        th.daemon=True
        th.start()
        print("started thread")

    def thread_function(self):
        rate = rospy.Rate(200)
        while(rospy.is_shutdown()==False):
            if self.exit_event.is_set():
                break
            else:
                rate.sleep()

    def signal_handler(self,signum, frame):
        self.exit_event.set()

    def callback_ee_pos(self,msg):
        #print("callback called 1 ", time.time(), msg.pose.position.x)
        self.cur_pos.pose = msg.pose

        #print ("PP", self.cur_pos.pose.position.x,self.cur_pos.pose.position.y,self.cur_pos.pose.position.z)
    def callback_joint_pos(self,msg):
        #print("callback called 2 ", time.time())
        self.cur_joint_pos = msg.data

    def set_pose(self,pose, orn, async=False):
        rospy.wait_for_service("/ur_pose_cmd")
        try:
            service = rospy.ServiceProxy("/ur_pose_cmd", SetPose)
            pose_cmd = Pose()
            pose_cmd.position.x = pose[0]
            pose_cmd.position.y = pose[1]
            pose_cmd.position.z = pose[2]
            pose_cmd.orientation.x = orn[0]
            pose_cmd.orientation.y = orn[1]
            pose_cmd.orientation.z = orn[2]
            pose_cmd.orientation.w = orn[3]

            ret = service(pose_cmd, async)
            return ret.result
        except:
            print("Set Pose failed")
            return False

    def get_pose(self):
        return([self.cur_pos.pose.position.x,
                self.cur_pos.pose.position.y,
                self.cur_pos.pose.position.z],
                [self.cur_pos.pose.orientation.x,
                self.cur_pos.pose.orientation.y,
                self.cur_pos.pose.orientation.z,
                self.cur_pos.pose.orientation.w] )

    def set_ee_speed(self, speed):
        rospy.wait_for_service('/ur_speed_cmd')
        try:
            service = rospy.ServiceProxy('/ur_speed_cmd', SetSpeed)
            ret = service(speed)
            return ret.result
        except:
            print("Set Speed Failed")
            return False

    def set_ee_acceleration(self, acceleration):
        rospy.wait_for_service('/ur_acceleration_cmd')
        try:
            service = rospy.ServiceProxy('/ur_acceleration_cmd', SetAcceleration)
            ret = service(acceleration)
            return ret.result
        except:
            print("Set Acceleration Failed")
            return False
    
     def set_joint_speed(self, speed):
        rospy.wait_for_service('/ur_joint_speed_cmd')
        try:
            service = rospy.ServiceProxy('/ur_joint_speed_cmd', SetSpeed)
            ret = service(speed)
            return ret.result
        except:
            print("Set Joint Speed Failed")
            return False

    def set_joint_acceleration(self, acceleration):
        rospy.wait_for_service('/ur_joint_acceleration_cmd')
        try:
            service = rospy.ServiceProxy('/ur_joint_acceleration_cmd', SetAcceleration)
            ret = service(acceleration)
            return ret.result
        except:
            print("Set Joint Acceleration Failed")
            return False

    def get_joint_positions(self):
        return list(self.cur_joint_pos)

    def set_joint_positions(self,joint_cmd, async=False):
        rospy.wait_for_service("/ur_joints_cmd")
        try:
            service = rospy.ServiceProxy("/ur_joints_cmd", SetJoints)
            joints = Float64MultiArray()
            joints.data = joint_cmd
            ret = service(joints, async)
            return ret.result
        except:
            print("Set Joints failed")
            return False

    def stop(self):
        rospy.wait_for_service("/ur_stop_cmd")
        try:
            service = rospy.ServiceProxy("/ur_stop_cmd", Trigger)
            trigger =  TriggerRequest()
            ret = service(trigger)
            return ret.success
        except:
            print("STOP Failed")
            return False

    def set_mode(self, mode):
        rospy.wait_for_service('/ur_mode_cmd')
        try:
            service = rospy.ServiceProxy('/ur_mode_cmd', SetMode)
            print("setting mode", mode)
            ret = service(mode)
            print(ret)
            return ret.result
        except:
            print("setmode failed")
            return False

    def freedrive(self, val):
        if(val):
            ret = self.set_mode("FREEDRIVE")
        else:
            ret = self.set_mode("POSITION")

    def servo(self, pos, orn):
        self.servo_msg.pose.position.x,self.servo_msg.pose.position.y,self.servo_msg.pose.position.z = pos
        self.servo_msg.pose.orientation.x,self.servo_msg.pose.orientation.y,self.servo_msg.pose.orientation.z,self.servo_msg.pose.orientation.w =orn
        self.servo_msg.header.seq = self.servo_seq
        self.servo_seq+=1
        self.servo_msg.header.stamp = rospy.get_rostime()
        self.servo_pub.publish(self.servo_msg)
