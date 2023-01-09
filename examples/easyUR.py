import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose
import math
import numpy as np
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool, Float64MultiArray
from geometry_msgs.msg import Point32
from std_srvs.srv import Empty
from easy_ur.srv import *

class UR:
    def __init__(self):
        rospy.init_node("easy_ur")

        self.pos_msg  = PoseStamped()
        self.cur_pos = PoseStamped()
        self.cur_wrench = WrenchStamped()
        self.pose_pub = rospy.Publisher('target_frame',PoseStamped, queue_size = 1)

        #set_speed = rospy.Publisher('set_speed',Float32, queue_size = 1)
        self.pos_msg.header.frame_id = "base_link"
        self.pos_seq = 0

        self.wrench_pub = rospy.Publisher('target_wrench',WrenchStamped, queue_size = 1)
        self.wrench_msg = WrenchStamped()
        self.wrench_msg.header.frame_id = "base_link"
        self.wrench_seq = 0

        self.cur_pos = PoseStamped()
        self.cur_wrench = WrenchStamped()
        self.cur_joint_pos =None

        #ur robot stuff
        rospy.wait_for_service('/ur_stop')
        self.stop_service = rospy.ServiceProxy('/ur_stop', Trigger)
        self.trigger =  TriggerRequest()
        rospy.Subscriber('/ur_pose', PoseStamped, self.callback_ee_pos)
        rospy.Subscriber("/ur_joints", Float64MultiArray, self.callback_joint_pos)
        rospy.sleep(0.5)


    def callback_ee_pos(self,msg):
        self.cur_pos.pose = msg.pose#.x
    def callback_joint_pos(self,msg):
        self.cur_joint_pos = msg.data

    def set_pose(self,pose, orn, async=False):
        rospy.wait_for_service("/ur_pose")
        try:
            service = rospy.ServiceProxy("/ur_pose", SetPose)
            pose_cmd = Pose()
            pose_cmd.position.x = pose[0]
            pose_cmd.position.y = pose[1]
            pose_cmd.position.z = pose[2]
            pose_cmd.orientation.x = orn[0]
            pose_cmd.orientation.y = orn[1]
            pose_cmd.orientation.z = orn[2]
            pose_cmd.orientation.w = orn[3]

            service(pose_cmd, async)
        except:
            print("Set Pose failed")
    def get_pose(self):
        return([self.cur_pos.pose.position.x,
                self.cur_pos.pose.position.y,
                self.cur_pos.pose.position.z],
                [self.cur_pos.pose.orientation.x,
                self.cur_pos.pose.orientation.y,
                self.cur_pos.pose.orientation.z,
                self.cur_pos.pose.orientation.w] )

    def set_ee_speed(self, speed):
        rospy.wait_for_service('/ur_speed')
        try:
            service = rospy.ServiceProxy('/ur_speed', SetSpeed)
            service(speed)
        except:
            print("Set Speed Failed")

    def set_ee_acceleration(self, acceleration):
        rospy.wait_for_service('/ur_acceleration')
        try:
            service = rospy.ServiceProxy('/ur_acceleration', SetAcceleration)
            service(acceleration)
        except:
            print("Set Acceleration Failed")

    def get_joint_positions(self):
        return list(self.cur_joint_pos)
    def set_joint_positions(self,joint_cmd, async=False):
        rospy.wait_for_service("/set_ur_joints")
        try:
            service = rospy.ServiceProxy("/set_ur_joints", SetJoints)
            joints = Float64MultiArray()
            joints.data = joint_cmd

            service(joints, async)
        except:
            print("Set Joints failed")
