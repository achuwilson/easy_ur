#!/usr/bin/env python3
import rospy
import rtde_control
import time
import rtde_receive
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Float32, Bool, Float64MultiArray
from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose
from scipy.spatial.transform import Rotation as R
from easy_ur.srv import SetSpeed, SetMode, SetAcceleration, SetPose#, SetSpeedResponse
from easy_ur.srv import *


rospy.init_node("UR_RTDE")
ROBOT_IP = "192.168.1.12"
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

print("RTDE Connected to UR at ", ROBOT_IP, " : ", rtde_c.isConnected())

#status = rtde_c.ftRtdeInputEnable(True,0.623,[0,0,0.0],[0,0,0.018])
#print("EXT FT SENSOR INIT STATUS ", status)

MODE = "POSITION"
speed=0.6#0.15
acceleration= 0.35
speed_j = 1.04 #joint speed used in joint control
acceleration_j = 1.4 #joint acceleration used in joint control

wrench_frame = [0, 0, 0, 0, 0, 0] #pose  vector that defines the force frame relative to the base frame.
compliance_axes = [0, 0, 0, 0, 0, 0]
wrench_type = 2 #The force frame is not transformed
wrench_motion_limits = [0.01, 1, 0.02, 1, 1, 1] #. For compliant axes, these values are the maximum allowed tcp speed along/about the axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual tcp position and the one set by the program.


# the data structures for publishing current positin
pos_msg  = PoseStamped()
pose_pub = rospy.Publisher('ur_pose',PoseStamped, queue_size = 1)
pos_msg.header.frame_id = "base_link"
#pos_msg.pose.orientation.x,pos_msg.pose.orientation.y,pos_msg.pose.orientation.z,pos_msg.pose.orientation.w = DEFAULT_ROT
pos_seq = 0

#data structure for publishing wrench_msg
#wrench_pub = rospy.Publisher('ur_wrench',WrenchStamped, queue_size = 1)
#wrench_msg = WrenchStamped()
#wrench_msg.header.frame_id = "base_link"
#wrench_seq = 0

q_pub = rospy.Publisher('ur_joints', Float64MultiArray, queue_size=1)
q_msg = Float64MultiArray()

def callback_pos(data):
    #print("Received ", data)\
    global speed, MODE, acc
    if(MODE=="FORCE"):
        rtde_c.forceModeStop()
    elif(MODE=="SERVOL"):
        rtde_c.servoStop()
    elif(MODE=="FREEDRIVE"):
        print("Change mode from FREEDRIVE mode to POS mode")
        return None
    MODE = "POSITION"
    pos=[data.pose.position.x,data.pose.position.y,data.pose.position.z]
    #print(pos)
    quat = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
    r= R.from_quat(quat)
    r=r.as_rotvec()
    pos.extend(r)
    rtde_c.moveL(pos, speed, acceleration,asynchronous=True)

def callback_servo(data):
    global speed, MODE
    if(MODE=="FORCE"):
        rtde_c.forceModeStop()
    elif(MODE=="FREEDRIVE"):
        print("Change mode from FREEDRIVE mode to SERVO mode")
        return None
    MODE = "SERVOL"
    pos=[data.pose.position.x,data.pose.position.y,data.pose.position.z]
    #print(pos)
    quat = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
    r= R.from_quat(quat)
    r=r.as_rotvec()
    pos.extend(r)

    dt = 1.0/500  # 2ms
    lookahead_time = 0.1
    gain = 300

    rtde_c.servoL(pos,speed, 0.1,dt,lookahead_time, gain)



def callback_wrench(data):
    global MODE
    if(MODE=="SERVOL"):
        rtde_c.servoStop()
    elif(MODE=="FREEDRIVE"):
        print("Change mode from FREEDRIVE mode to FORCE mode")
        return None
    MODE="FORCE"
    wrench = [data.wrench.force.x,data.wrench.force.y,data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y,data.wrench.torque.z]
    rtde_c.forceMode(wrench_frame, compliance_axes, wrench, wrench_type, wrench_motion_limits)

def stop_robot(req):
    if(MODE=="POSITION"):
        rtde_c.stopL(0.5)
    elif(MODE=="JOINT"):
        rtde_c.stopJ(2.0)
    elif(MODE=="FORCE"):
        rtde_c.forceModeStop()
    print("stop called")
    return TriggerResponse(
        success=True,
        message="Robot Stopped"
    )

def set_controlmode(req):
    global MODE
    old_MODE = MODE
    #TODO -  sanity check of MODE values
    MODE=req.mode
    if (old_MODE=="FREEDRIVE") and (MODE!="FREEDRIVE"):
        ret = rtde_c.endTeachMode()
    if (MODE=="FREEDRIVE"):
        ret = rtde_c.teachMode()
    else:
        ret = rtde_c.endTeachMode()  
    return SetModeResponse(
        result=ret
    )

def set_speed(req):
    global speed
    speed = req.speed
    #TODO - Sanity check of values
    print("speed set" ,speed)
    return SetSpeedResponse(
        result=True
    )

def set_acceleration(req):
    global acceleration
    acceleration = req.acceleration
    #TODO - Sanity check of values
    print("speed accelerations " ,acceleration)
    return SetAccelerationResponse(
        result=True
    )
def set_joint_speed(req):
    global speed_j
    speed_j = req.speed
    #TODO - Sanity check of values
    #print("speed joinset" ,speed)
    return SetSpeedResponse(
        result=True
    )

def set_joint_acceleration(req):
    global acceleration_j
    acceleration_j = req.acceleration
    #TODO - Sanity check of values
    #print("speed accelerations " ,acceleration)
    return SetAccelerationResponse(
        result=True
    )
def set_wrench_frame(req):
    global wrench_frame
    #TODO

def set_compliance_axes(req):
    global compliance_axes
    compliance_axes=[req.x,req.y,req.z,req.rx,req.ry,req.rz]
    return SetComplianceAxesResponse(
        result=True
    )

def set_compliance_limits(req):
    global wrench_motion_limits
    wrench_motion_limits = [req.x,req.y,req.z,req.rx,req.ry,req.rz]
    return SetComplianceLimitsResponse(
        result=True
    )

def set_pose(req):
    global speed, MODE, acceleration
    if(MODE=="FORCE"):
        rtde_c.forceModeStop()
    elif(MODE=="SERVOL"):
        rtde_c.servoStop()
    elif(MODE=="FREEDRIVE"):
        print("Change mode from FREEDRIVE mode to POS mode")
        return None
    MODE = "POSITION"
    pos=[req.pose.position.x,req.pose.position.y, req.pose.position.z]
    #print(pos)
    quat = [req.pose.orientation.x,req.pose.orientation.y, req.pose.orientation.z,req.pose.orientation.w ]
    r= R.from_quat(quat)
    r=r.as_rotvec()
    pos.extend(r)
    if(req.async):
        status = rtde_c.moveL(pos, speed, acceleration,asynchronous=True)
    else:
        status = rtde_c.moveL(pos, speed, acceleration,asynchronous=False)
    return SetPoseResponse(result=status)

def set_joints(req):
    global speed_j, MODE, acceleration_j
    if(MODE=="FORCE"):
        rtde_c.forceModeStop()
    elif(MODE=="SERVOL"):
        rtde_c.servoStop()
    elif(MODE=="FREEDRIVE"):
        print("Change mode from FREEDRIVE mode to POS mode")
        return None
    MODE = "JOINT"
    if(req.async):
        status = rtde_c.moveJ(list(req.q.data), speed_j, acceleration_j,asynchronous=True)
    else:
        status = rtde_c.moveJ(list(req.q.data), speed_j, acceleration_j,asynchronous=False)
    return SetJointsResponse(result=status)

def pub():
    global pos_seq, wrench_seq
    rate = rospy.Rate(500)
    while(rospy.is_shutdown()==False):
        #get current pos
        curpos = rtde_r.getActualTCPPose()
        rospy.sleep(0.005)

        #print("wr ", curwrench)
        pos_msg.pose.position.x,pos_msg.pose.position.y,pos_msg.pose.position.z = curpos[:3]
        r = R.from_rotvec(curpos[3:])
        r = r.as_quat()
        pos_msg.pose.orientation.x,pos_msg.pose.orientation.y,pos_msg.pose.orientation.z,pos_msg.pose.orientation.w = r
        pos_msg.header.seq = pos_seq
        pos_seq+=1
        pos_msg.header.stamp = rospy.get_rostime()


        #curwrench = rtde_r.getFtRawWrench()
        #rospy.sleep(0.005)
        #wrench_msg.wrench.force.x,wrench_msg.wrench.force.y,wrench_msg.wrench.force.z, wrench_msg.wrench.torque.x,wrench_msg.wrench.torque.y,wrench_msg.wrench.torque.z = curwrench
        #wrench_msg.header.seq = wrench_seq
        #wrench_seq+=1
        #wrench_msg.header.stamp = rospy.get_rostime()


        q_msg.data = rtde_r.getActualQ() # assign the array with the value you want to send
        q_pub.publish(q_msg)


        #wrench_pub.publish(wrench_msg)

        pose_pub.publish(pos_msg)

        #rate.sleep()
        continue
def callback_joint(msg):
    rtde_c.moveJ(msg.data, speed_j, acceleration_j,asynchronous=True)

if __name__ == '__main__':
    my_service0 = rospy.Service('/ur_mode_cmd', SetMode, set_controlmode)
    my_service1 = rospy.Service('/ur_speed_cmd', SetSpeed, set_speed)
    my_service2 = rospy.Service('/ur_stop_cmd', Trigger, stop_robot)
    #my_service3 = rospy.Service('/ur_compliance_axes', SetComplianceAxes, set_compliance_axes)
    #my_service4 = rospy.Service('/ur_compliance_limits', SetComplianceLimits, set_compliance_limits)
    my_service5 = rospy.Service('/ur_acceleration_cmd', SetAcceleration, set_acceleration)
    my_service6 = rospy.Service('/ur_pose_cmd', SetPose, set_pose)
    my_service7 = rospy.Service('/ur_joints_cmd', SetJoints, set_joints)
    my_service8 = rospy.Service('/ur_joint_acceleration_cmd', SetAcceleration, set_joint_acceleration)
    my_service9 = rospy.Service('/ur_joint_speed_cmd', SetSpeed, set_joint_speed)

    #TODO -  add custom wrench frame service types
    #my_service5 = rospy.Service('/ur_wrench_frame', SetWrenchFrame, set_wrench_frame)
    #rospy.Subscriber('/target_frame', PoseStamped, callback_pos)
    rospy.Subscriber('/ur_servo_cmd', PoseStamped, callback_servo)
    #rospy.Subscriber('/target_wrench',WrenchStamped, callback_wrench)
    #rospy.Subscriber('/target_joints',Float64MultiArray, callback_joint)

    #my_service3 = rospy.Service('/apply_force', Trigger, apply_force)
    #my_service4 = rospy.Service('/apply_forced', Trigger, apply_forced)
    pub()
    #rospy.spin()
