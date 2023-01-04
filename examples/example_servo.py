import rospy
from easy_ur.srv import *

from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point32

from std_srvs.srv import Empty
import numpy as np

from std_srvs.srv import Empty

rospy.init_node("example_servo")

cur_pos = PoseStamped()
servo_msg  = PoseStamped()

servo_pub = rospy.Publisher('target_servo',PoseStamped, queue_size = 1)
servo_msg.header.frame_id = "base_link"
servo_seq = 0

rospy.wait_for_service('/ur_stop')
rospy.wait_for_service("/ur_pose")

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

def set_ur_acceleration(acceleration):
    rospy.wait_for_service('/ur_acceleration')
    try:
        service = rospy.ServiceProxy('/ur_acceleration', SetAcceleration)
        service(acceleration)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

servo_seq = 0
def servo_pos(pos, orn):
    global servo_seq
    servo_msg.pose.position.x,servo_msg.pose.position.y,servo_msg.pose.position.z = pos
    servo_msg.pose.orientation.x,servo_msg.pose.orientation.y,servo_msg.pose.orientation.z,servo_msg.pose.orientation.w =orn
    servo_msg.header.seq = servo_seq
    servo_seq+=1
    servo_msg.header.stamp = rospy.get_rostime()
    servo_pub.publish(servo_msg)

def callback_pos(msg):
    cur_pos.pose = msg.pose#.x




def set_pose(pose, orn):
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
        service(pose_cmd)
    except:
        print("Set Pose failed")


def generate_spiral(POS):
    maxradius = 0.01 #1.5 cm radius
    gap = 0.002 #5 mm
    numpoints = int(maxradius/gap)
    x_origin  = POS[0]
    y_origin = POS[1]
    z_origin  = POS[2]

    #maxradius = 0.03
    #gap = 0.005
    numsteps = 20#int(maxradius/gap)
    numpoints = 1000
    angle = np.linspace(0,numsteps*np.pi,numpoints)
    radius = np.linspace(0,maxradius, numpoints)
    #TODO-  change axis assignment based on the orientation of surface
    y = y_origin + radius * np.cos(angle)
    z =  z_origin + radius * np.sin(angle)
    #clip z axis
    z[z<0.014] = 0.014
    z[z>0.032] = 0.032
    x = np.ones(numpoints)*x_origin
    return x,y,z

def run_demo():
    #rospy.sleep(1)
    #quaternion_from_euler(math.radians(180), math.radians(0), math.radians(-90))
    start_pos = [cur_pos.pose.position.x, cur_pos.pose.position.y,cur_pos.pose.position.z]
    start_orn = [cur_pos.pose.orientation.x,cur_pos.pose.orientation.y,cur_pos.pose.orientation.z,cur_pos.pose.orientation.w]
    set_ur_speed(0.5)
    set_ur_acceleration(0.35)
    #move up 5 cm
    set_pose([start_pos[0], start_pos[1],start_pos[2]+0.05],start_orn)

    #servo down 10 cm
    POS1 =  [cur_pos.pose.position.x, cur_pos.pose.position.y,cur_pos.pose.position.z]
    POS2 =   [cur_pos.pose.position.x, cur_pos.pose.position.y,cur_pos.pose.position.z-0.1]

    dist = np.linalg.norm(np.array(POS1)-np.array(POS2))
    num_points = int(dist/0.00055)
    lin_path = np.linspace(POS1,POS2,num_points)

    step_count=0
    t_delay=0.05
    print("Num_points = ", num_points)
    while step_count<num_points:

        servo_pos(lin_path[step_count],start_orn)
        #if (ft_z< -15):
        #    print("FT ", ft_z)
        #   rospy.sleep(t_delay)
        #   break
        if(step_count==150):
            rospy.sleep(t_delay)# use delay before breaking
            break
        rospy.sleep(t_delay)
        step_count=step_count+1

    set_pose([start_pos[0], start_pos[1],start_pos[2]],start_orn)


if __name__ == '__main__':

    rospy.Subscriber('/ur_pose', PoseStamped, callback_pos)

    rospy.sleep(0.5)
    set_ur_speed(0.6)
    set_ur_acceleration(0.35)

    run_demo()
