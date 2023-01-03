from tkinter import *
import time
import tkinter
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
import math
from wsg_50_common.srv import *
from wsg_50_common.msg import Status
from std_srvs.srv import Empty
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import os
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point32
from easy_ur.srv import *

import numpy as np

rospy.init_node('tkinterGUI', anonymous=True)
rospy.sleep(0.2)

pos_msg  = PoseStamped()
pose_pub = rospy.Publisher('target_frame',PoseStamped, queue_size = 1)
pos_msg.header.frame_id = "base_link"
#pos_msg.pose.orientation.x,pos_msg.pose.orientation.y,pos_msg.pose.orientation.z,pos_msg.pose.orientation.w = DEFAULT_ROT
pos_seq = 0

def set_ur_mode(mode):
	rospy.wait_for_service('/ur_mode')
	try:
		service = rospy.ServiceProxy('/ur_mode', SetMode)
		service(mode)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


########################## Gripper Stuff ###############################
pub_gspeed  =rospy.Publisher('/wsg_50_driver/goal_speed', Float32, queue_size=2)
G_MAX_POS = 2.5#2.8
G_MIN_POS = 0
def gripper_homing():
    rospy.wait_for_service('/wsg_50_driver/homing')
    try:
        service = rospy.ServiceProxy('/wsg_50_driver/homing', Empty)
        service()
        tare_markers()
        tare_cable()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
def set_gripper_force(force):
    rospy.wait_for_service('/wsg_50_driver/set_force')
    try:
        service = rospy.ServiceProxy('/wsg_50_driver/set_force', Conf)
        service(force)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
def set_gripper_speed(speed):
    pub_gspeed.publish(speed)

grip_force_measured = 0
grip_vel_measured = 0
grip_pos_measured = 0

def callback_gripper_status(status):
    global grip_force_measured
    global grip_vel_measured
    global grip_pos_measured
    grip_force_measured = status.force
    grip_vel_measured = status.speed
    grip_pos_measured = status.width

grip_force_cmd = 28

def gripper_gripF():
    g_speed = -20
    cnt=  0
    set_gripper_speed(g_speed)
    rospy.sleep(0.15)
    while True:
        rospy.sleep(0.02)
        if(abs(grip_vel_measured)<=2.0):
            #rospy.sleep(0.65)
            #if(grip_force_measured>=grip_force_cmd):
            set_gripper_speed(-2)
            print("gripped")
            break
        else:
            cnt = cnt+1
            set_gripper_speed(g_speed-cnt)

def gripper_releaseF():
    set_gripper_speed(10)
    while True:
        rospy.sleep(0.02)
        if(grip_force_measured<=0.50):
            #print("R1")
            rospy.sleep(0.35)
            if(grip_force_measured<=grip_force_cmd/2.0):
                set_gripper_speed(0)
                print("released")
                break

isGripped = False
def gripper_grip():
    global isGripped, G_MIN_POS
    tare_markers()
    g_speed = -25
    set_gripper_speed(g_speed)
    rospy.sleep(0.25)
    if(isGripped):
        print("Already gripped")
        tare_markers()
    else:
        while True:
            if(abs(markermag)>=1.0):
                print("grip made contact")
                #set_gripper_speed(-1)
                set_gripper_speed(0)
                rospy.sleep(0.5)
                G_MIN_POS = grip_pos_measured
                isGripped = True
                break
            if(abs(grip_vel_measured)<2.0):
                #set_gripper_speed(-1)
                set_gripper_speed(0)
                rospy.sleep(0.5)
                G_MIN_POS = grip_pos_measured
                print("grip made contactv")
                isGripped = True
                break

    tare_markers()

def gripper_release():
    global isGripped
    if(abs(grip_pos_measured-G_MIN_POS)>=G_MAX_POS):
        print("Gripper already at maxpos ", grip_pos_measured)
        isGripped=False
        tare_markers()
    else:
        tare_markers()
        g_speed = 3
        set_gripper_speed(g_speed)
        rospy.sleep(0.2)
        while True:
            if(abs(markermag)>=1.0):
                print("grip released contact")
                set_gripper_speed(0)
                set_gripper_speed(0)
                isGripped = False
                break
            if(abs(grip_pos_measured-G_MIN_POS)>=G_MAX_POS):
                print("reached gripper_maxpos")
                set_gripper_speed(0)
                set_gripper_speed(0)
                isGripped = False
                break
        tare_markers()

def gripper_loosen():
    tare_markers()
    g_speed = 2
    set_gripper_speed(g_speed)
    rospy.sleep(0.2)
    while True:
        if(abs(grip_pos_measured-G_MIN_POS)>=G_MAX_POS):
            print("reached gripper_maxpos")
            set_gripper_speed(0)
            set_gripper_speed(0)
            isGripped = False
            break
        rospy.sleep(0.025)
    tare_markers()

############## marker stuff #########################
############## marker stuff #########################
pub_mgrad  =rospy.Publisher('/markers/grad', Float32, queue_size=2)

from std_srvs.srv import Trigger, TriggerRequest
print("waiting for GelWedge Services")
#rospy.wait_for_service('/calibrate_markers')
#rospy.wait_for_service('/cabletrack_init')
print("Gelwedge services ready")
marker_service = rospy.ServiceProxy('/calibrate_markers', Trigger)
cable_service = rospy.ServiceProxy('/cabletrack_init', Trigger)
trigger =  TriggerRequest()

def tare_markers():
    marker_service(trigger)
def tare_cable():
    cable_service(trigger)
cable_center = Point32()
def callback_cable_center(msg):
    global cable_center
    cable_center  = msg
cable_angle = 90
def callback_cable_angle(msg):
    global cable_angle
    cable_angle  = msg.data
markermag=0
firstrun=True
markermag_array =[]
markermag_grad = 0
def callback_markermag(msg):
    global markermag, markermag_array,markermag_grad, firstrun
    markermag = msg.data
    if (firstrun ==True):
        for i in range(20):
            markermag_array.append(markermag)
        firstrun = False
    else:
        markermag_array.append(markermag)
        markermag_array.pop(0)
        markermag_grad = np.diff(markermag_array)[-1]
        gr_=round(markermag_grad,2)
        pub_mgrad.publish(gr_)


markerang=0
def callback_markerang(msg):
    global markerang
    markerang = msg.data
cableArea = 0
def callback_cableArea(msg):
    global cableArea
    cableArea = msg.data
u_sum=0
def callback_usum(msg):
    global u_sum
    u_sum = msg.data

v_sum=0
def callback_vsum(msg):
    global v_sum
    v_sum = msg.data

uv_sum=0
def callback_uvsum(msg):
    global uv_sum
    uv_sum = msg.data

uv_rms=0
def callback_uvrms(msg):
    global uv_rms
    uv_rms = msg.data

def bound(low, high, value):
    return max(low, min(high, value))
#############################################################################SSS

top = tkinter.Tk()
top.title('EASY UR')
top.geometry("500x600+300+300")
top.resizable(False,False)
px=0
py=0
pz=0
rx=0
ry=0
rz=0
DEFAULT_pdelta = 0.005 #5mm
MAX_pdelta=0.05 #50mm
p_delta = DEFAULT_pdelta

DEFAULT_rdelta = math.radians(2) #5mm
MAX_rdelta=math.radians(15) #50mm
r_delta = DEFAULT_rdelta

def xplus():
    global px,py,pz, rx,ry,rz
    x=px+get_pdelta()
    publish_pos(x, py, pz,rx,ry,rz)
def xminus():
    global px,py,pz, rx,ry,rz
    x=px-get_pdelta()
    publish_pos(x, py, pz,rx,ry,rz)

def yplus():
    global px,py,pz, rx,ry,rz
    y=py+get_pdelta()
    publish_pos(px, y, pz,rx,ry,rz)
def yminus():
    global px,py,pz, rx,ry,rz
    y=py-get_pdelta()
    publish_pos(px, y, pz,rx,ry,rz)
def zplus():
    global px,py,pz, rx,ry,rz
    z=pz+get_pdelta()
    publish_pos(px, py, z,rx,ry,rz)
def zminus():
    global px,py,pz, rx,ry,rz
    z=pz-get_pdelta()
    publish_pos(px, py, z,rx,ry,rz)
#rotation stuff
def rxplus():
    global px,py,pz, rx,ry,rz
    x=rx+get_rdelta()
    publish_pos(px, py, pz,x,ry,rz)
def rxminus():
    global px,py,pz, rx,ry,rz
    x=rx-get_rdelta()
    publish_pos(px, py, pz,x,ry,rz)
def ryplus():
    global px,py,pz, rx,ry,rz
    y=ry+get_rdelta()
    publish_pos(px, py, pz,rx,y,rz)
def ryminus():
    global px,py,pz, rx,ry,rz
    y=ry-get_rdelta()
    publish_pos(px, py, pz,rx,y,rz)
def rzplus():
    global px,py,pz, rx,ry,rz
    z=rz+get_rdelta()
    publish_pos(px, py, pz,rx,ry,z)
def rzminus():
    global px,py,pz, rx,ry,rz
    z=rz-get_rdelta()
    publish_pos(px, py, pz,rx,ry,z)

def publish_pos(px,py,pz,rx,ry,rz):
    global pos_seq
    pos_msg.header.seq = pos_seq
    pos_seq+=1
    pos_msg.header.stamp = rospy.get_rostime()
    pos_msg.pose.position.x,pos_msg.pose.position.y,pos_msg.pose.position.z = [px,py,pz]
    r = R.from_euler('xyz',[rx,ry,rz])
    quat = r.as_quat()
    pos_msg.pose.orientation.x,pos_msg.pose.orientation.y,pos_msg.pose.orientation.z,pos_msg.pose.orientation.w =quat

    pose_pub.publish(pos_msg)


def noactn(lb1=None):
    print("noactn")

def get_pdelta():
    global p_delta
    pdelta = E1.get()
    try:
        p_delta = float(pdelta)
    except:
        p_delta = DEFAULT_pdelta
    if(p_delta>MAX_pdelta): #max delta, 5 cm
        p_delta = MAX_pdelta
    return p_delta

def get_rdelta():
    global r_delta
    rdelta = E2.get()
    try:
        r_delta = float(rdelta)
    except:
        r_delta = DEFAULT_rdelta
    if(r_delta>MAX_rdelta): #max delta, 5 cm
        r_delta = MAX_rdelta
    return r_delta
def pos_mode():
    global B0, B1, B2
    set_ur_mode("POS")
    B0.configure(bg="green")
    B1.configure(bg="#f0f0f0")
    B2.configure(bg="#f0f0f0")


def teach_mode():
    global B0, B1, B2
    set_ur_mode("TEACH")
    B0.configure(bg="#f0f0f0")
    B1.configure(bg="#f0f0f0")
    B2.configure(bg="green")
label1 = Label(top,text="CONTROL MODE").place(x=180,y=30)
B0=tkinter.Button(top, text="POSITION CONTROL", command=pos_mode)
B0.place(x=20,y=60)

B1=tkinter.Button(top, text="FORCE CONTROL", command=noactn)
B1.place(x=180,y=60)

B2=tkinter.Button(top, text="FREEDRIVE", command=teach_mode)
B2.place(x=330,y=60)

#positioning buttons
By1 = tkinter.Button(top, text="Y+", command=yplus, width=3)
By1.place(x=65,y=100)
By2 = tkinter.Button(top, text="Y-", command=yminus, width=3)
By2.place(x=65,y=160)

Bx1 = tkinter.Button(top, text="X-", command=xminus, width=3)
Bx1.place(x=12,y=130)
Bx2 = tkinter.Button(top, text="X+", command=xplus, width=3)
Bx2.place(x=114,y=130)

Bz1 = tkinter.Button(top, text="Z+", command=zplus, width=3)
Bz1.place(x=125,y=95)
Bz2 = tkinter.Button(top, text="Z-", command=zminus, width=3)
Bz2.place(x=125,y=165)

#positioning delta:
E1=Entry(top,width=5)
E1.place(x=65,y=133)
#rotation delta:
E2=Entry(top,width=5)
E2.place(x=65,y=283)

#rotation buttons
#positioning buttons
By3 = tkinter.Button(top, text="Y+", command=ryplus, width=3)
By3.place(x=65,y=250)
By4 = tkinter.Button(top, text="Y-", command=ryminus, width=3)
By4.place(x=65,y=310)

Bx3 = tkinter.Button(top, text="X-", command=rxminus, width=3)
Bx3.place(x=12,y=280)
Bx4 = tkinter.Button(top, text="X+", command=rxplus, width=3)
Bx4.place(x=114,y=280)

Bz3 = tkinter.Button(top, text="Z+", command=rzplus, width=3)
Bz3.place(x=125,y=245)
Bz4 = tkinter.Button(top, text="Z-", command=rzminus, width=3)
Bz4.place(x=8,y=245)


#position feedback labels
label_px = Label(top, text="x: ")
label_px.place(x=20, y=450)
label_py= Label(top, text="y: ")
label_py.place(x=20, y=470)
label_pz = Label(top, text="z: ")
label_pz.place(x=20, y=490)

label_rx = Label(top, text="rx: ")
label_rx.place(x=20, y=510)
label_ry= Label(top, text="ry: ")
label_ry.place(x=20, y=530)
label_rz = Label(top, text="rz: ")
label_rz.place(x=20, y=550)

#GRIPPER BUTTONS
G1=tkinter.Button(top, text="GRIPPER HOMING", command=gripper_homing)
G1.place(x=230,y=130)

G2=tkinter.Button(top, text="GRIPPER GRIP", command=gripper_grip)
G2.place(x=230,y=170)


G3=tkinter.Button(top, text="GRIPPER RELEASE", command=gripper_release)
G3.place(x=230,y=220)

G4=tkinter.Button(top, text="GRIPPER LOOSEN", command=gripper_loosen)
G4.place(x=230,y=270)

M1=tkinter.Button(top, text="CALIBRATE_MARKER", command=tare_markers)
M1.place(x=230,y=320)

M2=tkinter.Button(top, text="CALIBRATE CABLE", command=tare_cable)
M2.place(x=230,y=370)

def updateloop():
    global px,py,pz, rx,ry,rz
    #print("updateloop ", time.time())
    #update labels
    label_px.config(text="x: "+str(px))
    label_py.config(text="y: "+str(py))
    label_pz.config(text="z: "+str(pz))
    label_rx.config(text="rx: "+str(round(math.degrees(rx),3)))
    label_ry.config(text="ry: "+str(round(math.degrees(ry),3)))
    label_rz.config(text="rz: "+str(round(math.degrees(rz),3)))
    #print(rz)
    top.after(1,updateloop)

top.bind("<Up>", noactn)

def callback_cur_pose(data):
    global px, py, pz,rx,ry,rz
    #print(data.pose.position.x)
    px = round(data.pose.position.x,3)
    py = round(data.pose.position.y,3)
    pz = round(data.pose.position.z,3)
    quat = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
    r= R.from_quat(quat)
    rx,ry,rz = r.as_euler('xyz')
    rx = round(rx,3)
    ry = round(ry,3)
    rz = round(rz,3)


#define ros subscribers before main loop starts
rospy.Subscriber("/ur_pose_fb", PoseStamped, callback_cur_pose)
rospy.Subscriber('/markers/percentile', Float32, callback_markermag)
rospy.Subscriber('/markers/angle', Float32, callback_markerang)
rospy.Subscriber('/markers/uv_sum', Float32, callback_uvsum)
rospy.Subscriber('/markers/u_sum', Float32, callback_usum)
rospy.Subscriber('/markers/v_sum', Float32, callback_vsum)
rospy.Subscriber('/markers/uv_rms', Float32, callback_uvrms)
rospy.Subscriber('/cable/center', Point32, callback_cable_center)
rospy.Subscriber('/cable/angle', Float32, callback_cable_angle)
rospy.sleep(0.5)

top.after(1,updateloop)
top.mainloop()
