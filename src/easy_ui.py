from tkinter import *
import time
import tkinter
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import *
from scipy.spatial.transform import Rotation as R
import math
from easy_ur.srv import *
import numpy as np

rospy.init_node('tkinterGUI', anonymous=True)
rospy.sleep(0.2)

pos_msg  = PoseStamped()
pose_pub = rospy.Publisher('target_frame',PoseStamped, queue_size = 1)
pos_msg.header.frame_id = "base_link"
#pos_msg.pose.orientation.x,pos_msg.pose.orientation.y,pos_msg.pose.orientation.z,pos_msg.pose.orientation.w = DEFAULT_ROT

q_pub = rospy.Publisher('target_joints', Float64MultiArray, queue_size=10)
q_msg = Float64MultiArray()

pos_seq = 0

def set_ur_mode(mode):
	rospy.wait_for_service('/ur_mode')
	try:
		service = rospy.ServiceProxy('/ur_mode', SetMode)
		service(mode)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


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
cur_q = None
DEFAULT_pdelta = 0.005 #5mm
MAX_pdelta=0.05 #50mm
p_delta = DEFAULT_pdelta

DEFAULT_rdelta = math.radians(2) #5mm
MAX_rdelta=math.radians(15) #50mm
r_delta = DEFAULT_rdelta

q_delta = 1.0
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

#joint control buttons
Bj0p = tkinter.Button(top, text="J0-", command=lambda: pub_q_delta([-1,0,0,0,0,0]), width=3)
Bj0p.place(x=250,y=120)
Bj0n = tkinter.Button(top, text="J0+", command=lambda: pub_q_delta([1,0,0,0,0,0]), width=3)
Bj0n.place(x=350,y=120)

Bj1p = tkinter.Button(top, text="J1-", command=lambda: pub_q_delta([0,-1,0,0,0,0]), width=3)
Bj1p.place(x=250,y=150)
Bj1n = tkinter.Button(top, text="J1+", command=lambda: pub_q_delta([0,1,0,0,0,0]), width=3)
Bj1n.place(x=350,y=150)

Bj2p = tkinter.Button(top, text="J2-", command=lambda: pub_q_delta([0,0,-1,0,0,0]), width=3)
Bj2p.place(x=250,y=180)
Bj2n = tkinter.Button(top, text="J2+", command=lambda: pub_q_delta([0,0,1,0,0,0]), width=3)
Bj2n.place(x=350,y=180)

Bj3p = tkinter.Button(top, text="J3-", command=lambda: pub_q_delta([0,0,0,-1,0,0]), width=3)
Bj3p.place(x=250,y=210)
Bj3n = tkinter.Button(top, text="J3+", command=lambda: pub_q_delta([0,0,0,1,0,0]), width=3)
Bj3n.place(x=350,y=210)

Bj4p = tkinter.Button(top, text="J4-", command=lambda: pub_q_delta([0,0,0,0,-1,0]), width=3)
Bj4p.place(x=250,y=240)
Bj4n = tkinter.Button(top, text="J4+", command=lambda: pub_q_delta([0,0,0,0,1,0]), width=3)
Bj4n.place(x=350,y=240)

Bj5p = tkinter.Button(top, text="J5-", command=lambda: pub_q_delta([0,0,0,0,0,-1]), width=3)
Bj5p.place(x=250,y=270)
Bj5n = tkinter.Button(top, text="J5+", command=lambda: pub_q_delta([0,0,0,0,0,1]), width=3)
Bj5n.place(x=350,y=270)

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

def callback_cur_q(msg):
    global cur_q
    cur_q = msg.data
def pub_q_delta(delta):
    #print("QDELTA ", np.array(delta)* math.radians(q_delta))
    next_q = np.array(cur_q)+ np.array(delta)* math.radians(q_delta)
    print("QNEXT ", next_q)
    q_msg.data = next_q # assign the array with the value you want to send
    q_pub.publish(q_msg)

#define ros subscribers before main loop starts
rospy.Subscriber("/ur_pose", PoseStamped, callback_cur_pose)
rospy.Subscriber("/ur_joints", Float64MultiArray, callback_cur_q)
top.after(1,updateloop)
top.mainloop()
