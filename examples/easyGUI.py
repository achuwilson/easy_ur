# Sample Python application demonstrating the
# working of FloatLayout in Kivy

# sudo apt-get install xclip
# python3 -m pip install kivy

import kivy

# base Class of your App inherits from the App class.
# app:always refers to the instance of your application
from kivy.app import App

# creates the button in kivy
# if not imported shows the error
from kivy.uix.button import Button
from kivy.uix.textinput import TextInput
from kivy.uix.label import Label
# module consist the floatlayout
# to work with FloatLayout first
# you have to import it
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
# To change the kivy default settings
# we use this module config
from kivy.config import Config
from kivy.core.window import Window
Window.minimum_height = 450
Window.minimum_width = 700
Window.size = (850, 550)
#Window.clearcolor = (179.0/255.0, 179.0/255.0, 179.0/255.0, 1.0)
#Window.clearcolor=(0.6,0.8,0.9,1.0)
Window.clearcolor=(0.5, 0.5, 0.5, 0.50)
from kivy.clock import Clock

import math
from scipy.spatial.transform import Rotation as R
import time 
import numpy as np
from easyUR import UR
robot=None
q_delta = 1.0
pos_delta = 0.01
orn_delta= 1.0
# 0 being off 1 being on as in true / false
# you can use 0 or 1 && True or False
#Config.set('graphics', 'resizable', True)

btn_zp=None
btn_fd=None
lblp1=None
lblp2=None
lblp3=None
lbl_j0 = None
lbl_j1=None
lbl_j2=None
lbl_j3=None
lbl_j4=None
lbl_j5=None
txtin_q=None
txtin_pos=None
txtin_orn=None

FREEDRIVE_FLAG=False
c_pos = None
c_orn = None
c_q = None

def bound(value, low, high):
    return max(low, min(high, value))


def clk(val, inst):
    global btn_zp
    print("clicked ",val)# inst, dir(inst.get_parent_window()))
    #btn.disabled=True
    #inst.background_color=(0.9, 0.8, 0, 0.9)
    btn_zp.background_color=(255/255, 213/255, 200/255, 1)
    #inst.get_parent_window()#.btn.background_color=(0.9, 0.8, 0, 0.9)
    #pass

def timedAction(dt):
    global r_cnt, lblp1, lblp2, lblp3
    global robot, c_pos, c_orn, c_q
    global lbl_j0, lbl_j1, lbl_j2, lbl_j3, lbl_j4, lbl_j5
    

    c_pos, c_orn = robot.get_pose()
    r= R.from_quat(c_orn)
    rx,ry,rz = r.as_euler('xyz')
    rx = format(math.degrees(rx),'.3f')
    ry = format(math.degrees(ry),'.3f')
    rz = format(math.degrees(rz),'.3f')

    qx = format(c_orn[0],'.3f')
    qy = format(c_orn[1],'.3f')
    qz = format(c_orn[2],'.3f')
    qw = format(c_orn[3],'.3f')

    c_q = robot.get_joint_positions()

    #print("timedaction called2 ", time.time(), c_pos)

   
    lblp1.text= "[color= 00]"+"x : "+ "[/color]"+format(c_pos[0], '.3f') + '\n'+ \
              "[color=00]"+"y : "+"[/color]"+ format(c_pos[1], '.3f') + '\n'+\
              "[color=00]"+"z : "+"[/color]"+ format(c_pos[2], '.3f') + '\n'

    lblp2.text= "[color=00]"+"rx : "+"[/color]"+ rx + '\n' +\
              "[color=00]"+"ry : "+"[/color]"+ ry + '\n' +\
              "[color=00]"+"rz : "+"[/color]"+ rz + '\n'

    lblp3.text= "[color=00]"+"qx : "+"[/color]"+ qx + '\n' +\
              "[color=00]"+"qy : "+"[/color]"+ qy + '\n' +\
              "[color=00]"+"qz : "+"[/color]"+ qz + '\n'+\
              "[color=00]"+"qw : "+"[/color]"+ qw + '\n'

    lbl_j0.text = format(math.degrees(c_q[0]),'.2f')
    lbl_j1.text = format(math.degrees(c_q[1]),'.2f')
    lbl_j2.text = format(math.degrees(c_q[2]),'.2f')
    lbl_j3.text = format(math.degrees(c_q[3]),'.2f')
    lbl_j4.text = format(math.degrees(c_q[4]),'.2f')
    lbl_j5.text = format(math.degrees(c_q[5]),'.2f')

    #print("Repeat ", dt)
def drivemode(val, inst):
    global FREEDRIVE_FLAG, robot
    if FREEDRIVE_FLAG==False:
        robot.freedrive(True)
        FREEDRIVE_FLAG=True
        btn_fd.color=(0.01,0.85,0.3,1.0)
        #btn_fd.background_color=(0,0.9,1.0, 1.0)
        btn_fd.text ='FREE DRIVE\n       ON  '
    else:
        robot.freedrive(False)
        FREEDRIVE_FLAG=False
        #btn_fd.background_color=(0.6,0.8,0.9,1.0)
        btn_fd.color=(0.031, 0.741, 0.682, 0.5)
        btn_fd.text ='FREE DRIVE\n       OFF'

# creating the App class
class EasyGUIApp(App):

    def build(self):
        print("starting drawing UI elements")
        u_t0 = time.time()
        global btn_zp, lblp1, lblp2,lblp3, btn_fd
        global robot
        global lbl_j0, lbl_j1, lbl_j2, lbl_j3, lbl_j4, lbl_j5
        global txtin_q, txtin_orn, txtin_pos

        # creating Floatlayout
        Fl = FloatLayout()

        # creating button
        # a button 30 % of the width and 20 %
        # of the height of the layout and
        # positioned at (300, 200), you can do:
        btn_zp = Button(text ='Z+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*10)+0.0075},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_yp = Button(text ='Y+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": 0.0833*9},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_xn = Button(text ='X-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.07, "y": 0.0833*8},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),

                    )
        btn_xp = Button(text ='X+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.23, "y": 0.0833*8},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_yn = Button(text ='Y-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": 0.0833*7},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_zn = Button(text ='Z-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*6)-0.0075},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )

        btn_rzp = Button(text ='RZ+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*4)+0.0075+0.02},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_ryp = Button(text ='RY+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*3)+0.02},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_rxn = Button(text ='RX-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.07, "y": (0.0833*2)+0.02},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_rxp = Button(text ='RX+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.23, "y": (0.0833*2)+0.02},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_ryn = Button(text ='RX+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*1)+0.02},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_rzn = Button(text ='RZ-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": 0.00+0.02-0.005},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )

        btn_j0n = Button(text ='J0-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  0.0833*9},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j0p = Button(text ='J0+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  0.0833*9},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j1n = Button(text ='J1-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*8)-0.0075},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j1p = Button(text ='J1+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*8)-0.0075},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j2n = Button(text ='J2-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*7)-(0.0075*2)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j2p = Button(text ='J2+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*7)-(0.0075*2)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j3n = Button(text ='J3-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*6)-(0.0075*3)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j3p = Button(text ='J3+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*6)-(0.0075*3)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j4n = Button(text ='J4-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*5)-(0.0075*4)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j4p = Button(text ='J4+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*5)-(0.0075*4)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j5n = Button(text ='J5-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*4)-(0.0075*5)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )
        btn_j5p = Button(text ='J5+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*4)-(0.0075*5)},
                    color=(0.031, 0.741, 0.682, 1),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    )


        txtin_pos = TextInput(multiline=False,
                        size_hint =(.1, 0.06),
                        pos_hint= {"x": 0.015, "y":(0.0833*10)+0.0075},
                        hint_text='DELTA POS',
                        input_filter = 'float',
                        write_tab=False,
                        )
        txtin_orn = TextInput(multiline=False,
                        size_hint =(.1, 0.06),
                        pos_hint= {"x": 0.015, "y": (0.0833*4)+0.0075+0.02},
                        hint_text='DELTA ORN',
                        input_filter = 'float',
                        write_tab=False,
                        )
        txtin_q = TextInput(multiline=False,
                        size_hint =(.1, 0.06),
                        pos_hint= {"x": 0.5, "y": (0.0833*10)+0.0075+0.02},
                        hint_text='DELTA Q',
                        input_filter = 'float',
                        write_tab=False,
                        )
        lblp1 = Label(text="",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.25, "y": 0.52},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lblp2 = Label(text="",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.28, "y": 0.05},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lblp3 = Label(text="",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": 0.055},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lbl_j0 = Label(text="-180.25",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": (0.0833*9)+0.0075},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lbl_j1 = Label(text="199.25",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": (0.0833*8)-0.0075},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lbl_j2 = Label(text="129.25",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": (0.0833*7)-(0.0075*2)},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lbl_j3 = Label(text="189.25",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": (0.0833*6)-(0.0075*3)},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lbl_j4 = Label(text="29.5",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": (0.0833*5)-(0.0075*4)},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )
        lbl_j5 = Label(text="1.2",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": (0.0833*4)-(0.0075*5)},
                    markup=True,
                    font_size= 20,
                    halign="left"
                    )

        lbl0 = Label(text="[color= 0]"+"[b]END EFFECTOR CONTROL[/b] "+"[/color]",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.15, "y": 0.925},
                    markup=True,
                    font_size= 22
                    )
        lbl1 = Label(text="[color=0]"+"[b]JOINT CONTROL[/b] "+"[/color]",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": 0.925},
                    markup=True,
                    font_size= 22
                    )
        btn_fd = Button(text ='FREE DRIVE\n       OFF',
                    size_hint =(.2, 0.18),
                    bold=True,
                    pos_hint= {"x": 0.75, "y": 0.425},
                    color= (0.031, 0.741, 0.682, 0.5),
                    font_size= 20,
                    background_color=(0.6,0.8,0.9,1.0),
                    halign="left"
                    )
        #print(dir(delta_pos))
        #btn2 = Button(text ='Hello world2',
        #            size_hint =(.3, .2),
        #            pos_hint= {"x": 0.25, "y": 0.8})
        #btn2 = Button(text ='Hello world2',
        #            size_hint =(.3, .2),
        #            pos =(300, 300))

        # adding widget i.e button
        #btn_zp.bind(on_release=lambda *args: clk('btn000', *args))
        Fl.add_widget(btn_zp)
        Fl.add_widget(btn_yp)
        Fl.add_widget(btn_xn)
        Fl.add_widget(btn_xp)
        Fl.add_widget(btn_yn)
        Fl.add_widget(btn_zn)
        btn_xp.bind(on_release=lambda *args:self.set_pos([1,0,0]))
        btn_xn.bind(on_release=lambda *args:self.set_pos([-1,0,0]))
        btn_yp.bind(on_release=lambda *args:self.set_pos([0,1,0]))
        btn_yn.bind(on_release=lambda *args:self.set_pos([0,-1,0]))
        btn_zp.bind(on_release=lambda *args:self.set_pos([0,0,1]))
        btn_zn.bind(on_release=lambda *args:self.set_pos([0,0,-1]))

        Fl.add_widget(btn_rzp)
        Fl.add_widget(btn_ryp)
        Fl.add_widget(btn_rxn)
        Fl.add_widget(btn_rxp)
        Fl.add_widget(btn_ryn)
        Fl.add_widget(btn_rzn)
        btn_rxp.bind(on_release=lambda *args:self.set_orn([1,0,0]))
        btn_rxn.bind(on_release=lambda *args:self.set_orn([-1,0,0]))
        btn_ryp.bind(on_release=lambda *args:self.set_orn([0,1,0]))
        btn_ryn.bind(on_release=lambda *args:self.set_orn([0,-1,0]))
        btn_rzp.bind(on_release=lambda *args:self.set_orn([0,0,1]))
        btn_rzn.bind(on_release=lambda *args:self.set_orn([0,0,-1]))
        
        #JOINT CONTROL BUTTONS
        Fl.add_widget(btn_j0n)
        Fl.add_widget(btn_j0p)
        Fl.add_widget(btn_j1n)
        Fl.add_widget(btn_j1p)
        Fl.add_widget(btn_j2n)
        Fl.add_widget(btn_j2p)
        Fl.add_widget(btn_j3n)
        Fl.add_widget(btn_j3p)
        Fl.add_widget(btn_j4n)
        Fl.add_widget(btn_j4p)
        Fl.add_widget(btn_j5n)
        Fl.add_widget(btn_j5p)
        
        btn_j0p.bind(on_release=lambda *args: self.set_joints([1,0,0,0,0,0]))
        btn_j0n.bind(on_release=lambda *args: self.set_joints([-1,0,0,0,0,0]))
        btn_j1p.bind(on_release=lambda *args: self.set_joints([0,1,0,0,0,0]))
        btn_j1n.bind(on_release=lambda *args: self.set_joints([0,-1,0,0,0,0]))
        btn_j2p.bind(on_release=lambda *args: self.set_joints([0,0,1,0,0,0]))
        btn_j2n.bind(on_release=lambda *args: self.set_joints([0,0,-1,0,0,0]))
        btn_j3p.bind(on_release=lambda *args: self.set_joints([0,0,0,1,0,0]))
        btn_j3n.bind(on_release=lambda *args: self.set_joints([0,0,0,-1,0,0]))
        btn_j4p.bind(on_release=lambda *args: self.set_joints([0,0,0,0,1,0]))
        btn_j4n.bind(on_release=lambda *args: self.set_joints([0,0,0,0,-1,0]))
        btn_j5p.bind(on_release=lambda *args: self.set_joints([0,0,0,0,0,1]))
        btn_j5n.bind(on_release=lambda *args: self.set_joints([0,0,0,0,0,-1]))
        


        Fl.add_widget(lbl_j0)
        Fl.add_widget(lbl_j1)
        Fl.add_widget(lbl_j2)
        Fl.add_widget(lbl_j3)
        Fl.add_widget(lbl_j4)
        Fl.add_widget(lbl_j5)

        Fl.add_widget(txtin_pos)
        Fl.add_widget(txtin_orn)
        Fl.add_widget(txtin_q)
        Fl.add_widget(lblp1)
        Fl.add_widget(lblp2)
        Fl.add_widget(lblp3)
        Fl.add_widget(lbl0)
        Fl.add_widget(lbl1)
        Fl.add_widget(btn_fd)

        btn_fd.bind(on_release=lambda *args: drivemode('btnfd', *args))
        timer = Clock.schedule_interval(timedAction, 0.075)

        #Fl.add_widget(btn2)

        

        # return the layout
        print("Done drawing UI elements", time.time()-u_t0)
        return Fl

 
    def set_joints(self,delta):
        global c_q, q_delta, txtin_q
        #get the delta from text input and validate
        try:
            q_delta = bound(float(txtin_q.text), 0.5, 15)
        except:
            q_delta = 1.0   
        #print("Set Joints ", q_delta)
        next_q = np.array(c_q)+ np.array(delta)* math.radians(q_delta)
        #print(next_q)
        robot.set_joint_positions(next_q)#, async=True)

    def set_pos(self, delta):
        global c_pos, pos_delta, txtin_pos, c_orn
        print("pressed pos ", delta, c_pos)
        try:
            pos_delta = bound(float(txtin_pos.text), 0.0005, 0.12)
        except:
            pos_delta = 0.01
        #print("pos delta", np.array(c_pos)+(np.array(delta)*pos_delta))
        next_pose = np.array(c_pos)+np.array(delta)*pos_delta
        robot.set_pose(next_pose, c_orn)
        #pass 
    def set_orn(self, delta):
        global c_pos, orn_delta, txtin_orn, c_orn
        try:
            orn_delta = bound(float(txtin_orn.text), 0.5, 15)
        except:
            orn_delta = 1.0
        r= R.from_quat(c_orn)
        rot_eul = r.as_euler('xyz')

        rx,ry,rz= np.array(rot_eul)+ (np.array(delta)*math.radians(orn_delta))
        r2 = R.from_euler('xyz',[rx,ry,rz])
        quat = r2.as_quat()
        robot.set_pose(c_pos, quat)
    


# run the App
if __name__ == "__main__":
    print("STRATING ROBOT")
    robot=UR()
    robot.freedrive(False)
    print("STARTING UI")

    app=EasyGUIApp()
    print("Running App")
    app.run()
