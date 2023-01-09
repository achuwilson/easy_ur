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
Window.minimum_height = 600
Window.minimum_width = 800
Window.size = (1000, 600)
Window.clearcolor = (0.1, 0.1, 0.1, 1.0)
from kivy.clock import Clock


# 0 being off 1 being on as in true / false
# you can use 0 or 1 && True or False
#Config.set('graphics', 'resizable', True)

btn_zp=None
btn_fd=None
lbl=None
FREEDRIVE_FLAG=False
def clk(val, inst):
    global btn_zp
    print("clicked ",val)# inst, dir(inst.get_parent_window()))
    #btn.disabled=True
    #inst.background_color=(0.9, 0.8, 0, 0.9)
    btn_zp.background_color=(48/255, 213/255, 200/255, 1)
    #inst.get_parent_window()#.btn.background_color=(0.9, 0.8, 0, 0.9)
    #pass
r_cnt=0
def timedAction(dt):
    global r_cnt, lbl
    r_cnt =  r_cnt+1
    val  = r_cnt/1000.0 - 1.2
    val = format(val, '.3f')
    lbl.text= "[color=FFFF00]"+"X : "+"[/color]"+ val + '\n'+ \
              "[color=FFFF00]"+"Y : "+"[/color]"+ val + '\n'+\
              "[color=FFFF00]"+"Z : "+"[/color]"+ val + '\n' + '\n' + \
              "[color=FFFF00]"+"RZ : "+"[/color]"+ val + '\n' +\
              "[color=FFFF00]"+"RY : "+"[/color]"+ val + '\n' +\
              "[color=FFFF00]"+"RX : "+"[/color]"+ val + '\n'
    #print("Repeat ", dt)
def drivemode(val, inst):
    global FREEDRIVE_FLAG
    if FREEDRIVE_FLAG==False:
        FREEDRIVE_FLAG=True
        btn_fd.background_color=(48/255, 213/255, 200/255, 1)
        btn_fd.text ='FREE DRIVE ON'
    else:
        FREEDRIVE_FLAG=False
        btn_fd.background_color=(200/255, 13/255, 200/255, 1)
        btn_fd.text ='FREE DRIVE OFF'

# creating the App class
class EasyGUIApp(App):

    def build(self):
        global btn_zp, lbl, btn_fd

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
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_yp = Button(text ='Y+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": 0.0833*9},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),
                    )
        btn_xn = Button(text ='X-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.07, "y": 0.0833*8},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),

                    )
        btn_xp = Button(text ='X+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.23, "y": 0.0833*8},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),
                    )
        btn_yn = Button(text ='Y-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": 0.0833*7},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),
                    )
        btn_zn = Button(text ='Z-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*6)-0.0075},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )

        btn_rzp = Button(text ='RZ+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*4)+0.0075+0.02},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_ryp = Button(text ='RY+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*3)+0.02},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),
                    )
        btn_rxn = Button(text ='RX-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.07, "y": (0.0833*2)+0.02},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),
                    )
        btn_rxp = Button(text ='RX+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.23, "y": (0.0833*2)+0.02},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),
                    )
        btn_ryn = Button(text ='RX+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": (0.0833*1)+0.02},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 1.0, 1.0, 1.0),
                    )
        btn_rzn = Button(text ='RZ-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.15, "y": 0.00+0.02-0.005},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )

        btn_j0n = Button(text ='J0-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  0.0833*9},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j0p = Button(text ='J0+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  0.0833*9},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j1n = Button(text ='J1-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*8)-0.0075},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j1p = Button(text ='J1+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*8)-0.0075},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j2n = Button(text ='J2-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*7)-(0.0075*2)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j2p = Button(text ='J2+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*7)-(0.0075*2)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j3n = Button(text ='J3-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*6)-(0.0075*3)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j3p = Button(text ='J3+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*6)-(0.0075*3)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j4n = Button(text ='J4-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*5)-(0.0075*4)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j4p = Button(text ='J4+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*5)-(0.0075*4)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j5n = Button(text ='J5-',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.415, "y":  (0.0833*4)-(0.0075*5)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )
        btn_j5p = Button(text ='J5+',
                    size_hint =(.0833, 0.0833),

                    bold=True,
                    pos_hint= {"x": 0.6, "y":  (0.0833*4)-(0.0075*5)},
                    color=(255 / 255, 255 / 255, 255 / 255, 1),
                    font_size= 20,
                    background_color=(0.2, 0.5, 1.0, 1.0),
                    )


        delta_pos = TextInput(multiline=False,
                        size_hint =(.1, 0.06),
                        pos_hint= {"x": 0.015, "y":(0.0833*10)+0.0075},
                        hint_text='DELTA POS',
                        input_filter = 'float',
                        write_tab=False,
                        )
        delta_orn = TextInput(multiline=False,
                        size_hint =(.1, 0.06),
                        pos_hint= {"x": 0.015, "y": (0.0833*4)+0.0075+0.02},
                        hint_text='DELTA ORN',
                        input_filter = 'float',
                        write_tab=False,
                        )
        delta_q = TextInput(multiline=False,
                        size_hint =(.1, 0.06),
                        pos_hint= {"x": 0.5, "y": (0.0833*10)+0.0075+0.02},
                        hint_text='DELTA Q',
                        input_filter = 'float',
                        write_tab=False,
                        )
        lbl = Label(text="[color=FFFF00]"+"X : "+"[/color]"+" 1233.2",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.25, "y": 0.42},
                    markup=True,
                    font_size= 25
                    )

        lbl0 = Label(text="[color=FFFFFF]"+"[b]END EFFECTOR CONTROL[/b] "+"[/color]",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.15, "y": 0.925},
                    markup=True,
                    font_size= 22
                    )
        lbl1 = Label(text="[color=FFFFFF]"+"[b]JOINT CONTROL[/b] "+"[/color]",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.5, "y": 0.925},
                    markup=True,
                    font_size= 22
                    )
        btn_fd = Button(text ='FREE DRIVE OFF',
                    size_hint =(.2, 0.1),
                    bold=True,
                    pos_hint= {"x": 0.775, "y": 0.55},
                    color=(250 / 255, 23 / 255, 23 / 255, 0.92),
                    font_size= 20,
                    background_color=(1, 0.8, 0.9, 1),
                    )
        #print(dir(delta_pos))
        #btn2 = Button(text ='Hello world2',
        #            size_hint =(.3, .2),
        #            pos_hint= {"x": 0.25, "y": 0.8})
        #btn2 = Button(text ='Hello world2',
        #            size_hint =(.3, .2),
        #            pos =(300, 300))

        # adding widget i.e button
        btn_zp.bind(on_press=lambda *args: clk('btn000', *args))
        Fl.add_widget(btn_zp)
        Fl.add_widget(btn_yp)
        Fl.add_widget(btn_xn)
        Fl.add_widget(btn_xp)
        Fl.add_widget(btn_yn)
        Fl.add_widget(btn_zn)
        Fl.add_widget(btn_rzp)
        Fl.add_widget(btn_ryp)
        Fl.add_widget(btn_rxn)
        Fl.add_widget(btn_rxp)
        Fl.add_widget(btn_ryn)
        Fl.add_widget(btn_rzn)

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

        Fl.add_widget(delta_pos)
        Fl.add_widget(delta_orn)
        Fl.add_widget(delta_q)
        Fl.add_widget(lbl)
        Fl.add_widget(lbl0)
        Fl.add_widget(lbl1)
        Fl.add_widget(btn_fd)

        btn_fd.bind(on_press=lambda *args: drivemode('btnfd', *args))
        timer = Clock.schedule_interval(timedAction, 0.02)

        #Fl.add_widget(btn2)

        # return the layout
        return Fl

# run the App
if __name__ == "__main__":

    app=EasyGUIApp()
    app.run()
