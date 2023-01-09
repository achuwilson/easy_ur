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
Window.minimum_height = 400
Window.minimum_width = 600
Window.size = (900, 500)
Window.clearcolor = (0.31, 0.31, 0.31, 1)
from kivy.clock import Clock


# 0 being off 1 being on as in true / false
# you can use 0 or 1 && True or False
#Config.set('graphics', 'resizable', True)

btn=None
lbl=None
def clk(val, inst):
    global btn
    print("clicked ",val)# inst, dir(inst.get_parent_window()))
    #btn.disabled=True
    #inst.background_color=(0.9, 0.8, 0, 0.9)
    btn.background_color=(48/255, 213/255, 200/255, 1)
    #inst.get_parent_window()#.btn.background_color=(0.9, 0.8, 0, 0.9)
    #pass
r_cnt=0
def timedAction(dt):
    global r_cnt, lbl
    r_cnt =  r_cnt+1
    lbl.text= "[color=FFFF00]"+"X : "+"[/color]"+ str(r_cnt/100.0)
    #print("Repeat ", dt)
# creating the App class
class EasyGUIApp(App):

    def build(self):
        global btn, lbl

        # creating Floatlayout
        Fl = FloatLayout()

        # creating button
        # a button 30 % of the width and 20 %
        # of the height of the layout and
        # positioned at (300, 200), you can do:
        btn = Button(text ='X+',
                    size_hint =(.1, 0.05),

                    bold=True,
                    pos_hint= {"x": 0.25, "y": 0.6},
                    color=(250 / 255, 23 / 255, 23 / 255, 0.92),
                    font_size= 20,
                    background_color=(1, 0.8, 0.9, 1),
                    )
        delta_pos = TextInput(multiline=False,
                        size_hint =(.1, 0.06),
                        pos_hint= {"x": 0.5, "y": 0.7},
                        hint_text='DELTA POS',
                        input_filter = 'float',
                        write_tab=False,
                        )
        lbl = Label(text="[color=FFFF00]"+"X : "+"[/color]"+" 1233.2",
                    size_hint =(.1, 0.06),
                    pos_hint= {"x": 0.3, "y": 0.9},
                    markup=True
                    )
        #print(dir(delta_pos))
        #btn2 = Button(text ='Hello world2',
        #            size_hint =(.3, .2),
        #            pos_hint= {"x": 0.25, "y": 0.8})
        #btn2 = Button(text ='Hello world2',
        #            size_hint =(.3, .2),
        #            pos =(300, 300))

        # adding widget i.e button
        btn.bind(on_press=lambda *args: clk('btn000', *args))
        Fl.add_widget(btn)
        Fl.add_widget(delta_pos)
        Fl.add_widget(lbl)
        timer = Clock.schedule_interval(timedAction, 0.02)

        #Fl.add_widget(btn2)

        # return the layout
        return Fl

# run the App
if __name__ == "__main__":

    app=EasyGUIApp()
    app.run()
