#!/usr/bin/env python3

import rospy
import tkinter as tk

from std_msgs.msg import UInt8
from std_srvs.srv import Empty

rospy.init_node("deliverybot_hw_gui")

open_bot_door_srv = rospy.ServiceProxy('/dbot/open_bot_door', Empty)
close_bot_door_srv = rospy.ServiceProxy('/dbot/close_bot_door', Empty)
set_pusher_int_srv = rospy.ServiceProxy('/dbot/set_pusher_in', Empty)
set_pusher_out_srv = rospy.ServiceProxy('/dbot/set_pusher_out', Empty)
deliver_package_srv = rospy.ServiceProxy('/dbot/deliver_package', Empty)


def open_bot_door():
    open_bot_door_srv.call()

def close_bot_door():
    close_bot_door_srv.call()

def pusher_in():
    set_pusher_int_srv.call()

def pusher_out():
    set_pusher_out_srv.call()

def deliver_package():
    deliver_package_srv.call()


class Application(tk.Frame):

    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pack()
        self.create_widgets()


    def create_widgets(self):
        self.create_bot_widgets()


    def create_bot_widgets(self):
        dbot_control_frame = tk.LabelFrame(self, text="DeliveryBot Controls")
        dbot_control_frame.grid_rowconfigure(0, weight=1)
        dbot_control_frame.grid_columnconfigure(0, weight=1)

        dbot_control_frame.pack(fill='x', pady=5)
        self.btn_open_bot_door = tk.Button(dbot_control_frame, text="Open Door", command=open_bot_door)
        self.btn_open_bot_door.grid(row=0,column=0, sticky="ew")

        self.btn_close_bot_door = tk.Button(dbot_control_frame, text="Close Door", command=close_bot_door)
        self.btn_close_bot_door.grid(row=1,column=0, sticky="ew")

        self.btn_pusher_out = tk.Button(dbot_control_frame, text="Pusher Out", command=pusher_out)
        self.btn_pusher_out.grid(row=0,column=1, sticky="ew")

        self.btn_pusher_in = tk.Button(dbot_control_frame, text="Pusher In", command=pusher_in)
        self.btn_pusher_in.grid(row=1,column=1,sticky="ew")

        self.btn_deliver = tk.Button(dbot_control_frame, text="Deliver Package", command=deliver_package)
        self.btn_deliver.grid(row=0, column=2, rowspan=2, sticky="news")


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Deliverybot HW GUI")
    root.resizable(False, False)
    app = Application(master=root)
    app.mainloop()