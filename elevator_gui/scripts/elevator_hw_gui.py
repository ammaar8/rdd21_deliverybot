#!/usr/bin/env python

import rospy
import Tkinter as tk

from std_srvs.srv import Empty
from elevator_controls.srv import ElevatorFloorGoal

rospy.init_node("elevator_hw_gui")

open_elevator_doors_srv = rospy.ServiceProxy('/elevator/open_elevator_doors', Empty)
close_elevator_doors_srv = rospy.ServiceProxy('/elevator/close_elevator_doors', Empty)
go_to_floor_srv = rospy.ServiceProxy('/elevator/elevator_goto_floor', ElevatorFloorGoal)


def go_to_floor(floor):
    floor = int(floor)
    go_to_floor_srv.call(floor)


def open_elevator_doors():
    open_elevator_doors_srv.call()


def close_elevator_doors():
    close_elevator_doors_srv.call()


class Application(tk.Frame):

    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pack()
        self.create_widgets()


    def create_widgets(self):
        self.create_elevator_widgets()


    def create_elevator_widgets(self):
        elevator_control_frame = tk.LabelFrame(self, text="Elevator Controls")
        elevator_control_frame.grid_rowconfigure(0, weight=1)
        elevator_control_frame.grid_columnconfigure(0, weight=1)
        elevator_control_frame.pack(fill="x")
        selected_floor = tk.IntVar()

        self.floor_label = tk.Label(elevator_control_frame, text="Floor")
        self.floor_label.grid(row=0, column=0, sticky="ew")

        self.dropdown_floors = tk.OptionMenu(elevator_control_frame, selected_floor, 0, 1, 2, command=go_to_floor)
        self.dropdown_floors.grid(row=1, column=0, sticky="ew")

        self.btn_open_elevator_doors = tk.Button(elevator_control_frame, text="Open Doors", command = open_elevator_doors)
        self.btn_open_elevator_doors.grid(row=0, column=1, sticky="ew")
        
        self.btn_close_elevator_doors = tk.Button(elevator_control_frame, text="Close Doors", command = close_elevator_doors)
        self.btn_close_elevator_doors.grid(row=1, column=1, sticky="ew")  


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Elevator HW GUI")
    root.resizable(False, False)
    app = Application(master=root)
    app.mainloop()