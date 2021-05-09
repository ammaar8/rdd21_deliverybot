#!/usr/bin/env python
import actionlib
import deliverybot_navigation.msg
import Tkinter as tk
import rospy

class Application(tk.Frame):
    
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.client = actionlib.SimpleActionClient(
            'delivery_server',
            deliverybot_navigation.msg.DeliverAction
            )
        self.client.wait_for_server()
        self.master = master
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        widget_frame = tk.LabelFrame(self, text="Delivery GUI")
        widget_frame.grid_columnconfigure(0, weight=1)
        widget_frame.grid_rowconfigure(0, weight=1)
        widget_frame.pack(fill='x', pady=5)

        building_name_var = tk.StringVar()
        floor_var = tk.StringVar()
        room_var = tk.StringVar()

        self.status_label = tk.Label(widget_frame, text="Idle")
        self.status_label.grid(row=0, column=0, columnspan=2, sticky="ew")
        building_name_label = tk.Label(widget_frame, text="Building Name")
        building_name_label.grid(row=1, column=0, sticky="ew")
        building_name_entry = tk.Entry(widget_frame, textvariable=building_name_var)
        building_name_entry.grid(row=1, column=1, sticky="ew")
        floor_label = tk.Label(widget_frame, text="Floor")
        floor_label.grid(row=2, column=0, sticky="ew")
        floor_entry = tk.Entry(widget_frame, textvariable=floor_var)
        floor_entry.grid(row=2, column=1, sticky="ew")
        room_label = tk.Label(widget_frame, text="Room")
        room_label.grid(row=3, column=0, sticky="ew")
        room_entry = tk.Entry(widget_frame, textvariable=room_var)
        room_entry.grid(row=3, column=1, sticky="ew")
        self.send_goal_button = tk.Button(
            widget_frame,
            text="Deliver",
            command=lambda: self.send_delivery_goal(
                building_name_var.get(),
                int(floor_var.get()),
                int(room_var.get())
                )
            )
        self.send_goal_button.grid(row=4, column=0, sticky="ew")

        def reset():
            building_name_entry.delete(0, tk.END)
            floor_entry.delete(0, tk.END)
            room_entry.delete(0, tk.END)

        reset_btn = tk.Button(widget_frame, text="Reset", command = reset)
        reset_btn.grid(row=4, column=1, sticky="ew")


    def send_delivery_goal(self, building_name, floor, room):
        self.client.wait_for_server()
        goal = deliverybot_navigation.msg.DeliverGoal(
            building_name,
            floor,
            room)
        self.client.send_goal(goal)
        self.status_label.config(text="Running Delivery")
        self.send_goal_button["state"] = "disabled"
        self.master.update()
        self.client.wait_for_result()
        self.send_goal_button["state"] = "normal"
        self.status_label.config(text="Idle")

if __name__ == "__main__":
    rospy.init_node("delivery_client_gui")
    root = tk.Tk()
    root.title("Delivery GUI")
    root.resizable(False, False)
    app =  Application(master=root)
    app.mainloop()