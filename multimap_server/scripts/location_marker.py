#!/usr/bin/env python

import Tkinter as tk
import rospy
import os
import tf2_ros
import tf_conversions
import yaml
import sys

class LocationMarker(tk.Frame):

    def __init__(self, master=None):
        tk.Frame.__init__(self, master=master)
        self.tfBuffer = tf2_ros.Buffer()
        self.file_path = None
        self.map = {"locations": {}}
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.pack(expand=True, fill="both")
        self.create_widgets()

    def save_yaml(self):
        '''
        Save contents of map to yaml file
        '''
        with open(os.path.abspath(self.file_path), 'w') as f:
            yaml.dump(self.map, f)

    def mark_location(self, location_name):
        '''
        Assign location_name with x, y, theta to map using lookup_transform between map and dbot/base_link.
        Save value to yaml file.        
        '''
        trans = self.tfBuffer.lookup_transform(
            'map',
            'dbot/base_link',
            rospy.Time()
        )
        self.map["locations"][location_name] = {
            "x": trans.transform.translation.x,
            "y": trans.transform.translation.y,
            "a": tf_conversions.transformations.euler_from_quaternion(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )[2]
        }
        rospy.loginfo("Location %s marked at x:%f y:%f a:%f" % (
            location_name,
            self.map["locations"][location_name]["x"],
            self.map["locations"][location_name]["y"],
            self.map["locations"][location_name]["a"]
            )
            )

        self.save_yaml()

    def load_map(self, file_path):
        '''
        Set map(dictionary) using values from map file
        '''
        self.file_path = file_path
        with open(os.path.abspath(file_path), 'r') as f:
            self.map = yaml.safe_load(f)

    def create_widgets(self):
        '''
        Code for creating widgets
        '''
        self.var_location_name = tk.StringVar()
        frame = tk.Frame(self)
        frame.pack(side="top", expand=True, fill="both")
        def mark_location_helper():
            if (self.var_location_name.get() != ""):
                self.mark_location(self.var_location_name.get())
                location_entry.delete(0, tk.END)

        location_label = tk.Label(frame, text="Name")
        location_label.grid(column=0, row=0, sticky="ew")

        location_entry = tk.Entry(frame, bd=2, textvariable=self.var_location_name)
        location_entry.grid(column=1, row=0, sticky="ew")

        submit_btn = tk.Button(frame, text="Mark", command=mark_location_helper)
        submit_btn.grid(column=2, row=0, sticky="ew")

if __name__=="__main__":
    rospy.init_node("location_marker", anonymous=True)
    root = tk.Tk()
    root.resizable(False, False)
    app = LocationMarker(master=root)
    app.load_map(sys.argv[1])
    tk.mainloop()