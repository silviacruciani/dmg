#!/usr/bin/python

"""
    This file contains a GUI for calling the shape analysis server

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
import Tkinter
from Tkinter import *
import math
from shape_analysis.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion

class InHandPathClient(Tkinter.Tk):
    def __init__(self):
        Tkinter.Tk.__init__(self)

        self.after(500, self.update)

        self.server_name ="/in_hand_path_server/in_hand_path"

        self.frame1 = Frame(self)
        self.frame1.pack(fill="both", expand="yes")

        self.labelframe_p1 = LabelFrame(self.frame1, text="initial contact 1")
        self.labelframe_p1.pack(side = LEFT, fill="both", expand="yes")

        self.px1 = Entry(self.labelframe_p1, width = 5)
        self.py1 = Entry(self.labelframe_p1, width = 5)
        self.pz1 = Entry(self.labelframe_p1, width = 5)
        self.qx1 = Entry(self.labelframe_p1, width = 5)
        self.qy1 = Entry(self.labelframe_p1, width = 5)
        self.qz1 = Entry(self.labelframe_p1, width = 5)
        self.qw1 = Entry(self.labelframe_p1, width = 5)

        self.labelframe_p2 = LabelFrame(self.frame1, text="initial contact 2")
        self.labelframe_p2.pack(side = LEFT, fill="both", expand="yes")

        self.px2 = Entry(self.labelframe_p2, width = 5)
        self.py2 = Entry(self.labelframe_p2, width = 5)
        self.pz2 = Entry(self.labelframe_p2, width = 5)
        self.qx2 = Entry(self.labelframe_p2, width = 5)
        self.qy2 = Entry(self.labelframe_p2, width = 5)
        self.qz2 = Entry(self.labelframe_p2, width = 5)
        self.qw2 = Entry(self.labelframe_p2, width = 5)

        self.frame2 = Frame(self)
        self.frame2.pack(fill="both", expand="yes")

        self.labelframe_d1 = LabelFrame(self.frame2, text="desired contact 1")
        self.labelframe_d1.pack(side = LEFT, fill="both", expand="yes")

        self.px1d = Entry(self.labelframe_d1, width = 5)
        self.py1d = Entry(self.labelframe_d1, width = 5)
        self.pz1d = Entry(self.labelframe_d1, width = 5)
        self.qx1d = Entry(self.labelframe_d1, width = 5)
        self.qy1d = Entry(self.labelframe_d1, width = 5)
        self.qz1d = Entry(self.labelframe_d1, width = 5)
        self.qw1d = Entry(self.labelframe_d1, width = 5)

        self.labelframe_d2 = LabelFrame(self.frame2, text="desired contact 2")
        self.labelframe_d2.pack(side = LEFT, fill="both", expand="yes")

        self.px2d = Entry(self.labelframe_d2, width = 5)
        self.py2d = Entry(self.labelframe_d2, width = 5)
        self.pz2d = Entry(self.labelframe_d2, width = 5)
        self.qx2d = Entry(self.labelframe_d2, width = 5)
        self.qy2d = Entry(self.labelframe_d2, width = 5)
        self.qz2d = Entry(self.labelframe_d2, width = 5)
        self.qw2d = Entry(self.labelframe_d2, width = 5)


        self.labelframe_call = LabelFrame(self, text="service call")
        self.labelframe_call.pack(fill="both", expand="yes")
       
        self.b_call = Tkinter.Button(self.labelframe_call, width = 5, text ="CALL", command = self.call)

        #packing all the buttons and entries
        self.px1.pack()
        self.py1.pack()
        self.pz1.pack()
        self.qx1.pack()
        self.qy1.pack()
        self.qz1.pack()
        self.qw1.pack()

        self.px2.pack()
        self.py2.pack()
        self.pz2.pack()
        self.qx2.pack()
        self.qy2.pack()
        self.qz2.pack()
        self.qw2.pack()

        self.px1d.pack()
        self.py1d.pack()
        self.pz1d.pack()
        self.qx1d.pack()
        self.qy1d.pack()
        self.qz1d.pack()
        self.qw1d.pack()

        self.px2d.pack()
        self.py2d.pack()
        self.pz2d.pack()
        self.qx2d.pack()
        self.qy2d.pack()
        self.qz2d.pack()
        self.qw2d.pack()

        self.b_call.pack()

        print ("initialized manual inputs interface.")

    def call(self):
        t1 = Point()
        o1 = Quaternion()
        t2 = Point()
        o2 = Quaternion()
        t1d = Point()
        o1d = Quaternion()
        t2d = Point()
        o2d = Quaternion()

        t1.x = float(self.px1.get())
        t1.y = float(self.py1.get())
        t1.z = float(self.pz1.get())
        o1.x = float(self.qx1.get())
        o1.y = float(self.qy1.get())
        o1.z = float(self.qz1.get())
        o1.w = float(self.qw1.get())

        t2.x = float(self.px2.get())
        t2.y = float(self.py2.get())
        t2.z = float(self.pz2.get())
        o2.x = float(self.qx2.get())
        o2.y = float(self.qy2.get())
        o2.z = float(self.qz2.get())
        o2.w = float(self.qw2.get())

        t1d.x = float(self.px1d.get())
        t1d.y = float(self.py1d.get())
        t1d.z = float(self.pz1d.get())
        o1d.x = float(self.qx1d.get())
        o1d.y = float(self.qy1d.get())
        o1d.z = float(self.qz1d.get())
        o1d.w = float(self.qw1d.get())

        t2d.x = float(self.px2d.get())
        t2d.y = float(self.py2d.get())
        t2d.z = float(self.pz2d.get())
        o2d.x = float(self.qx2d.get())
        o2d.y = float(self.qy2d.get())
        o2d.z = float(self.qz2d.get())
        o2d.w = float(self.qw2d.get())

        pose1 = Pose()
        pose2 = Pose()
        pose1d = Pose()
        pose2d = Pose()

        pose1.position = t1
        pose1.orientation = o1
        pose2.position = t2
        pose2.orientation = o2
        pose1d.position = t1d
        pose1d.orientation = o1d
        pose2d.position = t2d
        pose2d.orientation = o2d

        print("Contacting server...")

        rospy.wait_for_service(self.server_name, timeout = 5.0)
        try:
            manipulation_path = rospy.ServiceProxy(self.server_name, InHandPath)
            req = InHandPathRequest()
            req.initial_grasp.append(pose1)
            req.initial_grasp.append(pose2)
            req.desired_grasp.append(pose1d)
            req.desired_grasp.append(pose2d)
            print ("Sending sequest: ", req)
            resp = manipulation_path(req)
            print resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('in_hand_path_client')
    inp = InHandPathClient()
    inp.mainloop()