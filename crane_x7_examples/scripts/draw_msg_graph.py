#! /usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time
from sensor_msgs.msg import PointCloud

class GapDetector():
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection="3d")
        plt.ion() # enable graph output
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")
        self.ax.set_xlim3d(-1.0, 1.0)
        self.ax.set_ylim3d(-1.0, 1.0)
        self.ax.set_zlim3d(-1.0, 1.0)
        self.x = []
        self.y = []
        self.z = []
        self.line_3D, = self.ax.plot(self.x, self.y, self.z, zdir="z",
                                     label="pointcloud xyz position", 
                                     marker=".", linestyle="")
        self.bd_points_sub = rospy.Subscriber("/edge_img_bounding_box_points", PointCloud, self._bd_points_callback, queue_size=1)

    def plot_point_xyz_graph(self):
        self.line_3D.set_data_3d(self.x, self.y, self.z)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _bd_points_callback(self, pointcloud):
        print('pointcloud is here')
        self.x = self.y = self.z = []
        for p in pointcloud.points:
            self.x.append(p.x)
            self.y.append(p.y)
            self.z.append(p.z)

def main():
    gd = GapDetector()

    while not rospy.is_shutdown():
        print("Here is xyz")
        print("x:", gd.x)
        print("y:", gd.y)
        print("z:", gd.z)
        #gd.plot_point_xyz_graph()
        #plt.pause(1.0e-10)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
