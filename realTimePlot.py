#!/usr/bin/env python
from mimetypes import init
from tokenize import Double
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import rospy
from std_msgs.msg import Float32MultiArray
import threading

class plotDdata:
    def __init__(self):

        self.dx = np.array([])
        self.dy = np.array([])
        self.dz = np.array([])
        self.dr = np.array([])
        self.dp = np.array([])
        self.dyaw = np.array([])
        self.dAngle = np.array([])
        self.runTimeList = np.array([])
        self.afterTime = []
        self.lastAfterTime = -1.0
        mpl.rcParams['toolbar'] = 'None'
        plt.ion()
        mpl.use('TkAgg')
        self.runTime = 0.0
        self.initTime = 0.0
        self.init = False
        self.count = 0
        self.lock = threading.Lock()

        self.fig=plt.figure(figsize=(15,15))
        # plt.title("integrate2map\n", fontsize=12)
        # plt.margins(x=0.001)
        self.fig1=self.fig.add_subplot(331)
        self.fig1.set_ylabel("dx\nDistance (m$^3$)", fontsize=12)
        self.l1, = self.fig1.plot(self.dx, self.runTimeList, color='r', linestyle=" ", marker="o",markersize=0.25,label='dx')
        self.fig2=self.fig.add_subplot(332)
        self.fig2.set_ylabel("dy\nDistance (m)", fontsize=12)
        self.l2, = self.fig2.plot(self.dy, self.runTimeList, color='r',linestyle=" ", marker="o",markersize=0.25, label='dy')
        self.fig3=self.fig.add_subplot(333)
        self.fig3.set_ylabel("dz\nDistance (s)", fontsize=12)
        self.l3, = self.fig3.plot(self.dz, self.runTimeList, color='r',linestyle=" ", marker="o",markersize=0.25, label='dz')
        self.fig4=self.fig.add_subplot(334)
        self.fig4.set_ylabel("dr\nDistance (s)", fontsize=12)
        self.l4, = self.fig4.plot(self.dr, self.runTimeList, color='r', linestyle=" ", marker="o",markersize=0.25,label='droll')
        self.fig5=self.fig.add_subplot(335)
        self.fig5.set_ylabel("dp\nDistance (s)", fontsize=12)
        self.l5, = self.fig5.plot(self.dp, self.runTimeList, color='r', linestyle=" ", marker="o",markersize=0.25,label='dpich')
        self.fig6=self.fig.add_subplot(336)
        self.fig6.set_ylabel("dyaw\nDistance (s)", fontsize=12)
        self.l6, = self.fig6.plot(self.dyaw, self.runTimeList, color='r', linestyle=" ", marker="o",markersize=0.25,label='dyaw')
        self.fig7=self.fig.add_subplot(337)
        self.fig7.set_ylabel("dangle\nDistance (s)", fontsize=12)
        self.l7, = self.fig7.plot(self.dAngle, self.runTimeList, color='r', linestyle=" ", marker="o",markersize=0.25, label='dangle')
        # self.fig7.set_xlabel("Time Duration (s)", fontsize=12) #only set once

        rospy.init_node('realTimePlot')
        rospy.Subscriber("/data_show", Float32MultiArray, self.dataShowCallback)
        

    def add_data2fig(self, fig,l, x, y,kind="xyz"):
        l.set_xdata(x)
        l.set_ydata(y)
        if(self.runTime - self.initTime>1):
            fig.set_xlim(self.runTime-0.5, self.runTime+0.1)
        else:
            fig.set_xlim(self.initTime,self.initTime+1)

        for x in self.afterTime:
            fig.axvline(x,-10,10)

        if kind == "xyz":
            fig.set_ylim(-10, 10)
        elif kind == "rpy":
            fig.set_ylim(-0.5,0.5)
        elif kind == "angle":
            fig.set_ylim(0,5)

    def dataShowCallback(self, msg):
        self.lock.acquire()
        self.runTimeList = np.append(self.runTimeList,msg.data[0])
        self.runTime = msg.data[0]
        if(not self.init):
            self.initTime = self.runTime
            self.init = True
        self.dx = np.append(self.dx,msg.data[1])
        # print(self.dx.size,"   ",self.runTimeList.size)
        self.dy = np.append(self.dy,msg.data[2])
        self.dz = np.append(self.dz,msg.data[3])
        self.dr = np.append(self.dr,msg.data[4])
        self.dp = np.append(self.dp,msg.data[5])
        self.dyaw = np.append(self.dyaw,msg.data[6])
        self.dAngle = np.append(self.dAngle,msg.data[7])
        if(self.lastAfterTime!=msg.data[8]):
            self.afterTime.append(msg.data[8])
            self.lastAfterTime = msg.data[8]
        self.lock.release()

    def draw(self):
        a = rospy.Rate(500)
        while not rospy.is_shutdown():

            self.lock.acquire()
            self.add_data2fig(self.fig1, self.l1, self.runTimeList, self.dx,"xyz")
            self.add_data2fig(self.fig2, self.l2, self.runTimeList, self.dy,"xyz")
            self.add_data2fig(self.fig3, self.l3, self.runTimeList, self.dz,"xyz")
            self.add_data2fig(self.fig4, self.l4, self.runTimeList, self.dr,"rpy")
            self.add_data2fig(self.fig5, self.l5, self.runTimeList, self.dp,"rpy")
            self.add_data2fig(self.fig6, self.l6, self.runTimeList, self.dyaw,"rpy")
            self.add_data2fig(self.fig7, self.l7, self.runTimeList, self.dAngle,"angle")
            self.afterTime = []
            self.lock.release()
            if self.count == 40 or self.count==0:
                self.fig.canvas.draw()
                self.count = 0
            self.count +=1
            a.sleep()
        rospy.spin()
        


if __name__ == '__main__':
  a = plotDdata()
  a.draw()
  print("1")
