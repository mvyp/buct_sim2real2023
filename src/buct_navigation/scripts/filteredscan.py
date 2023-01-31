#!/usr/bin/env python3

from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DoFilter:
    def __init__(self):

        self.sub = rospy.Subscriber("/rplidar/scan", LaserScan, self.callback)
        self.pub = rospy.Publisher("filteredscan", LaserScan, queue_size=10)

    def callback(self, data):
        newdata = data
        newdata.ranges = list(data.ranges)
        
  #      newdata.intensities = list(data.intensities)

        for x in range(350,410):
            newdata.ranges[x]=0
     #       newdata.intensities[x]=0
        self.pub.publish(newdata)


if __name__ == '__main__':
    rospy.init_node('LidarFilter', anonymous=False)
    lidar = DoFilter()
    rospy.spin()
