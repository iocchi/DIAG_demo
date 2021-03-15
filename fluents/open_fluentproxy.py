# lightcolor fluent implementation

import sys

import rospy
from sensor_msgs.msg import LaserScan

from fluentproxy import FluentProxy

FLUENT_NAME = 'open'

TOPIC_scan = 'scan'

'''
Check if door in front of the robot is open,
it assumes the robot to face the door,
actually it just checks if there is free space ahead.

'''

class OpenFluentProxy(FluentProxy):

    def __init__(self, fluentnane):
        FluentProxy.__init__(self, fluentnane)
        self.laser_sub = rospy.Subscriber(TOPIC_scan, LaserScan, self.laserscan_cb)
        self.laser_center_dist = 10

    def __del__(self):
        FluentProxy.__del__(self)
        self.laser_sub.unregister()

    def laserscan_cb(self, data):
        nc = len(data.ranges)/2
        self.laser_center_dist = min(data.ranges[nc-2:nc+2])
        #print("angle min %.3f max %.3f inc %.6f" %(data.angle_min, data.angle_max, data.angle_increment))
        #print("center %.3f" %(self.laser_center_dist))


    # no input params
    def fluent_thread(self, params):

        while self.do_run:

            #print(self.laser_center_dist)

            if self.laser_center_dist > 1.5:
                value = 1
            else:
                value = 0

            self.setValue('',value)     # 1: true,  0: false,  -1: unknown

            rospy.sleep(0.5)





if __name__ == "__main__":

    params = ''
    if (len(sys.argv)>1):
        params = sys.argv[1]

    t = OpenFluentProxy(FLUENT_NAME)
    t.execute(params)  # blocking, CTRL-C to interrupt


