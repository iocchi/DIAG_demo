# getimage action

import time, math, sys, random

import rospy
import tf

from actionproxy import ActionProxy

from std_msgs.msg import String

ACTION_NAME = 'getimage'

ROS_NODE_NAME = 'takephoto'
TOPIC_takephoto = '/takephoto'
PARAM_takephoto_image_folder = '%s/imagefolder' %ROS_NODE_NAME


'''
Get an image and save it to data/image folder

params: imagefolder

'''

class GetImageActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)
        self.takephoto_pub = rospy.Publisher(TOPIC_takephoto, String, queue_size=1)
        print("Publisher to topic: %s" %TOPIC_takephoto)
        rospy.sleep(0.1)

    def __del__(self):
        ActionProxy.__del__(self)


    def action_thread(self, params):

        if (params!=None and params!=''):
            rospy.set_param(PARAM_takephoto_image_folder,params)

        s = String()
        s.data = 'get'
        self.takephoto_pub.publish(s)

        print('  -- take photo --')


if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = GetImageActionProxy(ACTION_NAME)
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt

