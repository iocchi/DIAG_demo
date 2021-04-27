# template for fluent implementation

import sys, os, socket

import rospy
from std_msgs.msg import String

#sys.path.append("....")                  # <--- rospy PLEXI folder
from fluentproxy import FluentProxy

FLUENT_NAME = 'personhere'                  # <--- fluent name

# takephoto topics/params
ROS_NODE_NAME = 'takephoto'
TOPIC_takephoto = '/takephoto'
PARAM_takephoto_image_folder = '%s/imagefolder' %ROS_NODE_NAME


class PersonHereFluentProxy(FluentProxy):   # <--- fluent class

    def __init__(self, fluentnane, rosnode=False):
        FluentProxy.__init__(self, fluentnane, rosnode)
        self.takephoto_pub = rospy.Publisher(TOPIC_takephoto, String, queue_size=1)
        rospy.sleep(0.1)

    def __del__(self):
        FluentProxy.__del__(self)

    def sensingStep(self):

        s = String()
        s.data = 'send'
        self.takephoto_pub.publish(s)

        rospy.sleep(0.2)

        sock = socket.socket()
        sock.connect(('localhost', 9250)) # stagepersondetection
        sock.sendall("GETRESULT\n\r")
        data = sock.recv(256)
        data = data.strip().decode('UTF-8')
        print(data)    
        sock.close()
        
        v = data.split(' ')
        if (float(v[1])<0.6):
            value = -1
        elif v[0]=='none':
            value = 0
        else:
            value = 1

        self.setValue(value)     # 1: true,  0: false,  -1: unknown
        


    def fluent_thread(self, params):
        v = params.split('_')
        while self.do_run:
            self.sensingStep()
            rospy.sleep(5)




if __name__ == "__main__":

    params = ''
    if (len(sys.argv)>1):
        params = sys.argv[1]

    t = PersonHereFluentProxy(FLUENT_NAME)  # <--- fluent class
    t.execute(params)  # blocking, CTRL-C to interrupt


