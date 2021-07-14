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

SPD_SERVER = 'localhost'
SPD_PORT = 9250

class PersonHereFluentProxy(FluentProxy, rosnode=False):   # <--- fluent class

    def __init__(self, fluentnane, rosnode=False):
        FluentProxy.__init__(self, fluentnane, rosnode)
        self.takephoto_pub = rospy.Publisher(TOPIC_takephoto, String, queue_size=1)
        rospy.sleep(0.1)
        self.server = SPD_SERVER
        self.port = SPD_PORT

    def __del__(self):
        FluentProxy.__del__(self)

    def setServer(server, port):
        rospy.param_get()
        self.server = server
        self.port = port

    def sensingStep(self):

        
        sockOK = False
        # connecting to stagepersondetection

        try:
            sock = socket.socket()          
            sock.connect((self.server,self.port))
            sock.close()
            sockOK = True            
        except Exception as e:
            print(e)
            print("Cannot connect to server %s:%d" %(self.server,self.port))
            self.server = '192.168.0.205'  # try this other server
        

        if not sockOK:
            try:
                sock = socket.socket()
                sock.connect((self.server,self.port))
                sock.close()
                sockOK = True            
            except Exception as e:
                print(e)
                print("Cannot connect to server %s:%d" %(self.server,self.port))
                return


        s = String()
        s.data = 'send %s %s' %(self.server,self.port)

        # notify takephoto to send an image to (self.server,self.port)
        self.takephoto_pub.publish(s)

        # wait for server to receive and analyze the image...
        rospy.sleep(0.2)

        if (self.server != 'localhost') and (self.server != '127.0.0.1'):
            rospy.sleep(1)

        # receive result from (self.server,self.port)
        try:
            sock = socket.socket()
            sock.connect((self.server,self.port))
            sock.sendall("GETRESULT\n\r")
            data = sock.recv(256)
            data = data.strip().decode('UTF-8')
            print(data)    
            sock.close()
        except:
            data = None

        if data is None:
            value = -1
        else:        
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

    t = PersonHereFluentProxy(FLUENT_NAME,rosnode=True)  # <--- fluent class
    t.execute(params)  # blocking, CTRL-C to interrupt


