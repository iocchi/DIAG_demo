# template action

import sys, os

import rospy
from std_msgs.msg import String

from actionproxy import ActionProxy

ACTION_NAME = 'say'

TOPIC_STAGESAY = '/stage_say'

class SayActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)
        self.stagesay_pub = rospy.Publisher(TOPIC_STAGESAY, String, queue_size=1, latch=True)

    def __del__(self):
        ActionProxy.__del__(self)

    #def interrupt(self):
    #    ActionProxy.end(self)
                                        # <--- action interrupt (default behavior: end)

    #def resume(self):
    #    ActionProxy.resume(self)
                                        # <--- action resume (default behavior: start with same params)

    def action_thread(self, params):

        v = params.split('_')
        if v[0]=='going':
            strtosay = "Hello world! I'm going to %s." %v[1]
        elif v[0]=='color':
            strtosay = "Hi! The led on %s, is %s." %(v[1],v[2])
        else:
            strtosay = v[0]

        s = String()
        s.data = strtosay
        self.stagesay_pub.publish(s)
        cmd = 'echo "TTS[en] %s" | netcat -w 1 localhost 9001' %strtosay
        os.system(cmd)

        t = 2.0 + len(params)/10.0
        dt = 0.25
        while self.do_run and t>0:
            rospy.sleep(dt)
            t -= dt
        s.data = ""
        self.stagesay_pub.publish(s)
        rospy.sleep(dt)

if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = SayActionProxy(ACTION_NAME)
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt

