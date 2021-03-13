# template action

import sys

import rospy
from std_msgs.msg import String

#sys.path.append("....")                # <--- rospy PLEXI folder
from actionproxy import ActionProxy

ACTION_NAME = 'say'

TOPIC_STAGESAY = '/stage_say'

class SayActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)
        self.stagesay_pub = rospy.Publisher(TOPIC_STAGESAY, String, queue_size=1, latch=True)

    def __del__(self):
        ActionProxy.__del__(self)
                                        # <--- action del

    #def interrupt(self):
    #    ActionProxy.end(self)
                                        # <--- action interrupt (default behavior: end)

    #def resume(self):
    #    ActionProxy.resume(self)
                                        # <--- action resume (default behavior: start with same params)

    def action_thread(self, params):
        s = String()
        s.data = params
        self.stagesay_pub.publish(s)
        t = 2.0 + len(params)/10.0
        dt = 0.25
        while self.do_run and t>0:
            rospy.sleep(dt)
            t -= dt
        s.data = ""
        self.stagesay_pub.publish(s)

if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = SayActionProxy(ACTION_NAME)
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt

