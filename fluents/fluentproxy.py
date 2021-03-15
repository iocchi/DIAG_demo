#
# FluentProxy base class
#

from threading import Thread

import rospy
from std_msgs.msg import String

'''
In this context, the terms conditions, fluents, and predicates are used as synonyms. In PNP, the term condition is most often used.

pnp_ros reads conditions from ROS params named `pnp/conditionsBuffer/<condition_name>` with values

     1: True
     0: False
    -1: unknown


Condition executors should set these values.

Test with CLI

    rosparam get pnp/conditionsBuffer/<condition>

    rosparam set pnp/conditionsBuffer/<condition> 1

Quit all proxies with

    rostopic pub pnp/action_str std_msgs/String "data: '%quit_server'" --once

'''


# topic for subscribers
TOPIC_PNPACTIONPROXY_STR = "pnp/action_str"

# param for conditions
PARAM_PNPCONDITIONBUFFER = "pnp/conditionsBuffer/"


class FluentProxy:

    def __init__(self, fluentname):

        self.fluentname = fluentname
        self.timestamp = None   # time of current value
        self.value = None       # current value
        self.cthread = None
        self.rosparam = PARAM_PNPCONDITIONBUFFER + fluentname

        # init ROS node
        nodename = fluentname+"_fluentproxy"
        rospy.init_node(nodename,  disable_signals=True)

        # subscribers
        self.actionproxy_sub = rospy.Subscriber(TOPIC_PNPACTIONPROXY_STR, String, self.actionproxy_cb)  # only to check quit signal


    def __del__(self):
        # just in case
        self.end()

    def actionproxy_cb(self, data):
        sdata = data.data
        if ('%quit_server' in sdata):
            self.end()

    def start(self, params=None):
        if self.cthread != None:
            self.end()
        self.do_run = True
        self.cthread = Thread(target=self.fluent_thread, args=(params,))
        self.cthread.start()

    def end(self):
        self.do_run = False
        if self.cthread != None:
            self.cthread.join()
        self.cthread = None

    def interrupt(self):
        self.end()

    def isRunning(self):
        if self.cthread != None and not self.cthread.is_alive():
            self.do_run = False
        return self.do_run

    def setValue(self, params, value):  # 1: true,  0: false,  -1: unknown
        self.lasttime = rospy.Time.now()
        if params == None or params == '':
            rospy.set_param(self.rosparam, value)
        else:
            rospy.set_param(self.rosparam+"_"+params, value)

    def getValue(self, params):
        v = -1
        try:
            v = rospy.get_param(self.rosparam+"_"+params)
        except:
            pass
        return v

    def execute(self, params):
        print("FluentProxy %s running ..." %(self.fluentname))

        self.start(params)
        while (self.isRunning()):
            try:
                #print("%s = %d" %(self.fluentname, self.getValue()))
                rospy.sleep(1)
            except KeyboardInterrupt:
                print("FluentProxy %s - user interrupt" %(self.fluentname))
                self.interrupt()
        self.end()

        print("FluentProxy %s quit" %(self.fluentname))


    def fluent_thread(self, params):
        pass


