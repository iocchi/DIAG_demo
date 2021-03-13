#
# ActionProxy base class
#

import time
from threading import Thread

import rospy
from std_msgs.msg import String

'''
pnp_ros publishes a String topic named `pnp/action_str` in the form 

    [<robotname>#]<actionname>[_<params>].<command>

Action executors should listen to this message and execute the corresponding action. Programmer is responsible for ensuring that only one executor would take care of execution of an action.

Test with CLI

    rostopic pub pnp/action_str std_msgs/String "data: 'wait_10.start'" --once

Quit all action proxies with

    rostopic pub pnp/action_str std_msgs/String "data: '%quit_server'" --once

'''



# topic for subscribers
TOPIC_PNPACTIONPROXY_STR = "pnp/action_str"

# topic for publishers
TOPIC_PNPACTIONCMD = "pnp/actionCmd"

# param to notify action end
PARAM_PNPACTIONSTATUS = "pnp/actionStatus/"

class ActionProxy:

    def __init__(self, actionname):
        self.do_run = False
        self.athread = None
        self.actionname = actionname
        self.params = None
        self.interrupted = False

        # init ROS node
        nodename = actionname+"_actionproxy"
        rospy.init_node(nodename,  disable_signals=True)

        # subscribers
        self.actionproxy_sub = rospy.Subscriber(TOPIC_PNPACTIONPROXY_STR, String, self.actionproxy_cb)

        # publishers
        self.actioncmd_pub = rospy.Publisher(TOPIC_PNPACTIONCMD, String, queue_size=1)


    def __del__(self):
        # just in case
        self.end()

    def actionproxy_cb(self, data):
        sdata = data.data
        #print(sdata)

        if ('%quit_server' in sdata):
            self.quit_server()
            return

        robot = None
        action = None
        params = None
        command = None
        v = sdata.split('#')
        if len(v)>1:
            robot = v[0]
            sdata = v[1]
        v = sdata.split('.')
        if len(v)!=2:
            raise Exception("ActionProxy: wrong format in %s [%s]" %(TOPIC_PNPACTIONPROXY_STR, sdata))
        command = v[1]
        k = v[0].find('_')
        if (k<0):
            action = v[0]
        else:
            action = v[0][0:k]
            params = v[0][k+1:]

        if action==self.actionname:
            #print("robot: %s action: %s params: %s command: %s" \
            #    %(robot, action, params, command))
            if command=='start':
                self.start(params)

            elif command=='interrupt':
                self.interrupt()

            elif command=='resume':
                self.resume()

            elif command=='end':
                self.end()

            else:
                print("ActionProxy: wrong command %s" %(command))


    # start the action monitor thread / non-blocking
    def start(self, params=None):
        if self.athread != None:
            self.end()
        self.params = params
        print("ActionProxy %s - start %s" %(self.actionname,self.params))
        self.interrupted = False
        self.do_run = True
        self.athread = Thread(target=self.action_thread, args=(params,))
        self.athread.start()

    def interrupt(self):
        print("ActionProxy %s - interrupt" %(self.actionname))
        self.end()
        self.interrupted = True

    def resume(self):
        print("ActionProxy %s - resume" %(self.actionname))
        self.start(self.params)

    def end(self):
        print("ActionProxy %s - end" %(self.actionname))
        self.do_run = False
        #data = "%s_%s.end" %(self.actionname,self.params)
        #self.actioncmd_pub.publish(data)
        rospy.set_param(PARAM_PNPACTIONSTATUS+self.actionname, "end")
        if self.athread != None:
            self.athread.join()
        self.athread = None

    def isRunning(self):
        if self.athread != None and not self.athread.is_alive():
            self.do_run = False
        return self.do_run

    # exec the action / blocking, CTRL-C to interrupt
    def execute(self, params):
        self.start(params)
        while (self.isRunning()):
            try:
                rospy.sleep(1)
            except KeyboardInterrupt:
                print("ActionProxy %s - user interrupt" %(self.actionname))
                self.end()
        self.end()

    def run_server(self):   # keep the server running -> actions managed by actionproxy_cb
        print("ActionProxy %s running ..." %(self.actionname))

        rate = rospy.Rate(1)
        self.server_run = True
        while not rospy.is_shutdown() and self.server_run:
            # idle

            if self.athread != None and not self.athread.is_alive():
                if self.do_run: # if action is running, send end signal
                    self.end()
                self.do_run = False

            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("ActionProxy %s - user interrupt" %(self.actionname))
                self.server_run = False

        print("ActionProxy %s quit" %(self.actionname))

    def quit_server(self):
        self.server_run = False


    # to be defined by specific ActionProxy class
    def action_thread(self, params):
        pass


