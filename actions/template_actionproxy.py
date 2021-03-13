# template action

import sys

import rospy

#sys.path.append("....")                # <--- rospy PLEXI folder
from actionproxy import ActionProxy

ACTION_NAME = 'template'                # <--- action name

class TemplateActionProxy(ActionProxy): # <--- action class

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)
                                        # <--- action init

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
                                        # <--- action params

        while self.do_run:
                                        # <--- action loop
            rospy.sleep(0.25)

                                        # <--- action end


if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = TemplateActionProxy(ACTION_NAME)
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt

