# template for fluent implementation

import sys

import rospy

#sys.path.append("....")                  # <--- rospy PLEXI folder
from fluentproxy import FluentProxy

FLUENT_NAME = 'template'                  # <--- fluent name

class TemplateFluentProxy(FluentProxy):   # <--- fluent class

    def __init__(self, fluentnane):
        FluentProxy.__init__(self, fluentnane)
                                          # <--- fluent init


    def __del__(self):
        FluentProxy.__del__(self)
                                          # <--- fluent del


    def fluent_thread(self, params):
        v = params.split('_')             # <--- fluent params
        
        while self.do_run:
                                          # <--- fluent loop

            self.setValue(value)          # <--- set fluent value       
                                          # 1: true,  0: false,  -1: unknown
            rospy.sleep(0.25)

                                          # <--- fluent end




if __name__ == "__main__":

    params = ''
    if (len(sys.argv)>1):
        params = sys.argv[1]

    t = TemplateFluentProxy(FLUENT_NAME)  # <--- fluent class
    t.execute(params)  # blocking, CTRL-C to interrupt


