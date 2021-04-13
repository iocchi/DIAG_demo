# lightcolor fluent implementation

import sys

import rospy

from fluentproxy import FluentProxy

FLUENT_NAME = 'lightcolor'

class LightColorFluentProxy(FluentProxy):

    def __init__(self, fluentnane, rosnode=True):
        FluentProxy.__init__(self, fluentnane, rosnode)
        self.lightlist = ['printer1light', 'printer2light']
        self.colorlist = ['red', 'green', 'yellow']

    def __del__(self):
        FluentProxy.__del__(self)


    def match_colors(self, colorstr, col):
        return  \
            (colorstr=="red" and col['b']<0.1 and col['g']<0.1 and col['r']>0.9) or \
            (colorstr=="green" and col['b']<0.1 and col['g']>0.9 and col['r']<0.1) or \
            (colorstr=="yellow" and col['b']<0.1 and col['g']>0.9 and col['r']>0.9)


    def sensingStep(self):

        for lightname in self.lightlist:

            rosparam_state = lightname+"/state"   # light state [0, 1]
            rosparam_color = lightname+"/color"   # light color { 'b': [0,1], 'g': [0,1], 'r': [0,1] }

            try:
                s = rospy.get_param(rosparam_state)
                c = rospy.get_param(rosparam_color)
            except:
                s = 0
                c = { 'b': 0, 'g': 0, 'r': 0 }

            value = -1

            for colorname in self.colorlist:

                if s==1 and self.match_colors(colorname, c):
                    value = 1
                else:
                    value = 0
                
                fluent_params = lightname+"_"+colorname
                
                #print("Debug: %s_%s s:%d c:%r -> value %d" 
                #    %(self.fluentname,fluent_params,s,c,value))

                self.setValue(value,fluent_params)     # 1: true,  0: false,  -1: unknown

    # no input params
    def fluent_thread(self, params):

        while self.do_run:
            self.sensingStep()
            rospy.sleep(0.5)





if __name__ == "__main__":

    params = ''
    if (len(sys.argv)>1):
        params = sys.argv[1]

    t = LightColorFluentProxy(FLUENT_NAME)
    t.execute(params)  # blocking, CTRL-C to interrupt


