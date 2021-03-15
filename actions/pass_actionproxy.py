# goto action

import time, math, sys, random

import rospy
import tf

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

sys.path.append("..")  # actionproxy
from actionproxy import ActionProxy

ACTION_NAME = 'pass'
TOPIC_amcl_pose = 'amcl_pose'   # localizer pose
TOPIC_cmd_vel = 'cmd_vel'
TOPIC_desired_cmd_vel = 'desired_cmd_vel'
TOPIC_odom = 'odom'
PARAM_gbnEnabled = '/gradientBasedNavigation/gbnEnabled'


'''
Pass through a door

Requires definition in the semantic map

    door3in:  { gotopose: [ 9.9, -2.9, 90 ] }
    door3out: { gotopose: [ 9.75, -1.3, -90 ] }

param: doorname (e.g., door3)

'''

def NORM_PI(a):
    if (a>math.pi):
        return a-2*math.pi
    elif (a<-math.pi):
        return a+2*math.pi
    else:
        return a


class PassActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)
        self.map_robot_pose = [0,0,0]
        self.odom_robot_pose = [0,0,0]
        self.odom_robot_vel = [0,0]
        self.cmd_pub = rospy.Publisher(TOPIC_cmd_vel, Twist, queue_size=1)
        self.des_cmd_pub = rospy.Publisher(TOPIC_desired_cmd_vel, Twist, queue_size=1)

    def __del__(self):
        ActionProxy.__del__(self)

    def distance(self, p1, p2):
        return math.sqrt(math.pow(p2[0]-p1[0],2)+math.pow(p2[1]-p1[1],2))

    def angle(self, p1, p2):
        return math.atan2(p2[1]-p1[1], p2[0]-p1[0])  # return [-pi, +pi]

    def get_gotopose(self, label):
        pname = "/map_server/%s/gotopose" %label
        if rospy.has_param(pname):
            p = rospy.get_param(pname)
            r = [ p[0], p[1], p[2] ]
            rospy.loginfo("Location %s: %r." %(label,r))
        else:
            rospy.logwarn("Location %s not found." %label)
            r = None
        return r

    def target_params(self, params):
        v = params.split('_')
        doorname = v[0] # e.g. door1

        p1 = self.get_gotopose(doorname+'in')
        p2 = self.get_gotopose(doorname+'out')

        if p1==None or p2==None:
            return None

        d1 = self.distance(self.map_robot_pose, p1)
        d2 = self.distance(self.map_robot_pose, p2)

        return p2 if (d1 < d2) else p1

    def getRobotPose(self):
        return self.map_robot_pose

    def localizer_cb(self, data):
        self.map_robot_pose[0] = data.pose.pose.position.x
        self.map_robot_pose[1] = data.pose.pose.position.y
        o = data.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(q)
        self.map_robot_pose[2] = euler[2] # yaw

    def odom_cb(self, data):
        self.odom_robot_pose[0] = data.pose.pose.position.x
        self.odom_robot_pose[1] = data.pose.pose.position.y
        o = data.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(q)
        self.odom_robot_pose[2] = euler[2] # yaw
        self.odom_robot_vel[0] = data.twist.twist.linear.x
        self.odom_robot_vel[1] = data.twist.twist.angular.z


    def setSpeed(self,lx,az,tm,stopend=False):

        delay = 0.1 # sec
        rate = rospy.Rate(1/delay) # Hz
        cnt = 0.0
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y =  0
        while not rospy.is_shutdown() and cnt<=tm:
            self.des_cmd_pub.publish(msg)
            cnt = cnt + delay
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("User KeyboardInterrupt")
                return False
        if (stopend):
            msg.linear.x = 0
            msg.angular.z = 0
            self.des_cmd_pub.publish(msg)
            try:
                rate.sleep()
            except:
                pass
        return True


    def exec_move_REL(self, tx):
        tv_good = 0.4
        tv_min = 0.1

        start_pose = list(self.odom_robot_pose)
        tv_nom = tv_good 
        r = True
        if (tx < 0):
            tv_nom *= -1
            tx *= -1
        dx = abs(self.distance(start_pose,self.odom_robot_pose) - tx)
        last_dx = dx
        while (dx>0.1 and last_dx>=dx):
            tv = tv_nom
            if (dx<0.5):
                tv = tv_nom*dx/0.5
            if (abs(tv)<tv_min):
                tv = tv_min*tv/abs(tv)
            rv = 0.0
            if self.setSpeed(tv, rv, 0.1, False):
                dx = abs(self.distance(start_pose,self.odom_robot_pose) - tx)
                if (dx < last_dx or dx>0.3): # to avoid oscillation close to 0
                    last_dx = dx
            else:
                print("move action canceled by user")
                r = False
                dx = 0
            #print("MOVE -- POS: %.2f %.2f  -- targetTX %.2f DX %.2f -- VEL: %.2f %.2f" %(self.odom_robot_pose[0], self.odom_robot_pose[1], tx, dx, tv, rv))
        self.setSpeed(0.0,0.0,0.1)
        return r

    def exec_turn_ABS(self, th):
        rv_good = 0.5 
        rv_min = 0.1

        current_th = self.odom_robot_pose[2]
        target_th = current_th + th - self.map_robot_pose[2]
        #print("TURN -- mapTh: %.3f -- absTh %.3f" %(self.map_robot_pose[2],th))
        #print("TURN -- currentTh: %.3f -- targetTh %.3f" %(current_th,target_th))

        r = True

        dth = NORM_PI(target_th-current_th)

        rv_nom = rv_good 
        if (dth < 0):
            rv_nom *= -1

        adth = abs(dth)
        last_adth = adth
        #print("TURN -- last_adth %.2f" %(last_adth))
        while (adth>rv_min/8.0 and last_adth>=adth):
            rv = rv_nom
            if (adth<0.8):
                rv = rv_nom*adth/0.8
            if (abs(rv)<rv_min):
                rv = rv_min*rv/abs(rv)
            tv = 0.0
            if self.setSpeed(tv, rv, 0.1, False):
                current_th = self.odom_robot_pose[2]
                dth = NORM_PI(target_th-current_th)
                adth = abs(dth)
                #print("TURN -- dTh %.3f" %(target_th-current_th))
                if (adth < last_adth or adth>0.3): # to avoid oscillation close to 0
                    last_adth = adth
            else:
                print("turn action canceled by user")
                r = False
                adth=0
            #print("TURN -- POS: %.3f -- targetTh %.3f dth %.3f adth %.3f -- VEL: %.2f %.2f" %(current_th, target_th, dth, adth, tv, rv))
        #print("TURN -- adth %.2f - last_adth %.2f" %(adth,last_adth))
        self.setSpeed(0.0,0.0,0.1)
        #print 'TURN -- end'
        return r



    def action_thread(self, params):

        self.loc_sub = rospy.Subscriber(TOPIC_amcl_pose, PoseWithCovarianceStamped, self.localizer_cb)
        self.odom_sub = rospy.Subscriber(TOPIC_odom, Odometry, self.odom_cb)
        rospy.sleep(0.2) # wait to get pose from localizer

        target_pose = self.target_params(params)
        if target_pose is None:
            return

        # turn to target_pose
        a = self.angle(self.map_robot_pose, target_pose)
        self.exec_turn_ABS(a)

        # pass through door with gbn

        rospy.set_param(PARAM_gbnEnabled, True)

        d = self.distance(self.map_robot_pose, target_pose)
        self.exec_move_REL(d)

        rospy.set_param(PARAM_gbnEnabled, False)

        self.loc_sub.unregister()
        self.odom_sub.unregister()


if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = PassActionProxy(ACTION_NAME)
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt

