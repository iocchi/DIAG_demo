# goto action

import time, math, sys, random

import rospy
import tf

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from actionproxy import ActionProxy

ACTION_NAME = 'goto'
ACTION_move_base = 'move_base'  # ROS action
TOPIC_amcl_pose = 'amcl_pose'   # localizer pose


class GotoActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)
        self.ac_movebase = None
        self.map_robot_pose = [0,0,0]
        self.ac_movebase = actionlib.SimpleActionClient(ACTION_move_base,MoveBaseAction)
        self.ac_movebase.wait_for_server()

    def __del__(self):
        ActionProxy.__del__(self)

    def distance(self, p1, p2):
        return math.sqrt(math.pow(p2[0]-p1[0],2)+math.pow(p2[1]-p1[1],2))

    def get_gotopose(self, label):
        pname = "/map_server/%s/gotopose" %label
        if rospy.has_param(pname):
            p = rospy.get_param(pname)
            r = [ p[0], p[1], p[2] ]
            rospy.loginfo("Location %s: %r." %(params,r))
        else:
            rospy.logwarn("Location %s not found." %params)
            r = None
        return r

    def target_params(self, params):
        #print(params)
        r = None
        v = params.split('_')
        if (len(v)==3):
            r =[float(v[0]), float(v[1]), float(v[2])]
        else:
            # get pose from semantic map
            if params=='random':
                params = random.choice(['printer1', 'printer2', 'door1', 'door2', 'door3'])

            print("goto %s ..." %params)
            r = self.get_gotopose(params)

        return r

    def getRobotPose(self):
        return self.map_robot_pose

    def localizer_cb(self, data):
        self.map_robot_pose[0] = data.pose.pose.position.x
        self.map_robot_pose[1] = data.pose.pose.position.y
        o = data.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(q)
        self.map_robot_pose[2] = euler[2] # yaw

    def send_goal(self, target_pose):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_pose[0]
        goal.target_pose.pose.position.y = target_pose[1]
        yaw = target_pose[2]/180.0*math.pi
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

        self.ac_movebase.send_goal(goal)

        rospy.loginfo("move_base: goal %r sent!" %(target_pose))

    def action_thread(self, params):

        target_pose = self.target_params(params)
        if target_pose is None:
            return

        self.loc_sub = rospy.Subscriber(TOPIC_amcl_pose, PoseWithCovarianceStamped, self.localizer_cb)

        self.send_goal(target_pose)

        #d = self.distance(self.map_robot_pose, target_pose)
        #print(d)
        finished = False # d < 1.0
        while self.do_run and not finished:
            self.ac_movebase.wait_for_result(rospy.Duration(1))
            status = self.ac_movebase.get_state() # 1 ACTIVE, 3 SUCCEEDED, 4 ABORTED
            result = self.ac_movebase.get_result() 
            
            #d = self.distance(self.map_robot_pose, target_pose)
            #print("%.1f %r %r" %(d,status,result))

            finished = (status == GoalStatus.SUCCEEDED) or (status == GoalStatus.ABORTED)

        state = self.ac_movebase.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("move_base: goal succeeded!")
        else:
            rospy.loginfo("move_base: goal failed!")

        self.ac_movebase.get_result()

        self.ac_movebase.cancel_all_goals()

        self.loc_sub.unregister()



if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = GotoActionProxy(ACTION_NAME)
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt

