#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from exp_assignment2.msg import BallCenterRadius
from nav_msgs.msg import Odometry
from exp_assignment2.msg import EmptyAction, EmptyGoal
import actionlib


class BallFollower:
    def __init__(self):
        # Follow Ball active
        self.active = False

        # Action look_left_right currently active?
        self.look_left_right_active = False

        # Ball visibility, position, radius
        self.ball_visible = False
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.ball_radius = 0

        # Robot pose
        self.robot_pose = Pose()

        # Publisher, 2 Subscribers, ActionServer, ActionClient
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.sub_ball = rospy.Subscriber(
            "camera1/ball_center_radius", BallCenterRadius, self.callback_ball_center_radius)
        self.sub_odom = rospy.Subscriber(
            "odom", Odometry, self.callback_odometry)

        self.server = actionlib.SimpleActionServer(
            'follow_ball', EmptyAction, self.callback_follow_ball, False)
        self.server.start()

        self.client = actionlib.SimpleActionClient(
            '/robot/look_left_right', EmptyAction)
        self.client.wait_for_server()

    def callback_ball_center_radius(self, msg):
        self.ball_visible = msg.visible
        self.ball_center_x = msg.center_x.data
        self.ball_center_y = msg.center_y.data
        self.ball_radius = msg.radius.data

    def callback_odometry(self, msg):
        self.robot_pose = msg.pose.pose

    def callback_follow_ball(self, goal):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
        # Only do something, if ball_follower active and ball visible and not look_left_right active
            if self.ball_visible and not self.look_left_right_active:
                vel = Twist()
                # Rotate to get ball into center of image
                if abs(self.ball_center_x-400) > 20: #We are not properly aligned
                    vel.angular.z = 0.002*(self.ball_center_x-400)
                
                desired_radius = 250
                # We are aligned but too far or too close
                if abs(self.ball_center_x-400) < 30 and abs(desired_radius - self.ball_radius) > 10: 
                    vel.linear.x = 1*(desired_radius - self.ball_radius)

                # We are aligned and close enough
                if abs(self.ball_center_x-400) < 30 and abs(desired_radius - self.ball_radius) <= 10:
                    vel.linear.x = 0
                    vel.angular.z = 0

                    goal = EmptyGoal()
                    self.client.send_goal(goal, done_cb=self.callback_look_left_right_done)
                    self.look_left_right_active = True
                
                self.pub_cmd_vel.publish(vel)

            if self.server.is_preempt_requested():
                vel = Twist()
                self.pub_cmd_vel.publish(vel)
                self.client.cancel_goal()
                self.server.set_preempted()
                return
                

            r.sleep()
    
    def callback_stop_follow_ball(self):
        self.active = False
        # Cancel lool_left_right
        self.client.cancel_goal()

    def callback_look_left_right_done(self):
        self.look_left_right_active = False


if __name__ == '__main__':
    rospy.init_node('ball_follower')
    BallFollower()
 
