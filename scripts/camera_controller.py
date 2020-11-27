#! /usr/bin/env python3

import roslib
import rospy
import actionlib

from exp_assignment2.msg import EmptyAction
from std_msgs.msg import Float64
from math import pi


class CameraController:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'look_left_right', EmptyAction, self.execute, False)
        self.server.start()
        self.pub = rospy.Publisher(
            "camera_position_controller/command", Float64, queue_size=10)

        self.action_active = False  

        self.publish_continuously_zero()

    def publish_continuously_zero(self):
        r = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            if not self.action_active:
                self.pub.publish(Float64(0))
                r.sleep()

    def execute(self, goal):
        self.action_active = True
        self.pub.publish(Float64(pi/4))
        rospy.sleep(2)
        self.pub.publish(Float64(-pi/4))
        rospy.sleep(2)
        self.pub.publish(Float64(0))
        self.server.set_succeeded()
        self.action_active = False


if __name__ == '__main__':
    rospy.init_node('camera_controller')
    server = CameraController()
    rospy.spin()
