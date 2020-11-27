#! /usr/bin/env python3

import roslib
import rospy
import actionlib

from exp_assignment2.msg import LookLeftRightAction
from std_msgs.msg import Float64


class CameraController:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'do_dishes', LookLeftRightAction, self.execute, False)
        self.server.start()
        self.pub = rospy.Publisher("camera_position_controller/command", Float64, queue_size=10)

        self.publish_continuously_zero()

    def publish_continuously_zero(self):
        r = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            self.pub.publish(Float64(0))
            r.sleep()

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('camera_controller')
    server = CameraController()
    rospy.spin()
