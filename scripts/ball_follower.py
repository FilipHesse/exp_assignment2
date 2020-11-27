#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool
import actionlib

class NumberCounter:
    def __init__(self):
        self.counter = 0
        self.pub_cmd_vel = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)

        self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)

        self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute, False)
        self.server.start()

        self.client = actionlib.SimpleActionClient('do_dishes', DoDishesAction)
        self.client.wait_for_server()

        goal = DoDishesGoal()
        # Fill in the goal here
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))

        new_msg = Int64()
        new_msg.data = self.counter
        self.pub.publish(new_msg)
 
    def callback_number(self, msg):
        self.counter += msg.data

    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff here
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('number_counter')
    NumberCounter()
    rospy.spin()
