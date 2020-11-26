#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import exp_assignment2.msg

from math import pow, atan2, sqrt, pi
from tf import transformations
import actionlib

class Pose():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

class RobotNavigator:
    # create messages that are used to publish feedback/result
    _feedback = exp_assignment2.msg.PlanningFeedback()
    _result = exp_assignment2.msg.PlanningResult()

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('odom',
                                                Odometry, self.update_pose)

        #An action server for receiving target positions
        self._action_name = "set_target_position"
        self._as = actionlib.SimpleActionServer(self._action_name, exp_assignment2.msg.PlanningAction, execute_cb=self.move2goal, auto_start = False)
        self._as.start()

        self.pose = Pose()
        self.rate = rospy.Rate(10)
        

    def update_pose(self, data):
        """Callback function which is called when a new message of type Odometry is
        received by the subscriber."""
        odom = data
        position = odom.pose.pose.position

        self.pose.x = round(position.x, 4)
        self.pose.y = round(position.y, 4)

        orientation_q = odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        yaw = transformations.euler_from_quaternion (orientation_list) [2]  #Third element is yaw angle

        self.pose.theta = yaw

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        p = rospy.get_param("p_linear")
        thr = rospy.get_param("thr_linear")
        vel = p * self.euclidean_distance(goal_pose)
        if vel > thr:
            vel = thr
        if vel < -thr:
            vel = -thr
        return vel

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        

    def angle_diff(self,goal_pose):
        angle = self.steering_angle(goal_pose) - self.pose.theta
        if angle > pi:
            angle = angle - 2*pi
        if angle < -pi:
            angle = angle + 2*pi
        return angle

    def angular_vel(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        p = rospy.get_param("p_angular")
        thr = rospy.get_param("thr_angular")
        rospy.loginfo("steering angle: {} robots angle: {}".format(self.steering_angle(goal_pose), self.pose.theta))
        vel = -p * self.angle_diff(goal_pose)  #MINUS, because we want to ratate in the inverse direction of the error angle (to make it zero!)
        if vel > thr:
            vel = thr
        if vel < -thr:
            vel = -thr
        return vel

    def move2goal(self, goal):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        goal_pose.x = goal.target_pose.pose.position.x
        goal_pose.y = goal.target_pose.pose.position.y
        rospy.loginfo("navigator: new goal received: x={} y={}".format(goal_pose.x, goal_pose.y))

        distance_tolerance = 0.1

        vel_msg = Twist()
        aligning_done = False

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            #Send feedback of action server
            self._feedback.position.position.x = self.pose.x
            self._feedback.position.position.y = self.pose.y
            self._feedback.stat = "moving"

            self._as.publish_feedback(self._feedback)
            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            if not aligning_done and abs(self.angle_diff(goal_pose)) > pi/20:
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)
            else:
                aligning_done = True
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

        

if __name__ == '__main__':
    try:
        n = RobotNavigator()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass