#!/usr/bin/env python3
"""Simulated User Interface

This script creates a ROS node, which simulates a user interface to
communicate with the robot pet.

It calls the pet_command service to send a command which consists of a
string to specify the command type and a robot_pet/Point2d to specify the
desired position in case it is a "go_to" command.

The programmer has chosen a service over a publisher, because we want to
make sure no message gets lost.

    Requirements:
        The following parameters need to be set in the ros parameter server:
            /map_width
            /map_height
        You can use the launchfile params.launch to set these to some 
        default values
"""

from __future__ import print_function
import rospy
import random
import actionlib
import exp_assignment2.msg
import geometry_msgs

def ball_position_client():
    """Action client, that sends target ball positions to the ball/position server 

    The positions are random, each fivth position has a negative z_value (ball should disappear)
    The time between two caommands is between 2 and 6 seconds
    """


    client = actionlib.SimpleActionClient('/reaching_goal', exp_assignment2.msg.PlanningAction)
    rospy.logdebug("Wait for actionserver /reaching_goal to be available")
    client.wait_for_server()

    map_range_x = [-8, 8]
    map_range_y = [-8, 8]

    counter = 0
    while not rospy.is_shutdown():
        #Fill the goal
        goal = exp_assignment2.msg.PlanningGoal

        goal.header = rospy.Header()

        pose_stamped = geometry_msgs.msg.PoseStamped()
        #Each fivth position is below the ground
        if (counter % 5) == 4:  # 4, first time when counter = 5-1
            
            pose_stamped.pose.position.x = random.uniform(map_range_x[0],map_range_x[1])
            pose_stamped.pose.position.y = random.uniform(map_range_y[0],map_range_y[1])
            pose_stamped.pose.position.z = -1 # = balls radius
            
        else:
            pose_stamped.pose.position.x = random.uniform(map_range_x[0],map_range_x[1])
            pose_stamped.pose.position.y = random.uniform(map_range_y[0],map_range_y[1])
            pose_stamped.pose.position.z = 0.5 # = balls radius 
        
        goal.target_pose = pose_stamped

        rospy.loginfo("User sending ball goal: x={} y={} z={}".format(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z))
        client.send_goal(goal)
        
        client.wait_for_result()

        #incement counter
        counter += 1

        #Wait for a random time between 2 and 6 seconds 
        rospy.sleep(random.uniform(2, 6))


if __name__ == "__main__":
    """Entry point of script
    """
    try:
        rospy.init_node('ui')
        ball_position_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")