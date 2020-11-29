#!/usr/bin/env python3
"""Simulated User Interface

This script creates a ROS node, which simulates a user interface to move the
ball.

The ball is moved to randompositions at random times, see class definitions for
details
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
    The time between two caommands is between 4 and 10 seconds
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
        rospy.sleep(random.uniform(4, 10))


if __name__ == "__main__":
    """Entry point of script
    """
    try:
        rospy.init_node('ui')
        ball_position_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")