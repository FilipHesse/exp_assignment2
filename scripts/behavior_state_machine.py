#!/usr/bin/env python3
"""Heart of robot_pet package: defines robots behavior

Contains a finite state machine implemented in smach. The 3 states of the
robot pet are NORMAL, PLAY and SLEEP. The state diagram can be found in the
README.md of the project

Each interface with the ROS infrastructure, such as service clients,
servers, action clients and publishers are implemented within separate
classes. All these interfaces are then passed to the smach-states while they
are constructed, in order to make the interfaces accessible for the states.

    Requirements:
        The following parameters need to be set in the ros parameter server:
            /map_width
            /map_height
        You can use the launchfile params.launch to set these to some 
        default values
"""

from __future__ import print_function
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import smach
import random
import smach_ros

from std_msgs.msg import Bool
from exp_assignment2.msg import PlanningAction, PlanningGoal
from exp_assignment2.msg import EmptyAction, EmptyGoal


class SetTargetActionClient():
    """Action client to set target position

    An action client has been chosen, because it is a non blocking call. This
    way, incoming commands can still be handeled properly and actions like
    "sleep" will be processed right after the service call has finished.

    To use this class, only use the functions call_action() to set a new target
    and check the value #TODO ready_for_new_target to check if previous action was 
    finished

    Attributes: 
        client (actionlib.SimpleActionClient): Clientobject to interface with 
            actual action 
    """

    def __init__(self):
        """Creates the client and waits for action server to be available
        """
        self.client = actionlib.SimpleActionClient(
            'set_target_position', PlanningAction)
        rospy.loginfo(
            "set_target_position_client: Waiting for action server to come up...")
        self.client.wait_for_server()

    def call_action(self, x, y):
        """Use this function to set a new target position of the robot_pet  

        Args:
            x (int): target x-position of the robot
            y (int): target y-position of the robot
        """
        if self.is_active():
            rospy.loginfo(
                "set_target_position_client: Trying to give new target position, but action server is still processing old goal")
            return

        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal,
                              done_cb=self.callback_done)

        rospy.loginfo("Goal (x={}, y={}) has been sent to the action server.".format(
            x, y))

    def callback_done(self, state, result):
        """This callback gets called when action server is done

        Args:
            state (state of action): Status of the action according to
                http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            result (SetTargetPositionResult): Result of action: Position of the point
                that was reached
        """
        rospy.loginfo("SetTargetAction is done!")

    def cancel_goal(self):
        self.client.cancel_goal()

    def is_active(self):
        return self.client.get_state() == GoalStatus.ACTIVE


class FollowBallActionClient():
    """Action client to set target position

    An action client has been chosen, because it is a non blocking call. This
    way, incoming commands can still be handeled properly and actions like
    "sleep" will be processed right after the service call has finished.

    Attributes: 
        client (actionlib.SimpleActionClient): Clientobject to interface with 
            actual action 
    """

    def __init__(self):
        """Creates the client and waits for action server to be available
        """
        self.client = actionlib.SimpleActionClient(
            'follow_ball', EmptyAction)
        rospy.loginfo(
            "follow_ball: Waiting for action server to come up...")
        self.client.wait_for_server()

    def call_action(self):
        """Use this function to set a new target position of the robot_pet  

        Args:
            x (int): target x-position of the robot
            y (int): target y-position of the robot
        """

        rospy.loginfo(
            "follow_ball: Action server has been called")

        if self.is_active():
            rospy.loginfo(
                "follow_ball: Trying to follow ball, but action server already busy doing it")
            return

        goal = PlanningGoal()
        self.client.send_goal(goal,
                              done_cb=self.callback_done)



    def callback_done(self, state, result):
        """This callback gets called when action server is done

        Args:
            state (state of action): Status of the action according to
                http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            result (SetTargetPositionResult): Result of action: Position of the point
                that was reached
        """
        rospy.loginfo(
            "FollowBallAction is done. Action state: {}".format(state))

    def cancel_goal(self):
        self.client.cancel_goal()

    def is_active(self):
        return self.client.get_state() == GoalStatus.ACTIVE


class SleepingTimer():
    """Timer Class, that schedules the sleeping times

    Contains a timer running in one-shot-mode. This allows to define different
    times for sleeping and being awake each time the timer has elapsed

    Usage: Check the flag time_to_sleep to check if robot should be sleeping
    right now or if it should be awake

    Attributes:
        sleeping_time_range ((int, int)): Sleeping time will be random between 10
            and 15 seconds
        awake_time_range  ((int, int)): Awake time will be random between 20 and 30 seconds
        time_to_sleep (bool): Flag for the user of this class to check if its time to sleep
            (True) or time to be awake (False)
        timer (rospy.Timer): Timer that triggers the callbacks
    """

    def __init__(self):
        """Initialize attributes
        """
        self.sleeping_time_range = (10, 15)  # Sleep between 10 and 15 seconds
        self.awake_time_range = (40, 60)  # Be awake for ...
        self.time_to_sleep = False
        self.timer = rospy.Timer(rospy.Duration(random.uniform(
            *self.awake_time_range)), self.callback, oneshot=True)

    def callback(self, msg):
        """Get called when self.timer has elapsed

        Toggles the flag time_to_sleep and restarts the timer with appropriate
        random time

        Args:
            msg (??): unused
        """
        self.time_to_sleep = not self.time_to_sleep

        if self.time_to_sleep:
            rospy.loginfo("It's time to go to bed!")
            self.timer = rospy.Timer(rospy.Duration(random.uniform(
                *self.sleeping_time_range)), self.callback, oneshot=True)
        else:
            rospy.loginfo("It's time to wake up!")
            self.timer = rospy.Timer(rospy.Duration(random.uniform(
                *self.awake_time_range)), self.callback, oneshot=True)


class BallVisibleSubscriber:
    def __init__(self):
        self.ball_visible = False
        self.sub = rospy.Subscriber(
            "camera1/ball_visible", Bool, self.callback)

    def callback(self, msg):
        self.ball_visible = msg.data

    def is_ball_visible(self):
        return self.ball_visible


########################################################
# STATE MACHINE CODE
#######################################################
class Normal(smach.State):
    """Defines the Smach-state NORMAL

    In this state the robot goes from one random target to another

    Attributes:
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
        map_width (int): Width of map to choose appropriate target positions
        map_height (int): Height of map to choose appropriate target positions
    """

    def __init__(self, set_target_action_client, ball_visible_subscriber, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """

        smach.State.__init__(self, outcomes=['sees_ball', 'sleeping_time'])

        self.set_target_action_client = set_target_action_client
        self.ball_visible_subscriber = ball_visible_subscriber
        self.sleeping_timer = sleeping_timer

        self.map_range_x = [-8, 8]
        self.map_range_y = [-8, 8]

        self.target_sent = False
                
        hz = 10
        self.rate = rospy.Rate(hz)
        sleep_seconds = 3
        self.iterations_to_sleep = sleep_seconds * hz

    def execute(self, userdata):
        """ Robot moves around between random positions

        Endless loop checks if it's time to sleep or if a user command has been
        sent to exit this state. Then it sends new position targets in case the
        robot is not already moving

        Args: userdata (----): unused

        Returns: string: outcomes "cmd_play" or "sleeping time
        """

        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE NORMAL ---\n--------------------------------------------------------------------------')

        sleep_iteration_counter = 0

        while True:
            # Check if its time to sleep
            if self.sleeping_timer.time_to_sleep:
                self.cancel_goal_and_wait_till_done(self.rate)
                return 'sleeping_time'

            if self.ball_visible_subscriber.is_ball_visible():
                rospy.loginfo('Ball seen! Canceling go_to_target')
                self.cancel_goal_and_wait_till_done(self.rate)
                return 'sees_ball'

            # Normal behavior: set random targets
            if not self.set_target_action_client.is_active():
                # Send Target
                if not self.target_sent:
                    next_x = random.uniform(
                        self.map_range_x[0], self.map_range_x[1])
                    next_y = random.uniform(
                        self.map_range_y[0], self.map_range_y[1])
                    self.set_target_action_client.call_action(next_x, next_y)
                    self.target_sent = True
                    sleep_iteration_counter = 0

                #Wait
                if self.target_sent:
                    # Is waiting time over? -> reset target_sent, so that new target can be sent
                    if sleep_iteration_counter >= self.iterations_to_sleep:
                        self.target_sent = False
                    else:
                        sleep_iteration_counter += 1
                        # sleeping happens at the end of loop

            self.rate.sleep()

    def cancel_goal_and_wait_till_done(self, rate):
        if self.set_target_action_client.is_active():
            self.set_target_action_client.cancel_goal()
            # Wait unitl client indeed is not active anymore (robot should have stopped)
            while self.set_target_action_client.is_active():
                self.rate.sleep()


class Sleep(smach.State):
    """Defines the Smach-state SLEEP

    In this state the robot goes to the house and stay there until sleeping time is over

    Attributes:
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
    """

    def __init__(self, set_target_action_client, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            sleeping_timer (SleepingTimer): See class description
        """

        smach.State.__init__(self, outcomes=['slept_enough'])

        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer

    def execute(self, userdata):
        """Robot goes to house and sleeps until sleeping time over

        Args:
            userdata (----): unused

        Returns:
            string: outcome: slept_enough
        """
        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE SLEEP ---\n--------------------------------------------------------------------------')
        self.rate = rospy.Rate(10)

        # Robot might still be moving (actually should not...)=> wait until last target reached
        while self.set_target_action_client.is_active():
            self.rate.sleep()

        # Get position of house
        x = rospy.get_param("/house_x")
        y = rospy.get_param("/house_y")

        # Set new target: house
        self.set_target_action_client.call_action(x, y)

        # just wait until wake-up flag is set
        while True:
            if not self.sleeping_timer.time_to_sleep:
                return 'slept_enough'
            self.rate.sleep()


# define state Play
class Play(smach.State):
    """Defines the Smach-state PLAY

    In this state the robot performs the following actions in a loop:
    1) Go to user
    2) Wait for a command that specifies a new target
    3) Go to new target
    Repeat

    The game is repeated for a randum number of times between 1 and 3

    Attributes:
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
        pub (rospy.Publisher()): A publisher to publish the pointer positions. This publisher is
            defined inside this state because it is not needed in any other state
    """

    def __init__(self, follow_ball_action_client, ball_visible_subscriber, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """
        smach.State.__init__(self, outcomes=['can_not_see_ball_3_s', 'sleeping_time'])
        self.follow_ball_action_client = follow_ball_action_client
        self.ball_visible_subscriber = ball_visible_subscriber
        self.sleeping_timer = sleeping_timer

        hz = 10
        self.rate = rospy.Rate(hz)
        no_ball_seconds = 3
        self.iterations_no_ball = no_ball_seconds * hz

    def execute(self, userdata):
        """Executes play mode

        In this state the robot performs the following actions in a loop:
        1) Go to user
        2) Wait for a command that specifies a new target
        3) Go to new target 
        4) Go back to person

        This function implements these steps and braks them down into smaller
        substeps Details can be viewed inside the code, which is commented. It
        is only checked at the end of each game (after going back to the
        person), if it is time to go to sleep or if the number of games to be
        played (random between 1 and 3) has been reached.

        Args:
            userdata (---): unused

        Returns:
            string: Outcomes of this state: "played_enough" or "sleeping_time"
        """
        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE PLAY ---\n--------------------------------------------------------------------------')

        counter_no_ball = 0

        # Call action client!
        follow_ball_action_client.call_action()

        while True:
            #If time to sleep, abort running action and change state
            if self.sleeping_timer.time_to_sleep:
                self.cancel_goal_and_wait_till_done()
                return 'sleeping_time'

            #If can not see ball for 3 seconds: abort running action and go back to normal state
            if not self.ball_visible_subscriber.is_ball_visible():
                counter_no_ball += 1
            else:
                counter_no_ball = 0
            
            if counter_no_ball >= self.iterations_no_ball:
                self.cancel_goal_and_wait_till_done()
                return 'can_not_see_ball_3_s'

            self.rate.sleep()

    def cancel_goal_and_wait_till_done(self):
        if self.follow_ball_action_client.is_active():
            self.follow_ball_action_client.cancel_goal()
            # Wait unitl client indeed is not active anymore (robot should have stopped)
            while self.follow_ball_action_client.is_active():
                self.rate.sleep()


if __name__ == "__main__":
    """Main function of this script

    Instanciates all classes, that have been defined above. Creates State
    machine with all the states and spins for callbacks
    """
    rospy.init_node('behavior_state_machine')

    set_target_action_client = SetTargetActionClient()
    follow_ball_action_client = FollowBallActionClient()
    sleeping_timer = SleepingTimer()
    ball_visible_subscriber = BallVisibleSubscriber()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=[])

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(set_target_action_client, ball_visible_subscriber, sleeping_timer),
                               transitions={'sees_ball': 'PLAY',
                                            'sleeping_time': 'SLEEP'})

        smach.StateMachine.add('SLEEP', Sleep(set_target_action_client, sleeping_timer),
                               transitions={'slept_enough': 'NORMAL'})

        smach.StateMachine.add('PLAY', Play(follow_ball_action_client, ball_visible_subscriber, sleeping_timer),
                               transitions={'can_not_see_ball_3_s': 'NORMAL',
                                            'sleeping_time': 'SLEEP'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
