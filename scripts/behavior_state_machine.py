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
from robot_pet.srv import PetCommand, PetCommandResponse, PetCommandRequest, GetPosition, GetPositionRequest
from robot_pet.msg import SetTargetPositionAction, SetTargetPositionGoal, SetTargetPositionResult, Point2dOnOff
import rospy
import actionlib
import smach
import random
import smach_ros


class PetCommandServer:
    """Server to process Pet Commands

    If a new command comes in, this class saves it as a member and sets a flag,
    that a new command is available.

    The member _command should not be accessed directly, but through the
    function get_new_command(), because that function automatically sets the
    _new_command_available-flag to False.


    Attributes:
        _command (bool): Content of last command
        _new_command_available (PetCommandRequest): Is a new unprocessed command available?
    """

    def __init__(self):
        """Initializes a service and the attributes
        """

        rospy.Service('pet_command', PetCommand, self.handle_command)
        rospy.loginfo("Ready to receive commands.")
        self._new_command_available = False 
        self._command = PetCommandRequest()    #Make variables private, so that get_new_command() has to be used


    def handle_command(self, req):
        """Saves command to Atrributes and returns an empty response to caller

        Args:
            req (PentCommandRequest): Service request containing a command and 
                optianally a targetpoint

        Returns:
            [PetCommandResponse]: Empty response
        """

        rospy.loginfo("Command received: {} {} {}".format(req.command, req.point.x, req.point.y))
        self._command = req
        self._new_command_available = True
        return PetCommandResponse()


    def is_new_command_available(self):
        """Call this function to check if new command is available

        Returns:
            bool: True if unprocessed command is available
        """

        return self._new_command_available


    def get_new_command(self):
        """Interface for state machine to get new command

        Returns:
            PetCommandRequest: Command as it was received by the server
        """

        self._new_command_available = False
        return self._command



class GetPositionClient(): 
    """Service client to get the position of an object from map
    """

    def __init__(self):
        """Waits for service to be available
        """
        rospy.loginfo("Wait for service get_position to be available")
        rospy.wait_for_service('get_position')

    def call_srv(self, obj): 
        """Calls the service get_position

        Args:
            obj (string): Request the position of an object, which might be
                - "user"
                - "house"
                - "pet"
                - "pointer"

        Returns:
            (int, int): x and y position of the target
        """
        try:
            get_position = rospy.ServiceProxy('get_position', GetPosition)

            #Fill the request
            request = GetPositionRequest()
            request.object = obj
            rospy.loginfo("Requesting Position of {}".format(request.object))

            #Call service
            res = get_position(request)

            #Check result and return it
            if res.success == True:
                rospy.loginfo("Position of {} returned: x={} y={}".format(request.object, res.point.x, res.point.y))
                return res.point.x, res.point.y
            else:
                rospy.loginfo("Position of {} not available!".format(request.object))
                return None, None

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)



class SetTargetActionClient():
    """Action client to set target position

    An action client has been chosen, because it is a non blocking call. This
    way, incoming commands can still be handeled properly and actions like
    "sleep" will be processed right after the service call has finished.

    To use this class, only use the functions call_action() to set a new target
    and check the value ready_for_new_target to check if previous action was 
    finished

    Attributes: 
        ready_for_new_target (bool): True if the last action 
            has been finished
        client (actionlib.SimpleActionClient): Clientobject to interface with 
            actual action 
    """
    def __init__(self):
        """Creates the client and waits for action server to be available
        """
        self.ready_for_new_target = True
        self.client = actionlib.SimpleActionClient('set_target_position_as', SetTargetPositionAction)
        rospy.loginfo("Waiting for action server to come up...")
        self.client.wait_for_server()

    def call_action(self, x, y):
        """Use this function to set a new target position of the robot_pet  

        Args:
            x (int): target x-position of the robot
            y (int): target y-position of the robot
        """
        goal = SetTargetPositionGoal()
        self.ready_for_new_target = False
        goal.target.x = x
        goal.target.y = y
        self.client.send_goal(goal,
                        done_cb=self.callback_done)

        rospy.loginfo("Goal (x={}, y={}) has been sent to the action server.".format(goal.target.x, goal.target.y))

    def callback_done(self, state, result):
        """This callback gets called when action server is done

        Sets attribute ready_for_new_target to true

        Args:
            state (state of action): Status of the action according to
                http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            result (SetTargetPositionResult): Result of action: Position of the point
                that was reached
        """
        rospy.loginfo("SetTargetAction is done, position x={} y={} reached. Action state: {}".format(result.final_position.x, result.final_position.y, state))
        self.ready_for_new_target = True



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
        self.sleeping_time_range = (10, 15)  #Sleep between 10 and 15 seconds
        self.awake_time_range = (20, 30)  #Be awake for ...
        self.time_to_sleep = False
        self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.awake_time_range)), self.callback, oneshot=True)

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
            self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.sleeping_time_range)), self.callback, oneshot=True)
        else:
            rospy.loginfo("It's time to wake up!")
            self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.awake_time_range)), self.callback, oneshot=True)



########################################################
## STATE MACHINE CODE
#######################################################

class Normal(smach.State):
    """Defines the Smach-state NORMAL

    In this state the robot goes from one random target to another

    Attributes:
        get_position_client (GetPositionClient): Service client to get position of an object
        pet_command_server (PetCommandServer): Allows accessing the last command from the user
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
        map_width (int): Width of map to choose appropriate target positions
        map_height (int): Height of map to choose appropriate target positions
    """

    def __init__(self, get_position_client, pet_command_server, set_target_action_client, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            get_position_client (GetPositionClient): See class description
            pet_command_server (PetCommandServer): See class description
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """

        smach.State.__init__(self, outcomes=['cmd_play','sleeping_time'])
        
        self.get_position_client = get_position_client
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer

        self.map_width = rospy.get_param("/map_width")
        self.map_height = rospy.get_param("/map_height")
        

    def execute(self, userdata):
        """ Robot moves around between random positions

        Endless loop checks if it's time to sleep or if a user command has been
        sent to exit this state. Then it sends new position targets in case the
        robot is not already moving

        Args: userdata (----): unused

        Returns: string: outcomes "cmd_play" or "sleeping time
        """

        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE NORMAL ---\n--------------------------------------------------------------------------')
        rate = rospy.Rate(10)
        while True:
            # Check if its time to sleep
            if self.sleeping_timer.time_to_sleep:
                while not self.set_target_action_client.ready_for_new_target:
                    rate.sleep()
                return 'sleeping_time'

            # React to user commands
            if self.pet_command_server.is_new_command_available():
                cmd = self.pet_command_server.get_new_command()
                if cmd.command == 'play':
                    #First reach next position then go to sleeping state
                    while not self.set_target_action_client.ready_for_new_target:
                        rate.sleep()
                    return 'cmd_play'
                if cmd.command =='go_to':
                    rospy.loginfo("Invalid command 'go_to' for state NORMAL. First say 'play' and then give go_to commands!")

            # Normal behavior: set random targets
            if self.set_target_action_client.ready_for_new_target:
                next_x = random.randint(0,self.map_width-1)
                next_y = random.randint(0,self.map_height-1) 
                self.set_target_action_client.call_action(next_x, next_y)
            
            rate.sleep()



class Sleep(smach.State):
    """Defines the Smach-state SLEEP

    In this state the robot goes to the house and stay there until sleeping time is over

    Attributes:
        get_position_client (GetPositionClient): Service client to get position of an object
        pet_command_server (PetCommandServer): Allows accessing the last command from the user
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
    """
    def __init__(self, get_position_client, pet_command_server, set_target_action_client, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            get_position_client (GetPositionClient): See class description
            pet_command_server (PetCommandServer): See class description
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """

        smach.State.__init__(self, outcomes=['slept_enough'])

        self.get_position_client = get_position_client
        self.pet_command_server = pet_command_server
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
        rate = rospy.Rate(10)
        
        # Robot might still be moving => wait until last target reached
        while not self.set_target_action_client.ready_for_new_target:
            rate.sleep()
        
        #Get position of house
        x,y = get_position_client.call_srv("house")
        
        #Set new target: house
        set_target_action_client.call_action(x,y)

        #just wait until wake-up flag is set
        while True:
            
            if self.pet_command_server.is_new_command_available():
                # Ignore user commands -> Get the commands to consider command as hadeled
                cmd = self.pet_command_server.get_new_command()
                rospy.loginfo("Command is ignored, because Robot is sleeping")

            if not self.sleeping_timer.time_to_sleep:
                return 'slept_enough'
            rate.sleep()
            

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
        get_position_client (GetPositionClient): Service client to get position of an object
        pet_command_server (PetCommandServer): Allows accessing the last command from the user
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
        pub (rospy.Publisher()): A publisher to publish the pointer positions. This publisher is
            defined inside this state because it is not needed in any other state
    """
    def __init__(self, get_position_client, pet_command_server, set_target_action_client, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            get_position_client (GetPositionClient): See class description
            pet_command_server (PetCommandServer): See class description
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """
        smach.State.__init__(self, outcomes=['played_enough','sleeping_time'])
        self.get_position_client = get_position_client
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer
        self.pub = rospy.Publisher('pointer_position', Point2dOnOff, queue_size=10) #define this publisher only here, because pointer position is not needed anywhere else

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
        rate = rospy.Rate(10)

        games_to_play = random.randint(1,3)

        number_games = 0
        while True:
            number_games += 1
            #Get Persons Position
            x,y = get_position_client.call_srv("user")
            #Go To Person
            set_target_action_client.call_action(x,y)
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                rate.sleep()
            
            #Discard all previous commands
            if self.pet_command_server.is_new_command_available():
                # Ignore user commands -> Get the commands to consider command as hadeled
                cmd = self.pet_command_server.get_new_command()
                rospy.loginfo("Previous commands were ignored, because robot was not ready to receive commands")
            
            valid_target = False
            #Check commands until a valid one comes in (go to)
            while not valid_target:
                #Wait for command
                while not self.pet_command_server.is_new_command_available():
                    rate.sleep()

                cmd = self.pet_command_server.get_new_command()
                if cmd.command == 'play':
                    rospy.loginfo("Robot is already playing")

                if cmd.command =='go_to':
                    valid_target = True
                    x = cmd.point.x
                    y = cmd.point.y

            # Send pointer position to map
            pointer_pos = Point2dOnOff()
            pointer_pos.point.x = x
            pointer_pos.point.y = y
            pointer_pos.on =  True  #SWITCH ON
            self.pub.publish(pointer_pos)

            #Go To Target
            set_target_action_client.call_action(x,y)
            
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                rate.sleep()

            # Send pointer position to map
            pointer_pos.on =  False     #SWITCH OFF
            self.pub.publish(pointer_pos)

            #Get Persons Position
            x,y = get_position_client.call_srv("user")
            
            #Go To Person
            set_target_action_client.call_action(x,y)
            
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                rate.sleep()
            #Check if time to sleep
            if self.sleeping_timer.time_to_sleep:
                rospy.loginfo("I am tired. Good night!")
                return 'sleeping_time'

            #Check if played enough
            if games_to_play == number_games:
                rospy.loginfo("I played {} games. This is enough".format(number_games))
                return 'played_enough'

            rate.sleep()



if __name__ == "__main__":
    """Main function of this script

    Instanciates all classes, that have been defined above. Creates State
    machine with all the states and spins for callbacks
    """
    rospy.init_node('behavior_state_machine')

    
    get_position_client = GetPositionClient()
    pet_command_server = PetCommandServer()
    set_target_action_client = SetTargetActionClient()
    sleeping_timer = SleepingTimer()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=[])

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(get_position_client, pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'cmd_play':'PLAY', 
                                            'sleeping_time':'SLEEP'})

        smach.StateMachine.add('SLEEP', Sleep(get_position_client, pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'slept_enough':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(get_position_client, pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'played_enough':'NORMAL',
                                            'sleeping_time':'SLEEP' })


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start() 

    # Execute SMACH plan
    outcome = sm_top.execute()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass