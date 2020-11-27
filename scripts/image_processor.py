#!/usr/bin/env python3
"""Map ROS node

This script creates a ROS node, that bundles all the knowledge about the map
and all the objects in it.

    Requirements:
        The following parameters need to be set in the ros parameter server:
            /map_width
            /map_height
            /user_pos_x
            /user_pos_y
            /user_house_x
            /user_house_y
            /user_pet_x
            /user_pet_y
        You can use the launchfile params.launch to set these to some 
        default values
"""

from __future__ import print_function

from robot_pet.srv import GetPosition, GetPositionResponse
from robot_pet.msg import Point2d, Point2dOnOff
from sensor_msgs.msg import Image
import rospy

#imports for visualization
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os



class map_node:
    """A ROS node that contains 3 subsribers, one server and a publisher

    The three subscribers subscribe to the variable positions of the objects in
    the map: user, pet, pointer

    The server provides the service get_position to the ros environment.

    The publisher publishes an image of the current map each time the map is
    updated.

    The image can be observed with the node rqt_image_view/rqt_image_view.

    Attributes:
        map_width (int): Width of the rectangular map
        map_height (int): Height of the rectangular map
        icon_size (int): Size of the icons of the objects in the map: 32 
        positions (dict): Dictionary which contains the positions (list 
            of x and y) of all the objects present in the map
        icons (dict): Dictionary which contains the icons (array)
            of all the objects present in the map
        pointer_on (bool): Should the pointer position be diplayed?
        bridge (CvBridge): Bridge to create a ros image from a cv2 image
            This is necessary for publishing images in ros.
        pos_serv (rospy.Service): Server for get_position service
        sub_pet (rospy.Subscriber): Subscriber subscribes to pet position
        sub_user (rospy.Subscriber): Subscriber subscribes to user position
        sub_pointer (rospy.Subscriber): Subscriber subscribes to pointer position
        pub (rospy.Publisher): Publishes image of map
    """

    def __init__(self):
        """Initializes the attributes
        """

        self.map_width = rospy.get_param("/map_width")
        self.map_height = rospy.get_param("/map_height")

        self.icon_size = 32

        self.positions = {
            "user": [rospy.get_param("/user_pos_x"), rospy.get_param("/user_pos_y")],
            "house": [rospy.get_param("/house_pos_x"), rospy.get_param("/house_pos_y")],
            "pet": [rospy.get_param("/pet_pos_x"), rospy.get_param("/pet_pos_y")],
            "pointer": [0, 0]
        }

        #Read images for printing
        self.icons = {
            "user"   : cv2.imread(os.path.join(os.path.dirname(__file__), 'images/user.jpg'),0),
            "house"  : cv2.imread(os.path.join(os.path.dirname(__file__), 'images/house.jpg'),0),
            "pet"    : cv2.imread(os.path.join(os.path.dirname(__file__), 'images/pet.jpg'),0),
            "pointer": cv2.imread(os.path.join(os.path.dirname(__file__), 'images/pointer.jpg'),0)
        }

        self.pointer_on = False #Pointer is off in the beginning

        self.bridge = CvBridge()

        #Initialize services
        self.pos_serv = rospy.Service('get_position', GetPosition, self.handle_get_position)
        rospy.loginfo("Position server ready")
        
        #Initialize subscribers 
        self.sub_pet = rospy.Subscriber("pet_position", Point2d, self.callback_pet_position)
        self.sub_user = rospy.Subscriber("user_position", Point2d, self.callback_user_position)
        self.sub_pointer = rospy.Subscriber("pointer_position", Point2dOnOff, self.callback_pointer_position)

        #initialize publishers
        self.pub = rospy.Publisher('map', Image, queue_size=1)

        #Publish initial map
        self.publish_map()

    def handle_get_position(self, req):
        """Handles Service call get_position

        Args:
            req (string): Contains a string with the object of interest
                Possible strings are "user", "house", "pet", "pointer"

        Returns:
            GetPositionResponse: Contains a success flag to tell if the requested
                object could be found in the map. If the Object could be found, then
                its x and y position are also returned
        """
        rospy.loginfo("Position of {} requested".format(req.object))
        resp = GetPositionResponse()
        if req.object in self.positions:
            resp.success = True
            resp.point.x = self.positions[req.object][0]
            resp.point.y = self.positions[req.object][1]
            rospy.loginfo("Returning x={} y={}".format(resp.point.x, resp.point.y))
        else:
            rospy.loginfo("Object {} does not exist in dataset! Available objects are: {}".format(req.object, self.positions.keys()))
            resp.success = False
            resp.point.x = 0
            resp.point.y = 0  
        return resp

    def publish_map(self):
        """Creates an image of the map and publishes it

        The map can only represent integer positions. The integer dimensions defined in the ros
        parameter server. The image of the map contains a 32x32 pixel square for each position.
        
        This function simply creates a white rectangle of dimensions width*32 x height*32 and draws
        icons at the positions, which are saved in the membervariables
        """
        self.map_image = 255*np.ones((self.icon_size*self.map_height, self.icon_size*self.map_width), np.uint8) #White map
        for icon in self.icons:
            self.draw_icon(icon)
        self.pub.publish((self.bridge.cv2_to_imgmsg(self.map_image)))

    def draw_icon(self, icon):
        """Draws an icon on the map

        To draw an icon with black on white background, the function np.minimum is used.
        The positions of the icon are taken from the attributes of this class.

        Args:
            icon (string): String of the icon, that should be drawed
        """
        #If pointer off, do not draw it
        if icon == "pointer" and not self.pointer_on:
            return

        #Coordinates in Grid
        x = self.positions[icon][0]
        y = self.positions[icon][1]

        #Pixel coordinates
        x_start = self.icon_size * x
        y_start = self.icon_size * y
        x_end = x_start + self.icon_size
        y_end = y_start + self.icon_size

        #
        self.map_image[y_start:y_end , x_start:x_end] = np.minimum(self.map_image[y_start:y_end , x_start:x_end], self.icons[icon])



    def callback_pet_position(self, data):
        """Callback function gets called when somethin was written to the topic pet_position

        If position is different from the saved position: Save it and publish the updated map

        Args:
            data (robot_pet/Point2d): Position of the pet 
        """
        if not self.positions["pet"] == [data.x, data.y]:
            self.positions["pet"] = [data.x, data.y]
            rospy.loginfo("New pet position: x={} y={}".format(self.positions["pet"][0], self.positions["pet"][1]))
            self.publish_map()

    def callback_user_position(self, data):
        """Callback function gets called when somethin was written to the topic user_position

        If position is different from the saved position: Save it and publish the updated map

        Args:
            data (robot_pet/Point2d): Position of the user
        """
        if not self.positions["user"] == [data.x, data.y]:
            self.positions["user"] = [data.x, data.y]
            rospy.loginfo("New user position: x={} y={}".format(self.positions["user"][0], self.positions["user"][1]))
            self.publish_map()

    def callback_pointer_position(self, data):
        """Callback function gets called when somethin was written to the topic pointer_position

        Save position and the flag if the pointer should be displayed. Then publish the updated map

        Args:
            data (robot_pet/Point2dOnOff): Position of the pointer and flag if it should be displayed or not 
        """
        self.positions["pointer"] = [data.point.x, data.point.y]
        self.pointer_on = data.on
        if self.pointer_on:
            rospy.loginfo("New Pointer position: x={} y={}".format(self.positions["pointer"][0], self.positions["pointer"][1]))
        else:
            rospy.loginfo("Pointer was switched off")
        self.publish_map()

if __name__ == "__main__":
    """Entry point of this script initializes node
    """
    rospy.init_node('map')

    #instanciate map_node class
    map_node()

    rospy.spin()