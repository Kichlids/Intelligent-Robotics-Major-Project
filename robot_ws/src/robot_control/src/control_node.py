#!/usr/bin/env python

import rospy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from path_finding import Graph
from path_finding import Astar

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Int32


#from robot_msgs.msg import coordinate
#from robot_msgs.msg import tasks

from tf.transformations import euler_from_quaternion



# Speed ft/s
LINEAR_SPEED_DEFAULT = 0.25
# Rotation speed rad ft/s 
ANGULAR_SPEED_DEFAULT = 0.25

linear_speed_fast = 2

# Obstacle avoidance threshold in ft, including the position of the laser scan sensor
LASER_AVOIDANCE_DISTANCE = 0.5#1.5

'''
If robot moves away this much distance (ft),
abandon this waypoint (and its destination if applicable)
and move on to next waypoint
'''
NAV_FAILURE_DISTANCE_THRESHOLD = 2

dist_diff_threshold = 0.1



class Coord():

    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def to_string(self):
        return str(self.x) + ', ' + str(self.y)


my_location = Coord(0, 0)
current_node = 'Node5'
destination_node = ''
node_path = []

yaw = 0.0
waypoints = []




class Odom():
    def __init__(self):
        self.support = Support()
        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
    def odom_callback(self, data):
        global my_location
        global yaw

        y = self.support.meters_to_feet(round(data.pose.pose.position.x, 2) + 1)
        x = self.support.meters_to_feet(round(-data.pose.pose.position.y, 2) + 11)

        #print(str(x) + ', ' + str(y))

        my_location = Coord(x, y)

        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw_rad) = euler_from_quaternion (orientation_list)
        yaw = -math.degrees(yaw_rad)

class Laser():

    def __init__(self):
        self.support = Support()

        self.obstacle_detected = False
        self.laser_data = LaserScan()
        self.laser_min_index = 0

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    # Find the minimum distance and its index of the ranges
    def find_min_laser_data(self, array):
        min_val = array[0]
        min_index = 0

        #print(len(array)) # 640

        if math.isnan(array[320]):
            return array[320], 320
        if math.isnan(array[0]):
            return array[0], 0
        if math.isnan(array[639]):
            return array[639], 639

        for i in range(len(array)):
            if min_val > array[i]: #or math.isnan(min_val):
                min_val = array[i]
                min_index = i
        
        return min_val, min_index

    # Laser Scanner callback function
    def laser_callback(self, data):

        # Convert ranges from meters to feet
        ranges = []
        for i in range(len(data.ranges)):
            ranges.append(self.support.meters_to_feet(data.ranges[i]))

        # Find minimum distance and index
        min_val, min_index = self.find_min_laser_data(ranges)

        #print(min_val)
        
        
        '''
        Object detected if minimum distance is less than threshold 
        or too close to read (nan)
        Assume (nan) values are below threshold, as:
        - (nan) values can either be less than range_min
        -             "              greater than range_max
        range_max is about 30ft. It is impossible to get reading of 30ft+ given our environment
        So the only other possible outcome is if distance read (nan) is below range_min, or threshold
        '''
        #if math.isnan(min_val) or min_val < LASER_AVOIDANCE_DISTANCE:
        if min_val < LASER_AVOIDANCE_DISTANCE:
            self.obstacle_detected = True
            print(self.obstacle_detected)
            
            self.laser_data = data
            self.laser_min_index = min_index
        # Object not detected
        else:
            self.obstacle_detected = False

class Camera():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

    def image_callback(self, img_msg):

        # Try to convert the ROS Image message to a CV2 Image
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Show the converted image
        self.show_image(cv_image)
        #self.filter_image(cv_image)

    def show_image(self, img):

        filtered = self.filter_image(img)

        cv2.imshow("Image Window", filtered)
        cv2.waitKey(3)
    
    def filter_image(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        lower_red = np.array([30,150,50])
        upper_red = np.array([255,255,180])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        median_blur = cv2.medianBlur(mask, 15)
        
        return median_blur



class Plan():
    def __init__(self):
        self.support = Support()
        self.graph = Graph()

        # list of all nodes (coord in meters!)
        self.nodes = {}
        # CS/ECE office
        self.nodes['Node1'] = [6, 3]
        self.nodes['Node2'] = [6, 8]
        # electronics lab
        self.nodes['Node3'] = [8.5, 8]
        # devon west atrium
        self.nodes['Node4'] = [11, 8]
        # devon west entrance
        self.nodes['Node5'] = [11, 1]
        # west sidewalks
        self.nodes['Node6'] = [18, 1]
        self.nodes['Node7'] = [18, 5]
        self.nodes['Node8'] = [24, 5]
        # devon north-east corner
        self.nodes['Node9'] = [6, 31]
        # computer lab
        self.nodes['Node10'] = [8.5, 31]
        # devon east atrium
        self.nodes['Node11'] = [11, 31]
        # devon east entrance
        self.nodes['Node12'] = [11, 38]
        # east sidewalks
        self.nodes['Node13'] = [18, 38]
        self.nodes['Node14'] = [18, 33]
        self.nodes['Node15'] = [24, 33]
        # REPF sidewalk
        self.nodes['Node16'] = [24, 19]
        # REPF entrance
        self.nodes['Node17'] = [28, 19]
        # REPF lobby
        self.nodes['Node18'] = [28, 22]
        self.nodes['Node19'] = [31.5, 22]
        # practice bay
        self.nodes['Node20'] = [31.5, 31]

        
        # list of important nodes (just the names)
        # TODO: change the names of the nodes to the corresponding location
        #self.imp_nodes = ['Devon_Atrium', 'Practice_Bay', 'Computer_Lab', 'Electronics_lab', 'CS/ECE_Office']
        self.imp_nodes = ['Node2', 'Node3', 'Node10', 'Node11', 'Node17', 'Node20']

        # list of nodes with a landmark
        self.imp_nodes_with_landmark = ['Node20']

        self.graph.connect('Node1', 'Node2', 5)
        self.graph.connect('Node2', 'Node3', 2.5)
        self.graph.connect('Node2', 'Node9', 23)
        self.graph.connect('Node3', 'Node4', 2.5)
        self.graph.connect('Node4', 'Node5', 6)
        self.graph.connect('Node4', 'Node11', 23)
        self.graph.connect('Node5', 'Node6', 7) # close enough
        self.graph.connect('Node6', 'Node7', 4)
        self.graph.connect('Node7', 'Node8', 6)
        self.graph.connect('Node7', 'Node14', 28)
        self.graph.connect('Node8', 'Node16', 16)
        self.graph.connect('Node16', 'Node17', 4)
        self.graph.connect('Node17', 'Node18', 3)
        self.graph.connect('Node18', 'Node19', 3.5)
        self.graph.connect('Node19', 'Node20', 10)
        self.graph.connect('Node9', 'Node10', 2.5)
        self.graph.connect('Node10', 'Node11', 2.5)
        self.graph.connect('Node11', 'Node12', 7)
        self.graph.connect('Node12', 'Node13', 7)
        self.graph.connect('Node13', 'Node14', 5)
        self.graph.connect('Node14', 'Node15', 6)
        self.graph.connect('Node15', 'Node16', 12)

        self.graph.make_undirected()
        
        self.astar = Astar(self.nodes, self.graph)
        
        self.plan = []
    
    # function for guidance from point A to point B
    # assuming the arguments are the names of the nodes
    def plan_route(self, first_node, second_node):
        global node_path
        global destination_node
        node_path = []
        #node_path.append(current_node)

        path, dist = self.astar.astar(current_node, first_node)
        for node in path:
            node_path.append(node)
            coord = Coord(self.support.meters_to_feet(self.nodes.get(node)[0]), self.support.meters_to_feet(self.nodes.get(node)[1]))
            self.plan.append(coord)
        # Remove duplicate nodes (first node)
        num = len(self.plan)

        path, dist = self.astar.astar(first_node, second_node)
        for node in path:
            node_path.append(node)
            coord = Coord(self.support.meters_to_feet(self.nodes.get(node)[0]), self.support.meters_to_feet(self.nodes.get(node)[1]))
            self.plan.append(coord)
        
        destination_node = second_node
        # Remove duplicate nodes (first node)
        self.plan.pop(num)
        node_path.pop(num)

        print('NODE PATH:')
        for node in node_path:
            print(node)
        
        # once done traverse the master path
        return self.plan
        
    def tour(self, first_node):
            
        path, dist = self.astar.astar(current_node, first_node)
        # add coordinates to the master plan
        for node in path:
            node_path.append(node)
            coord = Coord(self.support.meters_to_feet(self.nodes.get(node)[0]), self.support.meters_to_feet(self.nodes.get(node)[1]))
            self.plan.append(coord)
        
        path, dist = self.astar.find_tour_path(first_node, self.imp_nodes)
        for node in path:
            node_path.append(node)
            coord = Coord(self.support.meters_to_feet(self.nodes.get(node)[0]), self.support.meters_to_feet(self.nodes.get(node)[1]))
            self.plan.append(coord)
        
        
        return self.plan
            
class Support():

    def calculate_distance(self, coord1, coord2):
        dist = ((coord1.x - coord2.x) ** 2 + (coord1.y - coord2.y) ** 2) ** (0.5)
        return dist

    # Convert meters to feet
    def meters_to_feet(self, val):
        return val * 3.28

    # Convert feet to meters
    def feet_to_meters(self, val):
        return val / 3.28

class Navigation():

    def __init__(self, laser, plan):
        self.support = Support()
        self.laser = laser
        self.plan = plan

        self.waypoint_index = 0
        self.min_dist_to_dest = float('inf')

        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)

    #Return angle of rotation in degrees
    def get_rotation_angle(self, destination):
        global my_location
        global yaw

        delta_x = destination.x - my_location.x
        delta_y = destination.y - my_location.y

        angle = 0

        if delta_x == 0:
            # Target ahead or behind us
            if delta_y > 0:
                angle = 0
            elif delta_y < 0:
                angle = -180
        elif delta_y == 0:
            # Target to the left or right of us
            if delta_x > 0:
                angle = 90
            elif delta_x < 0:
                angle = -90
        else:
            angle = math.degrees(math.atan2(delta_x, delta_y))

        angle_to_turn = angle - yaw
        if angle_to_turn < -180:
            angle_to_turn += 360
        elif angle_to_turn > 180:
            angle_to_turn -= 360

        return angle_to_turn

    # Turns towards target pos
    def rotate_to_angle(self, waypoints):

        # Rotating
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        target_angle = self.get_rotation_angle(waypoints[self.waypoint_index])
        #print('Target angle: ' + str(target_angle))

        # Determine the direction to turn
        # Flip turn angle: positive turns right, negative turns left
        turn_msg = Twist()
        if target_angle >= 0:
            turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT
        else:
            turn_msg.angular.z = ANGULAR_SPEED_DEFAULT

        while current_angle < abs(target_angle):  
            if self.laser.obstacle_detected:
                return  
            self.velocity_pub.publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = math.degrees(ANGULAR_SPEED_DEFAULT) * (t1 - t0)  

    def avoid(self):

        turn_msg = Twist()

        # Turn in the direction away from the closest obstacle
        if self.laser.laser_min_index < (len(self.laser.laser_data.ranges) - 1) / 2:
            turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
        else:
            turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT
        
        while self.laser.obstacle_detected:            
            self.velocity_pub.publish(turn_msg)

    # Move in increments and turn
    def move_dist(self, waypoints):  # BOTH IN FT
        dist_diff = self.support.calculate_distance(my_location, waypoints[self.waypoint_index])
        
        if dist_diff < self.min_dist_to_dest:
            self.min_dist_to_dest = dist_diff
        
        while dist_diff > dist_diff_threshold:
            
            if self.laser.obstacle_detected:
                # Avoid obstacle if detected
                #print('Avoiding...')
                #self.avoid()
                turn_msg = Twist()

                # Turn in the direction away from the closest obstacle
                if self.laser.laser_min_index < (len(self.laser.laser_data.ranges) - 1) / 2:
                    turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
                else:
                    turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT
                
                while self.laser.obstacle_detected:            
                    self.velocity_pub.publish(turn_msg)
            else:
                # Rotate towards destination
                self.rotate_to_angle(waypoints)
            

            #self.rotate_to_angle(waypoints)
            
            rospy.sleep(1)

            speed = LINEAR_SPEED_DEFAULT
            if dist_diff > 2:
                speed = linear_speed_fast

            if not self.laser.obstacle_detected:
                # Move forward
                vel_msg = Twist()
                vel_msg.linear.x = self.support.feet_to_meters(speed)
                self.velocity_pub.publish(vel_msg)
                rospy.sleep(0.5)
            
            # Caluclate remaining distance
            dist_diff = self.support.calculate_distance(my_location, waypoints[self.waypoint_index])
            
            if dist_diff < self.min_dist_to_dest:
                self.min_dist_to_dest = dist_diff

            
            if dist_diff - self.min_dist_to_dest > NAV_FAILURE_DISTANCE_THRESHOLD:
                print('Failure to reach ' + str(waypoints[self.waypoint_index].to_string()))
                if self.waypoint_index % 2 == 0:
                    # waypoint was start
                    self.waypoint_index += 1
                break

    def navigate(self, waypoints):
        global my_location
        global current_node
        global node_path

        print('node path:')
        print(node_path)
        print('waypoit length')
        print(len(waypoints))

        # Assuming robot faces forward (+y direction) at 0,0 initially
        while self.waypoint_index < len(waypoints):
            
            print('Heading to ' + waypoints[self.waypoint_index].to_string())
            #print('From ' + my_location.to_string())
            rospy.sleep(1)
            
            self.move_dist(waypoints)
            # Do checks for walls in move_small too?
            # Call rotate_to_angle in move small as well?

            #print(my_location.to_string())
            print('Arrived at: ' + waypoints[self.waypoint_index].to_string())
            #current_node = destination_node

            print('current node: ' + current_node)
            current_node = node_path[self.waypoint_index]
            self.waypoint_index += 1
            self.min_dist_to_dest = float('inf')
            

            rospy.sleep(1)
        
        print('Finished')
        print('current node: ' + current_node)
        node_path = []
    
    # def look_for_landmark(self, current_node):
    #     if current_node in self.plan.imp_nodes_with_landmark:


def choice_callback(data):
    busy_bool.data = True
    busy_pub.publish(busy_bool)

    odom = Odom()
    laser = Laser()
    camera = Camera()

    planner = Plan()
    navigator = Navigation(laser, planner)


    waypoints = planner.plan_route('Node8', 'Node20')
    #waypoints = planner.tour('Node3')
    for i in range(len(waypoints)):
        print(waypoints[i].to_string())
    navigator.navigate(waypoints)

    busy_bool.data = False
    busy_pub.publish(busy_bool)

def init_control_node():
    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    global busy_bool 
    busy_bool = Bool()
    busy_bool.data = False


    # First time
    global busy_pub
    choice_sub = rospy.Subscriber('/robot/choice', Int32, choice_callback)
    busy_pub = rospy.Publisher('/robot/busy_bool', Bool, queue_size=10)
    while not busy_bool.data:
        busy_pub.publish(busy_bool)
        #print "Published busy bool"

    '''
    points = [[Coord(2, 3), Coord(4, 5)],
              [Coord(2, 6), Coord(1, 1)]]
    '''
    rospy.spin()

if __name__ == '__main__':
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass