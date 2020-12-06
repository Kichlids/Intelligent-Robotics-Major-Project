#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from robot_msgs.msg import coordinate
from robot_msgs.msg import tasks

from tf.transformations import euler_from_quaternion

# Speed ft/s
LINEAR_SPEED_DEFAULT = 0.5
# Rotation speed rad ft/s 
ANGULAR_SPEED_DEFAULT = 0.1

# Obstacle avoidance threshold in ft, including the position of the laser scan sensor
LASER_AVOIDANCE_DISTANCE = 2.3

'''
If robot moves away this much distance (ft),
abandon this waypoint (and its destination if applicable)
and move on to next waypoint
'''
NAV_FAILURE_DISTANCE_THRESHOLD = 2



class Coord():

    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def to_string(self):
        return str(self.x) + ', ' + str(self.y)


my_location = Coord(0, 0)
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

        y = self.support.meters_to_feet(round(data.pose.pose.position.x, 2))
        x = self.support.meters_to_feet(round(-data.pose.pose.position.y, 2))

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
        if math.isnan(min_val) or min_val < LASER_AVOIDANCE_DISTANCE:
            self.obstacle_detected = True
            
            self.laser_data = data
            self.laser_min_index = min_index
        # Object not detected
        else:
            self.obstacle_detected = False

class Plan():

    def __init__(self):
        self.support = Support()

    def plan_route(self, list_set):
        global my_location

        coord_pairs = list_set

        waypoints = []
        
        start = my_location        

        min_dist = float('inf')
        min_index = -1
        for i in range(len(coord_pairs)):

            dist = self.support.calculate_distance(start, coord_pairs[i][0])
            if dist < min_dist:
                min_dist = dist
                min_index = i

        waypoints.append(coord_pairs[min_index][0])
        waypoints.append(coord_pairs[min_index][1])

        start = coord_pairs[min_index][1]
        coord_pairs.pop(min_index)

        while len(coord_pairs) > 0:

            min_dist = float('inf')
            min_index = -1

            for i in range(len(coord_pairs)):
                dist = self.support.calculate_distance(start, coord_pairs[i][0])
                if dist < min_dist:
                    min_dist = dist
                    min_index = i
            
            waypoints.append(coord_pairs[min_index][0])
            waypoints.append(coord_pairs[min_index][1])
            
            start = coord_pairs[min_index][1]
            coord_pairs.pop(min_index)

        for i in range(len(waypoints)):
            waypoints[i].to_string()

        return waypoints

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

    def __init__(self, laser):
        self.support = Support()
        self.laser = laser

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
                angle = 180
        elif delta_y == 0:
            # Target to the left or right of us
            if delta_x > 0:
                angle = 90
            elif delta_x < 0:
                angle = -90
        else:
            angle = math.degrees(math.atan2(delta_x, delta_y))
        
        return angle - yaw

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
    def move_dist(self, waypoints):
        dist_diff = self.support.calculate_distance(my_location, waypoints[self.waypoint_index])
        
        if dist_diff < self.min_dist_to_dest:
            self.min_dist_to_dest = dist_diff
        
        while dist_diff > 1:
            
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

            if not self.laser.obstacle_detected:
                # Move forward
                vel_msg = Twist()
                vel_msg.linear.x = self.support.feet_to_meters(LINEAR_SPEED_DEFAULT)
                self.velocity_pub.publish(vel_msg)
                rospy.sleep(0.5)
            
            # Caluclate remaining distance
            dist_diff = self.support.calculate_distance(my_location, waypoints[self.waypoint_index])
            
            if dist_diff < self.min_dist_to_dest:
                self.min_dist_to_dest = dist_diff
            '''
            print('dist_diff ' + str(dist_diff))
            print('min_diff ' + str(self.min_dist_to_dest))
            print('T_diff ' + str(dist_diff - self.min_dist_to_dest))
            '''

            
            if dist_diff - self.min_dist_to_dest > NAV_FAILURE_DISTANCE_THRESHOLD:
                print('Failure to reach ' + str(waypoints[self.waypoint_index].to_string()))
                if self.waypoint_index % 2 == 0:
                    # waypoint was start
                    self.waypoint_index += 1
                break

    def navigate(self, waypoints):
        global my_location

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

            self.waypoint_index += 1
            self.min_dist_to_dest = float('inf')
            rospy.sleep(1)
        
        print('Finished')
        #print(my_location.to_string())

def tasks_callback(data):
    busy_bool.data = True
    busy_pub.publish(busy_bool)

    odom = Odom()
    laser = Laser()

    planner = Plan()
    navigator = Navigation(laser)

    points = []

    for i in range(0, len(data.coord_list), 2):
        print "i: " + str(i) + ", length: " + str(len(data.coord_list))

        temp = []

        start = Coord(data.coord_list[i].x_coord, data.coord_list[i].y_coord)
        end = Coord(data.coord_list[i + 1].x_coord, data.coord_list[i + 1].y_coord)

        temp.append(start)
        temp.append(end)

        points.append(temp)

    waypoints = planner.plan_route(points)

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
    tasks_sub = rospy.Subscriber('/robot/tasks', tasks, tasks_callback)
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