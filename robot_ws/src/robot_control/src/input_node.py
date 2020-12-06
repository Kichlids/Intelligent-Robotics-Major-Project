#! /usr/bin/env python
import rospy
import re

from robot_msgs.msg import coordinate
from robot_msgs.msg import tasks
from std_msgs.msg import Bool

# Class for reading input coordinates
class InputReader:

    # Create publisher for publishing tasks (coordinates) on the topic
    # Create subscriber for subscribing to bool from control node
    def __init__(self):
        self.tasks_pub = rospy.Publisher('/robot/tasks', tasks, queue_size = 10)
        self.busy_sub = rospy.Subscriber('/robot/busy_bool', Bool, self.input_callback)
    
    # Callback function for actually getting coordinate input
    # Only called when control node is not busy
    def input_callback(self, data):
        myTasks = tasks()

        if data.data == False:
            rospy.sleep(1)
            print "Input coordinates:"
            string_input = raw_input()
            while string_input != "":
                
                temp = re.findall(r'\d+', string_input)
                nums = list(map(float, temp)) 

                if len(nums) == 4:
                    start_coord = coordinate()
                    end_coord = coordinate()
                    start_coord.x_coord = nums[0]
                    start_coord.y_coord = nums[1]
                    end_coord.x_coord = nums[2]
                    end_coord.y_coord = nums[3]

                    myTasks.coord_list.append(start_coord)
                    myTasks.coord_list.append(end_coord)
                else:
                    print "Input not valid as (<start_coordinates>, <end_coordinates>)"
                string_input = raw_input()

            self.tasks_pub.publish(myTasks)
            rospy.sleep(5)

# Initializes our node as 'input_node'
def init_input_node():
    rospy.init_node('input_node', anonymous = False)
    rate = rospy.Rate(10)

    reader = InputReader()

    rospy.spin()
# Main func, just calls node initialization
if __name__ == '__main__':
    try:
        init_input_node()
    except rospy.ROSInterruptException:
        pass
