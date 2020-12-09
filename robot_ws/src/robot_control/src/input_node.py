#! /usr/bin/env python
import rospy

#from robot_msgs.msg import coordinate
#from robot_msgs.msg import tasks
from std_msgs.msg import Bool
from std_msgs.msg import Int32

# Class for reading input coordinates
class InputReader:

    # Create publisher for publishing tasks (coordinates) on the topic
    # Create subscriber for subscribing to bool from control node
    def __init__(self):
        #self.tasks_pub = rospy.Publisher('/robot/tasks', tasks, queue_size = 10)
        self.choice_pub = rospy.Publisher('/robot/choice', Int32, queue_size = 10)
        self.tour_pub = rospy.Publisher('/robot/tour', Int32, queue_size = 10)
        self.dest_pub = rospy.Publisher('/robot/destination', Int32, queue_size = 10)
        self.busy_sub = rospy.Subscriber('/robot/busy_bool', Bool, self.input_callback)
    
    # Callback function for actually getting coordinate input
    # Only called when control node is not busy
    def input_callback(self, data):
        #myTasks = tasks()

        myChoice = Int32()
        tour = Int32()
        dest = Int32()

        if data.data == False:
            rospy.sleep(1)
            print('To start the service enter an option number to choose a location to start at and press Enter \n'
                    + '1) Start at the West entrance to Devon \n' 
                    + '2) Start at the East entrance to Devon \n')

            string_input = raw_input()
            while string_input != '':
                
                if string_input == '1':
                    myChoice.data = 1
                elif string_input == '2':
                    myChoice.data = 2

            self.choice_pub.publish(myChoice)
            
            print "Select which service you would like to receive \n" 
                    + "1) Tour \n" 
                    + "2) Destination guidance \n"
            
            string_input = raw_input()
            while string_input != "":
                
                if string_input == "1":
                    tour.data = 1
                elif string_input == "2":
                    tour.data = 0
                    print "Select which destination you would like to go to \n"
                         + "1) Rawl Engineering Facility \n" + 
                         + "2) Rawl Practice Bay \n" 
                         + "3) Devon Energy Hall \n" 
                         + "4) Devon Computer Lab \n" 
                         + "5) Devon Electrical Lab \n"
                    
                    string_input = raw_input()
                    while string_input != '':
                        if string_input == '1':
                            dest.data = 1
                        elif string_input == '2': 
                            dest.data = 2
                        elif string_input == '3': 
                            dest.data = 3
                        elif string_input == '4': 
                            dest.data = 4
                        elif string_input == '5': 
                            dest.data = 5
            
            self.tour_pub.publish(tour)
            self.dest_pub.publish(dest)
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
