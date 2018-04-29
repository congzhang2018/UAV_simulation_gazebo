#!/usr/bin/env python

import rospy
# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist      # for sending commands to the drone
import time
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState

velocity = 1

class Moving_controller:

    # Velocity = 1

    def __init__(self):

        self.pubCommand = rospy.Publisher('/robot2/cmd_vel',Twist,queue_size=10)
        self.command = Twist()
        self.target_orientation_x = 0
        self.target_orientation_y = 0
        self.target_orientation_z = 0
        self.target_orientation_w = 0
        
        self.target_position_x = 0
        self.target_position_y = 0
        self.target_position_z = 0

        self.circle1 = True

        self.yaw_radians = 0
        self.yaw = 0
        # self.circle_path = np.loadtxt("/home/cong/catkin_ws/src/ar2landing_neural/src/circle.txt", dtype = 'float', delimiter = ',')
        # self.zigzag_path = np.loadtxt("/home/cong/catkin_ws/src/ar2landing_neural/src/zigzag.txt", dtype = 'float', delimiter = ',')
        # self.figure_eight_path = np.loadtxt("/home/cong/catkin_ws/src/ar2landing_neural/src/figure_eight.txt", dtype = 'float', delimiter = ',')
        self.counter = 0

    def SetCommand(self, x =0, y =0, z =0, yaw_velocity=0):
        # Called by the main program to set the current command
        
        self.command.linear.x  = x
        self.command.linear.y  = y
        self.command.linear.z  = z
        self.command.angular.z = yaw_velocity
        self.pubCommand.publish(self.command)


    def show_gazebo_models(self):

        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates_robot1 = model_coordinates('robot1', '')
            resp_coordinates_quadrotor = model_coordinates('quadrotor', '')

            self.target_position_x = resp_coordinates_robot1.pose.position.x
            self.target_position_y = resp_coordinates_robot1.pose.position.y
            self.target_position_z = resp_coordinates_robot1.pose.position.z
            
            self.target_orientation_x = resp_coordinates_robot1.pose.orientation.x
            self.target_orientation_y = resp_coordinates_robot1.pose.orientation.y
            self.target_orientation_z = resp_coordinates_robot1.pose.orientation.z
            self.target_orientation_w = resp_coordinates_robot1.pose.orientation.w
            
            orientation_list = [self.target_orientation_x, self.target_orientation_y, self.target_orientation_z, self.target_orientation_w]
            (roll, pitch, self.yaw_radians) = euler_from_quaternion (orientation_list)

            self.yaw = math.degrees(self.yaw_radians) + 180

            print self.yaw 

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


    def straight_line(self):
        self.SetCommand(velocity, 0, 0, 0)

    def circle(self):
        self.SetCommand(velocity, 0, 0, 0.05)

        # if self.counter < len(self.circle_path):
        #     # print "yes"
            
        #     distance =  math.sqrt((self.circle_path[self.counter , 0]-self.target_position_x)**2 + (self.circle_path[self.counter, 1]-self.target_position_y)**2)
        #     print "location", self.target_position_x, self.circle_path[self.counter, 0], self.target_position_y, self.circle_path[self.counter, 1]
        #     if distance < 0.1:
        #         self.counter += 1
        #     else:
        #         angle = math.atan2(self.circle_path[self.counter, 0] - self.target_position_x, self.circle_path[self.counter, 1]-self.target_position_y)
        #         control_angle = -angle - self. yaw_radians + math.pi/2
        #         self.SetCommand(2, 0, 0, control_angle) 
        #         print "counter:", self.counter, control_angle, distance, angle, self.yaw_radians
        # else:
        #     self.counter = 0
        #     print "Finised path simulation"
        #     print "counter:", self.counter, control_angle

    def zigzag(self):

        # if self.counter < len(self.zigzag_path):
        #     # print "yes"
        #     distance =  math.sqrt((self.zigzag_path[self.counter , 0]-self.target_position_x)**2 + (self.zigzag_path[self.counter, 1]-self.target_position_y)**2)
        #     print "location", self.target_position_x, self.zigzag_path[self.counter, 0], self.target_position_y, self.zigzag_path[self.counter, 1]
        #     if distance < 0.1:
        #         self.counter += 1
        #     else:
        #         angle = math.atan2(self.zigzag_path[self.counter, 0] - self.target_position_x, self.zigzag_path[self.counter, 1]-self.target_position_y)
        #         control_angle = -angle - self. yaw_radians + math.pi/2
        #         self.SetCommand(2, 0, 0, control_angle) 
        #         print "counter:", self.counter, control_angle, distance, angle, self.yaw_radians
        # else:
        #     self.counter = 0
        #     print "Finised path simulation"
        #     print "counter:", self.counter, control_angle

        self.SetCommand(velocity, 0, 0, 0)
        time.sleep(10)
        self.SetCommand(0.5, 0, 0, 0.783)
        time.sleep(2)
        self.SetCommand(velocity, 0, 0, 0)
        time.sleep(10)
        self.SetCommand(0.5, 0, 0, -0.783)
        time.sleep(2)

    def figure_eight(self):

        if self.circle1 == True:
            self.SetCommand(velocity, 0, 0, 0.25)
            if int(self.yaw) == 180:
                self.circle1 = False
        else:
            self.SetCommand(velocity, 0, 0, -0.25)
            if int(self.yaw) == 180:
                self.circle1 = True
    
    def arrow_heading_path(self):
        self.SetCommand(5, 0, 0, 5)


    def m_path(self):
        self.SetCommand(0.5, 0, 0, 0.2)
        time.sleep(16)
        self.SetCommand(0.5, 0, 0, -3.14)
        time.sleep(1.2)      


    # def TimerCallback(self, event):
    #     print "*****************************************************"
    #     self.show_gazebo_models()
    #     if self.start_flying:
    #         self.PN_controller()

def main():

    rospy.init_node('Moving_target', anonymous=True)
    MC = Moving_controller()
    case = raw_input("\
    switch the path: 1.straight_line \n \
                     2.circle \n \
                     3.zigzag \n \
                     4.figure_eight \n \
                     5.m_path \n "
                        )
    
    while not rospy.is_shutdown():

        MC.show_gazebo_models()

        if case == '1':
            MC.straight_line()
        elif case == '2':
            MC.circle()
        elif case == '3':
            MC.zigzag()
        elif case == "4":
            MC.figure_eight()
        elif case == '5':
            MC.m_path()
        else:
            MC.SetCommand(0, 0, 0, 0)

if __name__ == '__main__':
    
    main()
