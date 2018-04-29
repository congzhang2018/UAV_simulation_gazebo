#!/usr/bin/env python
from __future__ import division
import roslib
# ardrone_tutorials has a good ardrone classs for controlling drone. load_manifest import that python module
roslib.load_manifest("ardrone_control")
import sys
import logging
from io import FileIO, BufferedWriter
import rospy
from drone_controller import BasicDroneController
import time
import numpy as np
import math
# import control
import scipy.linalg
# from simple_PID import *

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose, PointStamped
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Range
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# record message and error information
LoggerWarningLevel = logging.DEBUG
# LoggerWarningLevel = logging.INFO
# LoggerWarningLevel = logging.WARNING
# LoggerWarningLevel = logging.ERROR

EnableLanding = False

TakeOffHight = 2
dt = 0.1
q = 0.85
q_land = 0.9

class LQR_sim:


    def __init__(self):


        self.start_flying = True
        self.stop_flying = False
        self.return_to_home = False
        self.is_takeoff = False
        # self.PID = SimplePID()

        self.drone_position_x = 0
        self.drone_position_y = 0
        self.drone_position_z = 0

        self.drone_velocity_x = 0
        self.drone_velocity_y = 0
        self.drone_velocity_z = 0

        self.drone_acceleration_x = 0
        self.drone_acceleration_y = 0
        self.drone_acceleration_z = 0

        self.target_position_x = 0
        self.target_position_y = 0
        self.target_position_z = 0

        self.target_velocity_x = 0
        self.target_velocity_y = 0
        self.target_velocity_z = 0

        self.drone_yaw = 0
        self.drone_yaw_radians = 0

        self.vx = 0
        self.vy = 0
        self.vx1 = 0
        self.vy1 = 0

        self.ax = 0
        self.ay = 0

        self.controller = BasicDroneController()
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
        
        self.logger = logging.getLogger('LQR_simulation')
        self.fileHandler_message = logging.StreamHandler(BufferedWriter(FileIO("LQR_simulation_data" + time.strftime("%Y%m%d-%H%M%S") + ".log", "w")))
        self.logger.addHandler(self.fileHandler_message)
        self.formatter_message = logging.Formatter('%(message)s')
        self.fileHandler_message.setFormatter(self.formatter_message)
        self.logger.setLevel(LoggerWarningLevel)
        self.logger.info('Time;target_position_x,target_position_y,target_position_z;target_velocity_x,target_velocity_y,target_velocity_z;drone_position_x,drone_position_y,drone_position_z;drone_velocity_x,drone_velocity_y,drone_velocity_z,vx1,vy1,ax,ay')

        self.logger_land = logging.getLogger('LQR_simulation_land')
        self.fileHandler_message = logging.StreamHandler(BufferedWriter(FileIO("LQR_simulation_PD_land_data" + time.strftime("%Y%m%d-%H%M%S") + ".log", "w")))
        self.logger_land.addHandler(self.fileHandler_message)
        self.formatter_message = logging.Formatter('%(message)s')
        self.fileHandler_message.setFormatter(self.formatter_message)
        self.logger_land.setLevel(LoggerWarningLevel)
        self.logger_land.info('Time;target_position_x,target_position_y,target_position_z;target_velocity_x,target_velocity_y,target_velocity_z;drone_position_x,drone_position_y,drone_position_z;drone_velocity_x,drone_velocity_y,drone_velocity_z,vx1,vy1,ax,ay')

    def show_gazebo_models(self):

        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates_robot1 = model_coordinates('robot1', '')
            resp_coordinates_quadrotor = model_coordinates('quadrotor', '')
            
            self.target_position_x = resp_coordinates_robot1.pose.position.x
            self.target_position_y = resp_coordinates_robot1.pose.position.y
            self.target_position_z = resp_coordinates_robot1.pose.position.z

            self.target_velocity_x = resp_coordinates_robot1.twist.linear.x
            self.target_velocity_y = resp_coordinates_robot1.twist.linear.y
            self.target_velocity_z = resp_coordinates_robot1.twist.linear.z

            self.drone_position_x = resp_coordinates_quadrotor.pose.position.x
            self.drone_position_y = resp_coordinates_quadrotor.pose.position.y
            self.drone_position_z = resp_coordinates_quadrotor.pose.position.z

            self.drone_velocity_x = resp_coordinates_quadrotor.twist.linear.x
            self.drone_velocity_y = resp_coordinates_quadrotor.twist.linear.y
            self.drone_velocity_z = resp_coordinates_quadrotor.twist.linear.z
            

            distance = math.sqrt((self.drone_position_x-self.target_position_x)**2 + (self.drone_position_y-self.target_position_y)**2)
            print "distance:",distance
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


    def sleep(self, sec):
        while(sec and not rospy.is_shutdown()):
            if(detect==1):
                break
            time.sleep(1)
            sec -= 1

    def reset_drone(self):
        self.controller.SetCommand(0,0,0,0)
        
    def fight_control(self, roll, pitch, z_velocity, yaw_velocity):
        self.controller.SetCommand(roll, pitch, z_velocity, yaw_velocity)

    def yaw_control(delf, degrees):

        if degrees > 0 and degrees < 180:
            self.controller.SetCommand(0,0,0,-0.5)
        else:
            self.controller.SetCommand(0,0,0, 0.5)

    def take_off(self, hight):
        if hight > self.drone_position_z:
            self.controller.SetCommand(0,0,1,0)
            # print "go up, current hight:%f, desired hight:%f" % (self.drone_position_z, hight)
        else:
            self.is_takeoff = True
            # print "take off successful"
    
    def dlqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
     
     
        x[k+1] = A x[k] + B u[k]
         
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        """
        #ref Bertsekas, p.151
     
        #first, try to solve the ricatti equation
        X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
         
        #compute the LQR gain
        K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
         
        eigVals, eigVecs = scipy.linalg.eig(A-B*K)
         
        return K, X, eigVals

    def land(self):

        self.record_land_data()

        u = [   self.target_position_x - self.drone_position_x, 
                self.target_position_y - self.drone_position_y,
                0                                       ]

        u_dot = [   self.target_velocity_x - self.drone_velocity_x, 
                    self.target_velocity_y - self.drone_velocity_y,
                    0                                       ]

        

        A = np.array([[1, 0, dt, 0], \
                      [0, 1, 0, dt], \
                      [0, 0, 1, 0 ], \
                      [0, 0, 0, 1 ]])

        B = np.array([[dt**2/2, 0], \
                      [0, dt**2/2], \
                      [dt, 0], \
                      [0, dt]])

        
        Q = q_land * np.eye(4)
        R = (1-q_land) * np.array([[100, 0],\
                                   [0, 100]])


        state_error = - np.array([[u[0]], [u[1]], [u_dot[0]], [u_dot[1]]])
        K, S, E = self.dlqr(A, B, Q, R)
        a = np.dot(-K, state_error)

        self.ax = a[0]
        self.ay = a[1]
        
        # ax = np.clip(ax, -5, 5)
        # ay = np.clip(ay, -5, 5)

        vx = self.drone_velocity_x + self.ax*0.1
        vy = self.drone_velocity_y + self.ay*0.1
        self.vx1 = math.cos(self.drone_yaw_radians)*vx + math.sin(self.drone_yaw_radians)*vy
        self.vy1 = -math.sin(self.drone_yaw_radians)*vx + math.cos(self.drone_yaw_radians)*vy

        self.fight_control(self.vx1, self.vy1, -0.5, 0)
        # self.yaw_control(180)

        if self.drone_position_z < 0.2:
            self.controller.SendLand()
        else:
            return
      

    def LQR_controller(self): 

        u = [   self.target_position_x - self.drone_position_x, 
                self.target_position_y - self.drone_position_y,
                0                                       ]

        u_dot = [   self.target_velocity_x - self.drone_velocity_x, 
                    self.target_velocity_y - self.drone_velocity_y,
                    0                                       ]



        A = np.array([[1, 0, dt, 0], \
                      [0, 1, 0, dt], \
                      [0, 0, 1, 0 ], \
                      [0, 0, 0, 1 ]])

        B = np.array([[dt**2/2, 0], \
                      [0, dt**2/2], \
                      [dt, 0], \
                      [0, dt]])

        
        Q = q * np.array([[1, 0, 0, 0], \
                          [0, 1, 0, 0], \
                          [0, 0, 10, 0 ], \
                          [0, 0, 0, 10 ]])

        R = (1-q) * np.array([[1, 0],\
                              [0, 1]])


        state_error = - np.array([[u[0]], [u[1]], [u_dot[0]], [u_dot[1]]])
        K, S, E = self.dlqr(A, B, Q, R)
        a = np.dot(-K, state_error)

        self.ax = a[0]
        self.ay = a[1]
        
        # self.ax = np.clip(self.ax, -3, 3)
        # self.ay = np.clip(self.ay, -3, 3)

        self.vx = self.drone_velocity_x + self.ax*0.1
        self.vy = self.drone_velocity_y + self.ay*0.1
        self.vx1 = math.cos(self.drone_yaw_radians)*self.vx + math.sin(self.drone_yaw_radians)*self.vy
        self.vy1 = -math.sin(self.drone_yaw_radians)*self.vx + math.cos(self.drone_yaw_radians)*self.vy
        
        # self.vx1 = np.clip(self.vx1, -5, 5)
        # self.vy1 = np.clip(self.vy1, -5, 5)

        # self.fight_control(self.vx1, -self.vy1, 0, 0)
        if self.drone_position_z < 0.5:
            self.fight_control(self.vx1, self.vy1, 0, 0)
        else:
            self.fight_control(self.vx1, self.vy1, 0, 0)

        # desired_angle = 0

        # input_yaw = desired_angle - self.drone_yaw
        # self.yaw_control(input_yaw)

    def record_data(self):

        self.logger.info('%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f' % \
            (   time.time(),

                self.target_position_x,
                self.target_position_y, 
                self.target_position_z,

                self.target_velocity_x, 
                self.target_velocity_y, 
                self.target_velocity_z, 

                self.drone_position_x, 
                self.drone_position_y, 
                self.drone_position_z, 

                self.drone_velocity_x, 
                self.drone_velocity_y, 
                self.drone_velocity_z,

                self.vx1,
                self.vy1,

                self.ax,
                self.ay,

                self.vx,
                self.vy,

                self.drone_yaw,

                self.drone_acceleration_x, 
                self.drone_acceleration_y, 
                self.drone_acceleration_z

            )) 

    def record_land_data(self):

        self.logger_land.info('%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f' % \
            (   time.time(),

                self.target_position_x,
                self.target_position_y, 
                self.target_position_z,

                self.target_velocity_x, 
                self.target_velocity_y, 
                self.target_velocity_z, 

                self.drone_position_x, 
                self.drone_position_y, 
                self.drone_position_z, 

                self.drone_velocity_x, 
                self.drone_velocity_y, 
                self.drone_velocity_z,

                self.vx1,
                self.vy1,

                self.ax,
                self.ay,

                self.vx,
                self.vy,

                self.drone_yaw,  

                self.drone_acceleration_x, 
                self.drone_acceleration_y, 
                self.drone_acceleration_z
            )) 

    def CallbackUserCommand(self, data):

        self.start_flying =   data.point.x == 1
        self.stop_flying =    data.point.y == 1
        self.return_to_home = data.point.z == 1

    def CallbackAcceleartion(self, data):

        self.drone_acceleration_x = data.linear_acceleration.x
        self.drone_acceleration_y = data.linear_acceleration.y
        self.drone_acceleration_z = data.linear_acceleration.z


    def ReceiveNavdata(self, data):
        self.drone_yaw = data.rotZ
        self.drone_yaw_radians = math.radians(self.drone_yaw)
        print "yaw angle", self.drone_yaw , self.drone_yaw_radians

    def TimerCallback(self, event):
        self.show_gazebo_models()
        if self.is_takeoff == False:
            self.take_off(TakeOffHight)
        else:
            distance_2D = math.sqrt((self.drone_position_x - self.target_position_x)**2 + (self.drone_position_y - self.target_position_y)**2)
            if distance_2D < 0.2 and EnableLanding:
                self.land()
            else:
                self.LQR_controller()
        self.record_data()
       

def main():
 
    rospy.init_node('LQR_simulation_test', anonymous=True)
    LQR = LQR_sim()

    # Setup subscriber
    rospy.Subscriber("/user/command_data", PointStamped, LQR.CallbackUserCommand, queue_size=2)
    rospy.Subscriber('/ardrone/navdata',Navdata, LQR.ReceiveNavdata) 
    rospy.Subscriber("/ardrone/imu", Imu, LQR.CallbackAcceleartion, queue_size=2)
    LQR.controller.StartSendCommand()
    time.sleep(1)
    LQR.controller.SendTakeoff()
    LQR.reset_drone()
    dTimeStep = 0.1
    rospy.Timer(rospy.Duration(dTimeStep), LQR.TimerCallback)

    rospy.spin()


if __name__ == '__main__':
    
    main()
