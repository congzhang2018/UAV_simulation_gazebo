#!/usr/bin/env python
#  *  coding: UTF 8  * 

import time
import math

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped

import User
import logging
import sys
from io import FileIO, BufferedWriter


LoggerWarningLevel = logging.DEBUG
# LoggerWarningLevel = logging.INFO
# LoggerWarningLevel = logging.WARNING
# LoggerWarningLevel = logging.ERROR


logger = logging.getLogger('User_message')
fileHandler_message = logging.StreamHandler(BufferedWriter(FileIO("User_message" + time.strftime("%Y%m%d-%H%M%S") + ".log", "w")))
logger.addHandler(fileHandler_message)
formatter_message = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
fileHandler_message.setFormatter(formatter_message)
logger.setLevel(LoggerWarningLevel)


def sigint_handler():

    logger.warning("shutdown!")

 

def display_user_menu():

    print("\n")

    print("+--------------------------------------------------------------+")
    print("| [s] start filght test                                        |")
    print("| [f] stop all flight command                                  |")
    print("| [r] return to home                                           |")
    print("| [99] Exit                                                    |")
    print("|Note: return to home enable only after stop all flight command|")
    print("+--------------------------------------------------------------+")

def publish_command_data():
    # Start to publish data
    msgCommandData = PointStamped()
    msgCommandData.header.stamp = rospy.Time.now()
    msgCommandData.point.x = user.start
    msgCommandData.point.y = user.stop
    msgCommandData.point.z = user.return_to_home
    CommandDataPub.publish(msgCommandData)
    logger.info('send command data')

def clear():
    user.start = 0
    user.stop = 0
    user.return_to_home = 0
    logger.info('reset command data')


if __name__ == '__main__':

    # Initial the node 
    rospy.init_node('user_node', anonymous=True)
    rospy.on_shutdown(sigint_handler)

    user = User.User()

    # setup publisher
    CommandDataPub = rospy.Publisher("/user/command_data", PointStamped, queue_size=10)


    while not rospy.is_shutdown():

        display_user_menu()

        inputKey = raw_input("input a key then press enter:")
        
        if inputKey == 's':
            user.start = 1
            logger.warning('start flying')

        elif inputKey == 'f':
            user.stop = 1
            logger.warning('stop flying')

        elif inputKey == 'r':
            logger.warning('return to home')
            user.return_to_home = 1
            
        elif inputKey ==  '99':
            logger.warning("exiting!")
            sigint_handler()
            break
        
        else:
            logger.warning("there is no such key!")
            continue


        publish_command_data()
        clear()

    # deal with some left-behinds
    # After spin() no code will be excuted.
    rospy.spin()

