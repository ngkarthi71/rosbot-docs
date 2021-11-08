#!/usr/bin/env python

import rospy
import actionlib
import tf
import numpy as np
import roslaunch

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from find_object_2d.msg import ObjectsStamped

import os
import traceback

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

# boolean value to know when the exploration starts
exploration_started = False

# string prefix to add before object ids
objPrefix_='object'


"""
    launch_func : To launch the explore.launch file to start the exploration

    parametrs:
        t_start : starting time of launch
"""
def launch_func(t_start):
    global duration
    t_now = rospy.Time.now()
    os.system("roslaunch ros_challenge explore.launch")


"""
    callback: Getting the callback once the rosbot sees the marker

    parametrs:
        data : id, orientation and frame of the marker seen, inside an array
"""
def callback(data):
    global exploration_started
    t_start = rospy.Time.now()
    objects = data.objects.data
    print("Data: :", objects)
    if len(objects) > 0:
        print("Hazards", objects[0], type(data))
        # id of the start marker is 30
        if objects[0] == 30 and exploration_started == False:
            exploration_started  = True
            print("start Launch")
            launch_func(t_start)


"""
    main : Setting up the node and starting the exploration
"""
if __name__ == '__main__':
    try:
        rospy.init_node('nav_node', anonymous=True)
        global objects

        # subsribing to topic , to get the ids of the markers
        rospy.Subscriber('/objectsStamped', ObjectsStamped, callback)
        objPrefix_ = rospy.get_param('~object_prefix', objPrefix_)
        rospy.sleep(300)

    except rospy.ROSInterruptException:
        pass
