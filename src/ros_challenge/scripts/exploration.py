#!/usr/bin/env python
import roslaunch

import rospy
import tf
import numpy as np

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from find_object_2d.msg import ObjectsStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import actionlib

import threading
import os


"""
    global variables to store data temporarily
"""
# set to store the ids of the seen objects
seen_objs = set()

# max duration to wait after exploration is done
max_duration = rospy.get_param("~duration", 20)

# distance between object and rosbot using laser scan
x_pos=0

# list to store the initial position
origin = None

# instance of Path
path = Path()

# instance of PoseStamped
pose = PoseStamped()


"""
    publishing topics:
        1. hazards: for hazards marker position
        2. path : for returning the path of the rosbot

    subscribing topics:
        1. map : to get the area explored by rosbot
        2. odom : to get the rosbot pose
        3. scan : to get the distance between the marker and rosbot
        4. objectsStamped : to store the data from the /objects topic with time stamp
"""
pub_hazards = rospy.Publisher('hazards', Marker)
pub_path = rospy.Publisher('/path', Path, queue_size=100)

rospy.Subscriber('map', OccupancyGrid, map_callback)
odom = rospy.Subscriber('/odom', Odometry, get_odom)
laserSub = rospy.Subscriber('/scan',LaserScan,getLaserScan)
rospy.Subscriber('/objectsStamped', ObjectsStamped, callback)


"""
    afterExplore: Function to store the map, return the rosbot to initial position and shutdown the rospy

    parametrs:
        t_start : current time
"""
def afterExplore(t_start):
    try:
        # store the map in the give path
        os.system("rosrun map_server map_saver -f /home/karthi/ros_challenge_1/src/ros_challenge/maps/saved_map")
        # go back to initial position
        goBack()

        # shutdown rospy if it is still running
        while not rospy.is_shutdown():
            t_now = rospy.Time.now()
            if t_now - t_start > rospy.Duration(max_duration):
                rospy.on_shutdown(shutdown)
    except:
        print("[Error] afterExplore: function failed")


"""
    goBack: To return the rosbot to initial position
"""
def goBack():
    try:
        # contains the initial position
        global origin

        # instance of MoveBaseAction
        action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # waiting fot action server to start
        action_client.wait_for_server()

        # creating a goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # giving the coordinates of origin to set the goal
        goal.target_pose.pose.position.x = origin[0]
        goal.target_pose.pose.position.y = origin[1]
        goal.target_pose.pose.position.z = origin[2]

        # sending the rosbot to the goal
        action_client.send_goal(goal)
        wait = action_client.wait_for_result()

        # if server is not available, shutdown the signal
        if not wait:
            rospy.signal_shutdown("Action server not available!")
        else:
            # getting results for the goal
            return action_client.get_result()
    except:
        print("[Error] goBack: not able to go back to initial pose")


"""
    shutdown: Shutdown the rospy in 10 seconds
"""
def shutdown():
    print('Inside shutdown()')
    rospy.sleep(10)


"""
    marker_obj: Funciton to publish the marker on the given coordinate
"""
def marker_obj():
    global x_pos
    global pose
    marker = Marker()
    marker.header.frame_id = "/neck"

    # setting the marker properties
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0

    # adding the distance between the marker and rosbot to the poisiton of rosbot
    marker.pose.position.x = pose.position.x + x_pos
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z
    marker.pose.orientation.w = 1.0

    return marker



"""
    callback: Getting the callback once the rosbot sees the marker

    parametrs:
        data : id, orientation and frame of the marker seen, inside an array
"""
def callback(data):
    print("Objects: :", data.objects.data)
    if len(data.objects.data) > 0:
        if data.objects.data[0] not in seen_objs:
            seen_objs.add(data.objects.data[0])
            marker = marker_obj()
            publisher.publish(marker)


"""
    get_odom : Getting the odometry data to get the rosbot pose, publishing the path

    parametrs:
        msg: current position and orientation of rosbot
"""
def get_odom(msg):
    global pose
    path.header = msg.header
    pose.header = msg.header
    pose.pose = msg.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)


"""
    getLaserScan: Fetching the distance between the obstacle and rosbot using laser scan

    parametrs:
        msg: data from laser scan, 0th position stores the x distance
"""
def getLaserScan(msg):
    global x_pos
    x_pos = msg.ranges[0]


"""
    map_callback: Getting the map occpancy data from rosbot

    parametrs:
        OccupancyGrid : to get the original position of rosbot
"""
def map_callback(OccupancyGrid):
    global origin

    origin = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z])

    rospy.sleep(1)


"""
    main : Setting up the node and waiting for exploration to end
"""
if __name__ == '__main__':
    try:
        # setting the node name, for launch file to identify
        rospy.init_node('explore_node')
        # storing the current time
        t_start = rospy.Time.now()
        # start the exploration at stored time
        afterExplore(t_start)
        # sleep after 30 seconds
        rospy.sleep(30)

    except rospy.ROSInterruptException:
        pass
