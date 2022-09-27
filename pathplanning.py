#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import math
import matplotlib.pyplot as plt

def BFS (map, start, goal):
    queue = [start]
    visited = []
    path = []
    parent = {}
    while queue:
        current = queue.pop(0) 
        if current == goal:
            path = path_map(start, goal, parent)
            rospy.loginfo(map)
            return path

        else:
            rospy.loginfo(current)
            visited.append(current)
            map[current[0]][current[1]] = 1
            adjacent = adjacent_map(map, current)
            for n in adjacent:
                if n not in visited and n not in queue and map[n[0]][n[1]] == 0:
                    queue.append(n)
                    parent[n] = current

def adjacent_map(map, node):
    x = node[0]
    y = node[1]

    (height, width) = map.shape
    valid_neighbour = []

    adjacent = [(x, y+1), (x+1,y), (x+1,y+1), (x,y-1), (x-1,y), (x-1,y-1), (x+1,y-1), (x-1,y+1)]
    for n in adjacent:
        if n[0] >= 0 and n[0] < height and n[1] >= 0 and n[1] < width: 
            valid_neighbour.append(n)
    return valid_neighbour

def path_map(start, goal, parent):
    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

####################################

def callback(msg):
    map = OccupancyGrid()
    map = msg.data
    info = msg.info
    start = (int(info.origin.position.x), int(info.origin.position.y))
    width = int(info.width)
    height = int(info.height)
    goal = (info.height - 1, info.width - 1)
    map = np.array(map)
    map = np.resize(map, (info.height, info.width))   
    # rospy.loginfo(start)
    path = BFS(map, start, goal)
    print('Got path')
    
    stamped_poses = []
    for point in (path):
        datatype_point = Point(point[1], point[0], 0)
        datatype_pose = Pose(datatype_point, Quaternion(0, 0, 0, 1))
        datatype_pose_stamped = PoseStamped(header=None, pose=datatype_pose)
        datatype_pose_stamped.header.frame_id = 'map' 
        stamped_poses.append(datatype_pose_stamped)
        
    datatype_path = Path(header=None, poses=stamped_poses)
    datatype_path.header.frame_id = 'map' #change
    rospy.loginfo(datatype_path)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo('Publishing Path..')
        pub.publish(datatype_path)
        rate.sleep()
    print('Published path')

if __name__ == '__main__':
    try:
        rospy.init_node('map_subscriber')
        pub = rospy.Publisher('path', Path, queue_size=10)
        sub = rospy.Subscriber('map', OccupancyGrid, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass