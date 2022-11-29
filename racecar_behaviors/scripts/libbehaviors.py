#!/usr/bin/env python3

import rospy
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.srv import GetMap
from math import sqrt
from matplotlib import pyplot as plt

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

def yaw_to_quaternion(yaw):
    return quaternion_from_euler(0, 0, yaw)

def multiply_transforms(trans1, rot1, trans2, rot2):
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)
    
    return trans3, rot3


def brushfire(occupancyGrid):

    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)

    mapOfWorld[occupancyGrid==0] = 1 # obstacles
    mapOfWorld[occupancyGrid==89] = 1 # unknowns

    # do brushfire algorithm here
    for level in range(1, 30):
        for x in range(0, mapOfWorld.shape[0]):
            for y in range(0, mapOfWorld.shape[1]):
                if mapOfWorld[x, y] == level:
                    prevX = x - 1 if x - 1 >= 0 else x
                    prevY = y - 1 if y - 1 >= 0 else y
                    nextX = x + 1 if x + 1 < mapOfWorld.shape[0] else x
                    nextY = y + 1 if y + 1 < mapOfWorld.shape[1] else y

                    mapOfWorld[prevX, y] = increment_level(mapOfWorld[prevX, y], level)
                    mapOfWorld[nextX, y] = increment_level(mapOfWorld[nextX, y], level)
                    mapOfWorld[x, prevY] = increment_level(mapOfWorld[x, prevY], level)
                    mapOfWorld[x, nextY] = increment_level(mapOfWorld[x, nextY], level)

        print("Completed level {0}\n".format(level))

    # Make the safer places a lower value (for later use in cost map)
    brushfire_cost_map = abs(np.amax(mapOfWorld) - mapOfWorld)
    brushfire_cost_map[occupancyGrid==0] = 100000 # obstacles
    brushfire_cost_map[occupancyGrid==89] = 100000 # unknowns
    return brushfire_cost_map*2

def increment_level(value, level):
    return level + 1 if value == 0 else value


def eucledian_map(grid, start_x, start_y):
    mapOfWorld = np.zeros(grid.shape, dtype=int)
    for y in range(mapOfWorld.shape[0]):
        for x in range(mapOfWorld.shape[1]):
            mapOfWorld[y][x] = sqrt((x-start_x)**2+(y-start_y)**2)
            
    return mapOfWorld

def get_map():
    rospy.init_node('brushfire')
    prefix = "racecar"
    rospy.wait_for_service(prefix + '/get_map')
    try:
        get_map = rospy.ServiceProxy(prefix + '/get_map', GetMap)
        response = get_map()
    except (rospy.ServiceException) as e:
        print("Service call failed: %s"%e)
        return
    
    rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)    
    grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
    grid[grid == -1] = 89
    grid[grid == 0] = 178
    grid[grid == 100] = 0
    cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(grid, -1)))
    brushfireMap = brushfire(grid)
    return np.flip(grid.T), np.flip(brushfireMap)