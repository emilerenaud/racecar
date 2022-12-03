#!/usr/bin/env python

import rospy
import cv2
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
    
    return (trans3, rot3)

def brushfire(occupancyGrid, use8CellWindow=False):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)

    mapOfWorld[occupancyGrid==100] = 1 # obstacles
    mapOfWorld[occupancyGrid==-1] = 1  # unknowns
    
    # do brushfire algorithm here
    a = 0
    nRows, nCols = mapOfWorld.shape

    
    while 0 in mapOfWorld:
      a = a + 1
      for iRow in range(nRows):
        for iCol in range(nCols):
          if mapOfWorld[iRow][iCol] == a:
    	
    	    # 4-cells window
            if (iRow > 0): 
              if (mapOfWorld[iRow-1][iCol] == 0) : mapOfWorld[iRow-1][iCol] = a + 1
            if (iRow < nRows-1): 
              if (mapOfWorld[iRow+1][iCol] == 0) : mapOfWorld[iRow+1][iCol] = a + 1
            if (iCol > 0):
              if (mapOfWorld[iRow][iCol-1] == 0) : mapOfWorld[iRow][iCol-1] = a + 1
            if (iCol < nCols-1):
              if (mapOfWorld[iRow][iCol+1] == 0) : mapOfWorld[iRow][iCol+1] = a + 1
        
            # 8-cells window
            if (use8CellWindow):
              if (iRow > 0) and (iCol > 0):
                if (mapOfWorld[iRow-1][iCol-1]) == 0 : mapOfWorld[iRow-1][iCol-1] = a + 1
              if (iRow > 0) and (iCol < nCols-1):
                if (mapOfWorld[iRow-1][iCol+1]) == 0 : mapOfWorld[iRow-1][iCol+1] = a + 1
              if (iRow < nRows-1) and (iCol > 0):
                if (mapOfWorld[iRow+1][iCol-1]) == 0 : mapOfWorld[iRow+1][iCol-1] = a + 1
              if (iRow < nRows-1) and (iCol < nCols-1):
                if (mapOfWorld[iRow+1][iCol+1]) == 0 : mapOfWorld[iRow+1][iCol+1] = a + 1
    
    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    return mapOfWorld
    

