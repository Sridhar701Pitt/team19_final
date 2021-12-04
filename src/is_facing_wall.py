#!/usr/bin/env python

# Python 2.x: pip install enum34
from enum import Enum

class Quadrant(Enum):
    N = 0
    E = 1
    S = 2
    W = 3

class Sign(Enum):
    GOAL = 0
    RIGHT = 1
    LEFT = 2
    U_TURN = 3
    NO_SIGN = 4
    

def is_facing_wall(fov, quadrant):
    '''
    Function to find if the specified quadrant contaisn a wall and the distance to that wall 
    Args:
        # lidar_scan: ROS standard scan message
        fov: field of view of a given quadrant
        quadrant: enum variable which holds the direction flag values
    
    Return:
        is_facing_wall: Bool value which denotes if the robot is facing a wall or not
        dist_to_wall: float value which gives the distance to the wall that the robot is facing
    '''

    # Check if the enum variable has the corretc type
    if not isinstance(quadrant, Quadrant):
        raise TypeError('quadrant must be an instance of Quadrant Enum')

    
    