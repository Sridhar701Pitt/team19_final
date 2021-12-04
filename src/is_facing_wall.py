#!/usr/bin/env python    

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

    
    