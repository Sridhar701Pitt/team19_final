#!/usr/bin/env python

from enums import *
from move_robot import *
from detect_sign import *


def test_linear_motion():

    # Instantiate class move_robot
    move_object = move_robot()

    move_object.move_robot_forward()


if __name__ == '__main__':

    '''
    # Instantiate class move_robot
    move_object = move_robot()
    
    # Instantiate class detect_sign
    detect_sign_object = Detect_Sign()

    while True:
        
        if move_object.is_facing_wall(Quadrant.N)[0]: 
            
            # Find the sign
            sign_command = detect_sign_object.detect_sign()
            
            if sign_command != Sign.NO_SIGN and sign_command != Sign.GOAL:
                
                # execute rotation
                move_object.rotate_robot(sign_command)
                
                if sign_command == Sign.RIGHT:
                    move_object.rotate_robot_precise(Quadrant.W)
                
                elif sign_command == Sign.LEFT:
                    move_object.rotate_robot_precise(Quadrant.E)
                
                else: #Sign.U-TURN
                    move_object.rotate_robot_precise(Quadrant.S)

            elif sign_command == Sign.GOAL:
                rospy.loginfo('Goal Reached')
                print('Goal Reached')
                break
            
            else: # Sign.NO_SIGN
                rospy.logerr("Sign not found, recovery behaviour")
                print("Sign not found, recovery behaviour")
                # execute recovery behaviour
                continue
        
        else:
            move_object.move_robot_forward()
            move_object.rotate_robot_precise(Quadrant.N)
    
    '''

    test_linear_motion()