#!/usr/bin/env python

from enums import *
from move_robot import *
from detect_sign import *
import rospy

def test_linear_motion():

    # Instantiate class move_robot
    move_object = move_robot()

    move_object.move_robot_forward()

    print("success linear")

def test_rotate_robot_precise():

    # Instantiate class move_robot
    move_object = move_robot()
    
    move_object.rotate_robot_precise(Quadrant.N)    # Quadrant.N for testing rotate_robot_precise just after linear motion is performed

    print('success precise')

def test_rotate_robot():

    # Instantiate class move_robot
    move_object = move_robot()
    
    move_object.rotate_robot(Sign.U_TURN)

    print('success rotate')

def test_classify():
    # Instantiate class move_robot
    move_object = move_robot()

    # Instantiate class move_robot
    detect_sign_object = Detect_Sign()

    while True:

        sign_command = detect_sign_object.detect_sign()

        if sign_command == Sign.GOAL:
            print("\n GOAL")

        elif sign_command == Sign.LEFT:
            print("\n LEFT")
        
        elif sign_command == Sign.RIGHT:
            print("\n RIGHT")
        
        elif sign_command == Sign.NO_SIGN:
            print("\n NO_SIGN")
        
        elif sign_command == Sign.U_TURN:
            print("\n U_TURN")
        
        else:
            print("\n UN_IDENTIFIED")
        

if __name__ == '__main__':

    # # Instantiate class move_robot
    # move_object = move_robot()

    # # Instantiate class detect_sign
    # detect_sign_object = Detect_Sign()

    # while True:
        
    #     if move_object.is_facing_wall(Quadrant.N)[0]: 
            
    #         # Find the sign
    #         sign_command = detect_sign_object.detect_sign()
            
    #         if sign_command != Sign.NO_SIGN and sign_command != Sign.GOAL:
    #             # execute rotation
    #             move_object.rotate_robot(sign_command)
                
    #             if sign_command == Sign.RIGHT:
    #                 move_object.rotate_robot_precise(Quadrant.W)
                
    #             elif sign_command == Sign.LEFT:
    #                 move_object.rotate_robot_precise(Quadrant.E)
                
    #             else: #Sign.U-TURN
    #                 move_object.rotate_robot_precise(Quadrant.S)

    #         elif sign_command == Sign.GOAL:
    #             move_object.stop_robot()
    #             rospy.loginfo('Goal Reached')
    #             print('Goal Reached')
    #             print("Yoyo gogogo HAHAHAH!!!!")
    #             break
            
    #         else: # Sign.NO_SIGN
    #             rospy.logerr("Sign not found, recovery behaviour")
    #             # execute recovery behaviour - turn to empty space and keep moving
    #             _, distance_e = move_object.is_facing_wall(Quadrant.E)
    #             _, distance_w = move_object.is_facing_wall(Quadrant.W)

    #             if distance_e != -1:
    #                 move_object.rotate_robot(Sign.RIGHT)
    #             elif distance_w != -1:
    #                 move_object.rotate_robot(Sign.LEFT)
    #             else:
    #                 move_object.rotate_robot(Sign.U_TURN)
        
    #     else:
    #         move_object.move_robot_forward()
    #         move_object.rotate_robot_precise(Quadrant.N)

    # test_linear_motion()

    # test_rotate_robot_precise()
 
    # test_rotate_robot()

    test_classify()

    rospy.sleep(3)

    # rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
