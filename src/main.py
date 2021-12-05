#!/usr/bin/env python

from ROS.catkin_ws.src.team19_final.src.enums import Quadrant
import enums
import move_robot
import detect_sign


if __name__ == '__main__':

    # Instantiate class move_robot
    move_object = move_robot()
    # Instantiate class detect_sign
    detect_sign_object = detect_sign()

    while True:
        if move_object.is_facing_wall(Quadrant.N)[0]: 
            # Find the sign
            sign_command = detect_sign_object.sign_detect()
            if sign_command != Sign.NO_SIGN or sign != Sign.GOAL:
                # execute rotation
                move_object.rotate_robot(sign_command)
                if sign_command == Sign.RIGHT:
                    move_object.rotate_robot_precise(Quadrant.W)
                elif sign_command == Sign.U_TURN:
                    move_object.rotate_robot_precise(Quadrant.S)

            elif sign == Sign.GOAL:
                print('Goal Reached')
                break
            else:
                # execute recovery behaviour
                pass
            del
        else:
            move_forward_robot()
            rotate_precise_robot()    
            
