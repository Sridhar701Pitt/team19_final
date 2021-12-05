#!/usr/bin/env python

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
            sign_command = detect_sign_object.detect_sign()
            if sign_command != Sign.NO_SIGN or sign != Sign.GOAL:
                # execute rotation
                move_object.rotate_robot(sign_command)
                if sign_command == Sign.RIGHT:
                    move_object.rotate_robot_precise(Quadrant.W)
                elif sign_command == Sign.LEFT:
                    move_object.rotate_robot_precise(Quadrant.E)
                else: #Sign.U-TURN
                    move_object.rotate_robot_precise(Quadrant.S)

            elif sign_command == Sign.GOAL:
                print('Goal Reached')
                break
            else: # Sign.NO_SIGN
                print("Sign not found, recovery behaviour")
                # execute recovery behaviour
                continue
        else:
            move_object.move_robot_forward()
            move_object.rotate_robot_precise(Quadrant.N)    
            
