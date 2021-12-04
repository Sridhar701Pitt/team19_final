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

if __name__ == '__main__':

    while True:
        if is_facing_wall(fov, Quadrant.N): 
            sign = detect_sign()
            if sign != Sign.NO_SIGN or sign != Sign.GOAL:
                # execute rotation
                rotate_robot(sign)
            elif sign == Sign.GOAL:
                break
            else:
                # execute recovery behaviour
                pass
        else:
            move_forward_robot()
            rotate_precise_robot()    
            
