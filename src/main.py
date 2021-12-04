#!/usr/bin/env python

import enums

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
            
