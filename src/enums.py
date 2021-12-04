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