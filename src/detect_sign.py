#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

import cv2
import numpy as np
import sys
import os
import imutils

import tensorflow
from PIL import Image, ImageOps

from enums import *


class Detect_Sign:
    
    def __init__(self):
        '''
        Initialise the ROS node for detecting signs and subscriber for getting images from RasPi camera
        
        Args: None
        Returns: None
        '''

        self.cnn_model = tensorflow.keras.models.load_model('keras_model.h5', compile=False)
        self.img_size = (224,224)
        self.detected_sign = None
        self.latest_img = np.ndarray(shape = (1,224,224,3), dtype = np.float32)

        # rospy.init_node('detect_sign_node', anonymous=True)
        
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.get_latest_img_callback, queue_size = 1, buff_size=2**24)


    def get_latest_img_callback(self, img_data):
        '''
        Function which stores the latest image input from RasPi camera which will be used for detecting the sign 

        Args: 
            img_data: buffer image from standard ROS compressed image message

        Returns:
            None 
        '''

        single_img_batch = np.ndarray(shape = (1,224,224,3), dtype = np.float32)

        img_np_arr = np.fromstring(img_data.data, np.uint8)
        PIL_image = Image.fromarray(np.uint8(img_np_arr)).convert('RGB')

        print(type(PIL_image))

        img_cropped = ImageOps.fit(PIL_image, self.img_size, Image.ANTIALIAS)

        print("\n img cropped:", type(img_cropped))

        img_cropped_array = np.asarray(img_cropped)

        normalized_img_arr = (img_cropped_array.astype(np.float32) / 127.0) - 1

        single_img_batch[0] = normalized_img_arr

        self.latest_img = single_img_batch[0]


    def detect_sign(self):
        '''
        Function which detects the sign in the latest image from the RasPi camera
        
        Args: 
            None
        Returns:
            None
        '''

        prediction = self.cnn_model.predict(self.latest_img)

        prediction_idx = np.argmax(prediction)

        if prediction_idx == 0:
            self.detected_sign = Sign.NO_SIGN
        
        elif prediction_idx == 1:
            self.detected_sign = Sign.LEFT
        
        elif prediction_idx == 2:
            self.detected_sign = Sign.RIGHT
        
        elif prediction_idx == 3 or prediction_idx == 4:
            self.detected_sign = Sign.U_TURN
        
        else:
            self.detected_sign = Sign.GOAL

        return self.detected_sign

