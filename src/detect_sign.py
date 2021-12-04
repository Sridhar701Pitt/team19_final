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

import enums


class Detect_Sign:

    self.cnn_model = tensorflow.keras.models.load_model('keras_model.h5', compile=False)
    self.img_size = (224,224)
    self.detected_sign = None

    
    def __init__(self):
        '''
        Initialise the ROS node fro detecting signs and subscriber for getting images from RasPi camera
        
        Args: None
        Returns: None
        '''

        rospy.init_node('detect_sign_node', anonymous=True)

	    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, detect_sign_callback, queue_size = 1, buff_size=2**24)        


    def detect_sign_callback(self, img_data):
        '''
        Call back function for the image subscriber which passes the input image through the trained CNN to generate a prediction for the detected sign
        
        Args: 
            img_data: buffer image from standard ROS compressed image message
        
        Returns:
            None
        '''

        single_img_batch = np.ndarray(shape = (1,224,224,3), dtype = np.float32)

        img_np_arr = np.fromstring(img_data.data, np.uint8)
        PIL_image = Image.fromarray(np.uint8(img_np_arr)).convert('RGB')

        img_cropped = ImageOps.fit(PIL_image, self.img_size, Image.ANTIALIAS)

        img_cropped_array = np.asarray(img_cropped)

        normalized_img_arr = (img_cropped_array.astype(np.float32) / 127.0) - 1

        single_img_batch[0] = normalized_image_arr

        prediction = self.cnn_model.predict(single_img_batch)

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