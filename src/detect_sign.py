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
        self.cnn_model_left_right = tensorflow.keras.models.load_model('keras_model_left_right.h5', compile=False)
        self.img_size = (224,224)
        self.detected_sign = None

        # self.latest_img_for_CNN = np.ndarray(shape = (1,224,224,3), dtype = np.float32)
        self.current_img = None

        self.latest_img = np.ndarray(shape = (1,224,224,3), dtype = np.float32)
        self.writeidx = 0

        # rospy.init_node('detect_sign_node', anonymous=True)
        
        self.camera_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.get_latest_img_callback, queue_size = 1, buff_size=2**24)


    def get_latest_img_callback(self, img_data):
        '''
        Function which stores the latest image input from RasPi camera which will be used for detecting the sign 

        Args: 
            img_data: buffer image from standard ROS compressed image message

        Returns:
            None 
        '''

        img_np_arr = np.fromstring(img_data.data, np.uint8)
        cv_image = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)

        # cv2.imshow("window",cv_image)

        # if cv2.waitKey(5) != -1:
        #     self.writeidx = self.writeidx + 1
        #     filename = 'nosign/nosign_' + str(self.writeidx) + '.jpg'
        #     cv2.imwrite(filename, cv_image)
        #     print('image saved')

        self.current_img = cv_image


    def detect_sign(self):
        '''
        Function which detects the sign in the latest image from the RasPi camera
        
        Args: 
            None
        Returns:
            None
        '''

        # h, w, c = cv_image.shape
        # print("\n Image height: ", h, " Image width: ", w, " Image channels: ", c)

        while self.current_img is None:
            rospy.sleep(0.1)

        single_img_batch = np.ndarray(shape = (1,224,224,3), dtype = np.float32)
        
        cv_image = cv2.cvtColor(self.current_img, cv2.COLOR_BGR2RGB)
        PIL_image = Image.fromarray(cv_image)

        width, height = PIL_image.size

        # print("\n Image width: ", width, " Image height: ", height)

        img_cropped = ImageOps.fit(PIL_image, self.img_size, Image.ANTIALIAS)
        width_cropped, height_cropped = img_cropped.size

        # print("\n img cropped - width: ", width_cropped, " height: ", height_cropped)

        img_cropped_array = np.asarray(img_cropped)

        normalized_img_arr = (img_cropped_array.astype(np.float32) / 127.0) - 1

        single_img_batch[0] = normalized_img_arr

        latest_img_for_CNN = single_img_batch

        # ---------------------------------------------------------------------------------------

        prediction = self.cnn_model.predict(latest_img_for_CNN)

        prediction_idx = np.argmax(prediction)

        if prediction_idx == 0:
            self.detected_sign = Sign.NO_SIGN
            print('-> SIGN NOT DETECTED')
        
        elif prediction_idx == 1 or prediction_idx == 2:
            # self.detected_sign = Sign.LEFT
            # self.detected_sign = Sign.RIGHT

            prediction_drxn = self.cnn_model_left_right.predict(latest_img_for_CNN)
            prediction_drxn_idx = np.argmax(prediction_drxn)

            if prediction_drxn_idx == 0:
                self.detected_sign = Sign.LEFT
                print('-> TURN LEFT')
            
            elif prediction_drxn_idx == 1:
                self.detected_sign = Sign.RIGHT
                print('-> TURN RIGHT')
        
        elif prediction_idx == 3 or prediction_idx == 4:
            self.detected_sign = Sign.U_TURN
            print('-> U-TURN')
        
        else:
            self.detected_sign = Sign.GOAL
            print('-> GOAL REACHED')

        return self.detected_sign