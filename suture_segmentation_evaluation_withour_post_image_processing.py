# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 17:44:44 2020

@author: User
"""

import numpy as np
from keras.models import *
from keras.layers import *
from keras.applications.vgg16 import VGG16
from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import *
from keras.callbacks import ModelCheckpoint
import cv2


model_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Deep_learning_based_methods/unet_keras-master/'
model_name = 'unet_gray_ICL_dVRK_1_to_20_fine_tune.h5'

input_image_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Deep_learning_based_methods/frames/images_dVRK/with_ICL/evaluation/input_images/'
img_saving_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Deep_learning_based_methods/frames/images_dVRK/with_ICL/evaluation/segmentation_results/with_transfer/'

model = vgg10_unet(input_shape=(512,512,3))
model.load_weights(model_path + model_name)

for i in range(41, 61): 
    x = cv2.imread(input_image_path + '%d.png'%i)
            
    x = cv2.resize(x, (512, 512))
            
    #cv2.imshow('image', x)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    x = x / 255.0
    x = np.array([x])
    mask = model.predict(x, batch_size=None, verbose=0, steps=None)
    mask = mask[0]
    mask = mask * 255
    mask = cv2.resize(mask, (640, 480))
    #ret,th1 = cv2.threshold(mask, 27,255,cv2.THRESH_BINARY)
    
    cv2.imwrite(img_saving_path + '%d.jpg'%i, mask)
    #cv2.imshow('mask', mask)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()