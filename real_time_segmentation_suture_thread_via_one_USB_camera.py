# -*- coding: utf-8 -*-
"""
Created on Tue Aug 27 20:39:27 2019

@author: User
"""
# In[]
import numpy as np
from keras.models import *
from keras.layers import *
from keras.applications.vgg16 import VGG16
from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import *
from keras.callbacks import ModelCheckpoint
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import skimage  
from unet_functions import *

# In[]
model_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Deep_learning_based_methods/unet_keras-master/'
model = vgg10_unet(input_shape=(512,512,3))
model.load_weights(model_path + 'unet_gray_dVRK_image_1to60.h5')
#model.load_weights(model_path + 'unet_gray_image_81_150_thick_suture.h5')


def save_max_objects(img):
    #find all your connected components (white blobs in your image)
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(np.uint8(255*((img/255)*(-1) + 1)), connectivity=8)
    #connectedComponentswithStats yields every seperated component with information on each of them, such as size
    #the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
    sizes = stats[1:, -1]; nb_components = nb_components - 1
    
    # minimum size of particles we want to keep (number of pixels)
    #here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
    min_size = 550  
    
    #your answer image
    img2 = np.zeros((output.shape))
    #for every component in the image, you keep it only if it's above min_size
    for i in range(0, nb_components):
        if sizes[i] >= min_size:
            img2[output == i + 1] = 255
    
    return np.uint8(255*((img2/255)*(-1) + 1))

cap = cv2.VideoCapture(1)

# Define the codec and create VideoWriter object
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output4.avi',fourcc, 20.0, (640,480))
erode_kernel = np.ones((3, 3),np.uint8)
#dilate_kernel = np.ones((2, 2),np.uint8)

   ## In[]
while(cap.isOpened()):
    ret, frame = cap.read()
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    if ret==True:
        frame = cv2.flip(frame,0)
        resized_frame = cv2.resize(frame, (512, 512))
        img_r = cv2.resize(frame, (512, 512))
        #img_r_gray = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        img_r = img_r / 255.0
        img_r = np.array([img_r])
        mask = model.predict(img_r, batch_size=None, verbose=0, steps=None)
        mask = mask[0]
        mask = (mask * 255)
        
        
        img_r = 255 * img_r
        img_r = img_r[0,:,:,:]
        mask = np.uint8(mask)
        
        mask_pro = cv2.erode(mask,erode_kernel,iterations = 1)
        #mask_pro = cv2.dilate(mask,dilate_kernel,iterations = 1)
        #dst = cv2.fastNlMeansDenoisingColored(mask,None,10,10,7,21)
        
        ret, thresh1 = cv2.threshold(mask_pro,127,255,cv2.THRESH_BINARY)
        picked_shape = save_max_objects(thresh1)
        
        [a,b] = np.where(picked_shape < 200) # for highlighting the structure
        
        red = [0,0,255]
        #black = 255
        #white = 0
        for x, y in zip(a, b):
            resized_frame[x, y] = red  
                        
        resized_back_frame = cv2.resize(resized_frame, (640, 480))
        #mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
        #mask = mask.astype(int)
        #imgplot = plt.imshow(mask[:, :, 1])
        s1 = np.concatenate((frame, resized_back_frame), axis=1)
        
        
        cv2.imshow('Suture Segmentation 2 ', s1)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

    else:
        break
    
# Release everything if job is finished
cap.release()
#out.release()
cv2.destroyAllWindows()

# In[]
import numpy as np
import cv2
import time

cap = cv2.VideoCapture(0)

if (cap.isOpened()== False): 
  print("Error opening video stream or file")
  
while(cap.isOpened()):
    ret, frame = cap.read()
    
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if ret==True:
        frame = cv2.flip(frame,0)
        resized_frame = cv2.resize(frame, (512, 512))
        img_r = cv2.resize(frame, (512, 512))
        #img_r_gray = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        img_r = img_r / 255.0
        img_r = np.array([img_r])
        mask = model.predict(img_r, batch_size=None, verbose=0, steps=None)
        mask = mask[0]
        mask = (mask * 255)
        
        
        img_r = 255 * img_r
        img_r = img_r[0,:,:,:]
        mask = np.uint8(mask)
        
        mask_pro = cv2.erode(mask,erode_kernel,iterations = 1)
        #mask_pro = cv2.dilate(mask,dilate_kernel,iterations = 1)
        #dst = cv2.fastNlMeansDenoisingColored(mask,None,10,10,7,21)
        
        ret, thresh1 = cv2.threshold(mask_pro,127,255,cv2.THRESH_BINARY)
        picked_shape = save_max_objects(thresh1)
        
        [a,b] = np.where(picked_shape < 240) # for highlighting the structure
        
        red = [0,0,255]
        #black = 255
        #white = 0
        for x, y in zip(a, b):
            resized_frame[x, y] = red  
           
                     
        resized_back_frame = cv2.resize(resized_frame, (640, 480))
        #mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
        #mask = mask.astype(int)
        #imgplot = plt.imshow(mask[:, :, 1])
        
        cv2.imshow('Suture Segmentation ', resized_back_frame)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        time.sleep(0.2)
    else:
        break
    #time.sleep(0.2)
# Release everything if job is finished
cap.release()
#out.release()
cv2.destroyAllWindows()