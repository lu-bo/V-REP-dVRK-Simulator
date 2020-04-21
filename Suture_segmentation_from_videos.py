# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 16:13:17 2019

@author: User
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Sep 16 16:35:44 2019

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

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import skimage

# In[]
def train_generator(batch_size=32):
    data_gen_args = dict(featurewise_center=True,
                         rotation_range=90.,
                         width_shift_range=0.1,
                         height_shift_range=0.1,
                         fill_mode="constant",
                         cval=255,
                         horizontal_flip=True,
                         vertical_flip=True,
                         zoom_range=0.2)
    image_datagen = ImageDataGenerator(**data_gen_args)
    mask_datagen = ImageDataGenerator(**data_gen_args)

    seed = 1
    image_generator = image_datagen.flow_from_directory(
        '../frames/data/trianing/img_gray',
        class_mode=None,
        batch_size=batch_size,
        color_mode='rgb',
        target_size=(512,512),
        #save_to_dir='./data/gen/images',
        seed=seed)

    mask_generator = mask_datagen.flow_from_directory(
        '../frames/data/trianing/masks',
        class_mode=None,
        color_mode='grayscale',
        target_size=(512,512),
        batch_size=batch_size,
        #save_to_dir='./data/gen/masks',
        seed=seed)

    # combine generators into one which yields image and masks
    train_generator = zip(image_generator, mask_generator)
    for (imgs, masks) in train_generator:
        imgs = imgs / 255.0
        masks = masks / 255.0
        yield (imgs,masks)


def vgg10_unet(input_shape=(256,256,3), weights='imagenet'):
    vgg16_model = VGG16(input_shape=input_shape, weights=weights, include_top=False)

    block4_pool = vgg16_model.get_layer('block4_pool').output
    block5_conv1 = Conv2D(1024, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block4_pool)
    block5_conv2 = Conv2D(1024, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block5_conv1)
    block5_drop = Dropout(0.5)(block5_conv2)

    block6_up = Conv2D(512, 2, activation='relu', padding='same', kernel_initializer='he_normal')(
        UpSampling2D(size=(2, 2))(block5_drop))
    block6_merge = Concatenate(axis=3)([vgg16_model.get_layer('block4_conv3').output, block6_up])
    block6_conv1 = Conv2D(512, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block6_merge)
    block6_conv2 = Conv2D(512, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block6_conv1)
    block6_conv3 = Conv2D(512, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block6_conv2)

    block7_up = Conv2D(256, 2, activation='relu', padding='same', kernel_initializer='he_normal')(
        UpSampling2D(size=(2, 2))(block6_conv3))
    block7_merge = Concatenate(axis=3)([vgg16_model.get_layer('block3_conv3').output, block7_up])
    block7_conv1 = Conv2D(256, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block7_merge)
    block7_conv2 = Conv2D(256, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block7_conv1)
    block7_conv3 = Conv2D(256, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block7_conv2)

    block8_up = Conv2D(128, 2, activation='relu', padding='same', kernel_initializer='he_normal')(
        UpSampling2D(size=(2, 2))(block7_conv3))
    block8_merge = Concatenate(axis=3)([vgg16_model.get_layer('block2_conv2').output, block8_up])
    block8_conv1 = Conv2D(128, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block8_merge)
    block8_conv2 = Conv2D(128, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block8_conv1)

    block9_up = Conv2D(64, 2, activation='relu', padding='same', kernel_initializer='he_normal')(
        UpSampling2D(size=(2, 2))(block8_conv2))
    block9_merge = Concatenate(axis=3)([vgg16_model.get_layer('block1_conv2').output, block9_up])
    block9_conv1 = Conv2D(64, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block9_merge)
    block9_conv2 = Conv2D(64, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block9_conv1)

    block10_conv1 = Conv2D(64, 3, activation='relu', padding='same', kernel_initializer='he_normal')(block9_conv2)
    block10_conv2 = Conv2D(1, 1, activation='sigmoid')(block10_conv1)

    model = Model(inputs=vgg16_model.input, outputs=block10_conv2)
    return model

def save_max_objects(img):
    #find all your connected components (white blobs in your image)
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(np.uint8(255*((img/255)*(-1) + 1)), connectivity=8)
    #connectedComponentswithStats yields every seperated component with information on each of them, such as size
    #the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
    sizes = stats[1:, -1]; nb_components = nb_components - 1
    
    # minimum size of particles we want to keep (number of pixels)
    #here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
    min_size = 30  
    
    #your answer image
    img2 = np.zeros((output.shape))
    #for every component in the image, you keep it only if it's above min_size
    for i in range(0, nb_components):
        if sizes[i] >= min_size:
            img2[output == i + 1] = 255
    
    return np.uint8(255*((img2/255)*(-1) + 1))

def DL_based_segmentation(frame):
    segmented_results = 255 * np.ones((512, 512), np.uint8)
    
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    #dilate_kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (1, 1))
    
    img_r = cv2.resize(frame, (512, 512))
    #img_r_gray = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
    img_r = img_r / 255.0
    img_r = np.array([img_r])
    mask = model.predict(img_r, batch_size=None, verbose=0, steps=None)
    mask = mask[0]
    mask = (mask * 255)   
    mask = np.uint8(mask)
        
    mask_pro = cv2.erode(mask,erode_kernel,iterations = 2)
    #mask_pro = cv2.dilate(mask,dilate_kernel,iterations = 1)
    #dst = cv2.fastNlMeansDenoisingColored(mask,None,10,10,7,21)
    
    ret, thresh1 = cv2.threshold(mask_pro,127,255,cv2.THRESH_BINARY)
    picked_shape = save_max_objects(thresh1)
    
    [a,b] = np.where(picked_shape < 150) # for highlighting the structure

    black = 0
    #black = 255
    #white = 0
    for x, y in zip(a, b):
        #resized_frame[x, y] = red 
        segmented_results[x, y] = black 
        
    return segmented_results

###########################################################
###########################################################
###########################################################
###########################################################
# In[]
model_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Deep_learning_based_methods/unet_keras-master/'
model = vgg10_unet(input_shape=(512,512,3))
#model.load_weights(model_path + 'unet_gray_image_01_180_dark_light_color.h5')
model.load_weights(model_path + 'unet_gray_ICL_dVRK_1_to_30_fine_tune.h5')

path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Paper_preparation/Online_video_tests/'
# -------- from USB camera -----------

#cap_1 = cv2.VideoCapture(0)
#cap_2 = cv2.VideoCapture(1)

cap_1 = cv2.VideoCapture(path + 'Simple Interrupted Suturing Trim3.mp4')
# Define the codec and create VideoWriter object
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output4.avi',fourcc, 20.0, (640,480))



c_t = 1
while(cap_1.isOpened()):
    cap_1.set(cv2.CAP_PROP_POS_MSEC, c_t) 
    ret_1, original_frame_1 = cap_1.read()
    L_O = np.shape(original_frame_1)[1]
    H_O = np.shape(original_frame_1)[0]
    
 
    ret_1, frame_1 = cap_1.read()
    #ret_2, frame_2 = cap_2.read()
    #segmented_results_1 = 255 * np.ones((512, 512), np.uint8)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    if ((ret_1==True)): #& (ret_2==True)):

        frame_L = frame_1[:, 0 : int(L_O / 2) ]
        frame_R = frame_1[:, int(L_O / 2 ) : L_O ]
        
        segmented_results_L = DL_based_segmentation(frame_L)
        segmented_results_R = DL_based_segmentation(frame_R)
                               
        #resized_back_frame = cv2.resize(resized_frame, (640, 480)).
        #cv2.imwrite('segmented_suture.jpg', resized_back_frame)
        
        segmented_results_sizeBack_L = cv2.resize(segmented_results_L, (int(L_O / 2), H_O))
        segmented_results_sizeBack_R = cv2.resize(segmented_results_R, (int(L_O / 2), H_O))
        
        montage_stereo_result = np.concatenate( (segmented_results_sizeBack_L, segmented_results_sizeBack_R) , axis = 1)
        
        #segmented_results_sizeBack_r = cv2.resize(segmented_results_2, (640, 480))
        [a,b] = np.where(montage_stereo_result < 100)
        red = [0,0,255]

        for x, y in zip(a, b):
            frame_1[x, y] = red  
        
        #cv2.imwrite((path + 'segmented_suture_l.jpg'), segmented_results_sizeBack_l)
        #cv2.imwrite((path + 'segmented_suture_r.jpg'), segmented_results_sizeBack_r)
        #print('new image is ready')
        #mask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
        #mask = mask.astype(int)
        #imgplot = plt.imshow(mask[:, :, 1])
        
        #segmented_results_sizeBack_l_3_channel = cv2.cvtColor(montage_stereo_result, cv2.COLOR_GRAY2BGR)       
        #numpy_vertical_concat = np.concatenate((frame_1, segmented_results_sizeBack_l_3_channel), axis=1)
        segmented_results_sizeBack_L = cv2.cvtColor(segmented_results_sizeBack_L, cv2.COLOR_GRAY2BGR)   
        numpy_vertical_concat = np.concatenate((original_frame_1[:, 0 : int(L_O/2)], frame_L), axis=1 )
        #numpy_vertical_concat = np.resize(numpy_vertical_concat, [int(L_O /2), H_O])
        
        
        cv2.imshow('Suture Segmentation ', numpy_vertical_concat)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

    else:
        print('Missing_frame')
        #cv2.imwrite((path + 'segmented_suture_l.jpg'), segmented_results_sizeBack_l)
        #cv2.imwrite((path + 'segmented_suture_r.jpg'), segmented_results_sizeBack_r)
    c_t = c_t + 400
# Release everything if job is finishedq
cap_1.release()
#cap_2.release()
cv2.destroyAllWindows()
# In[]
# for single camera

model_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Deep_learning_based_methods/unet_keras-master/'
model = vgg10_unet(input_shape=(512,512,3))
#model.load_weights(model_path + 'unet_gray_image_01_180_dark_light_color.h5')
model.load_weights(model_path + 'unet_color_image_1to40_fine_tune.h5')

path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Paper_preparation/video_data/'
# -------- from USB camera -----------

#cap_1 = cv2.VideoCapture(0)
#cap_2 = cv2.VideoCapture(1)

cap_1 = cv2.VideoCapture(path + 'Endoscopic_View_March28_Trim_2.mp4')
# Define the codec and create VideoWriter object
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output4.avi',fourcc, 20.0, (640,480))

erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
#erode_kernel = np.ones((8, 8),np.uint8)

dilate_kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (1, 1))


c_t = 1
while(cap_1.isOpened()):
    cap_1.set(cv2.CAP_PROP_POS_MSEC, c_t) 
    ret_1, original_frame_1 = cap_1.read()
    
    L_O = np.shape(original_frame_1)[1]
    H_O = np.shape(original_frame_1)[0]
    
 
    ret_1, frame_1 = cap_1.read()
    #ret_2, frame_2 = cap_2.read()
    #segmented_results_1 = 255 * np.ones((512, 512), np.uint8)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    if ((ret_1==True)): #& (ret_2==True)):        
        segmented_results_L = DL_based_segmentation(frame_1)
        
        
        segmented_results_sizeBack_L = cv2.resize(segmented_results_L, (L_O, H_O))
        
        #segmented_results_sizeBack_r = cv2.resize(segmented_results_2, (640, 480))
        [a,b] = np.where(segmented_results_sizeBack_L < 100)
        red = [0,0,255]

        for x, y in zip(a, b):
            frame_1[x, y] = red  
          
        #numpy_vertical_concat = np.concatenate((frame_1, segmented_results_sizeBack_l_3_channel), axis=1)
        segmented_results_sizeBack_L = cv2.cvtColor(segmented_results_sizeBack_L, cv2.COLOR_GRAY2BGR)   
        numpy_vertical_concat = np.concatenate((original_frame_1, frame_1), axis=1 )
        numpy_vertical_concat = cv2.resize(numpy_vertical_concat, (int(L_O/2), int(H_O/4)))
        cv2.imshow('Suture Segmentation ', numpy_vertical_concat)
        
        #cv2.imwrite(path + 'trim3/Original_imgs/img_%d.jpg'%c_t, original_frame_1)
        #cv2.imwrite(path + 'trim3/Mask_imgs/img_%d.jpg'%c_t, segmented_results_sizeBack_L)
        #cv2.imwrite(path + 'trim3/Segmented_imgs/img_%d.jpg'%c_t, frame_1)
        
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

    else:
        print('Missing_frame')
        #cv2.imwrite((path + 'segmented_suture_l.jpg'), segmented_results_sizeBack_l)
        #cv2.imwrite((path + 'segmented_suture_r.jpg'), segmented_results_sizeBack_r)
    c_t = c_t + 200
    #break
# Release everything if job is finishedq
cap_1.release()
#cap_2.release()
cv2.destroyAllWindows()