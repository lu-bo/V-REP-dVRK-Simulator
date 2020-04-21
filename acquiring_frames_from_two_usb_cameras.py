# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 15:11:34 2019

@author: User
"""

import numpy as np
import cv2

video_capture_0 = cv2.VideoCapture(0)
video_capture_1 = cv2.VideoCapture(1)
calibration_path_l = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Tests_by_industrial_camera/industrial_camera_calibration/camera_l/'
calibration_path_r = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Tests_by_industrial_camera/industrial_camera_calibration/camera_r/'
#path_l = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/PhD Studies/Semester 4/3D Reconstruction/2017.03.20/v-repApi/'
#path_r = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/PhD Studies/Semester 4/3D Reconstruction/2017.03.20/v-repApi/'

#dVRK_image_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/Postdoc_works/2019-second_half/Research_works/Suturing/Deep_learning_based_methods/frames/images_dVRK/img/'

fig_num = 54

while True:
    # Capture frame-by-frame
    ret0, frame0 = video_capture_0.read()
    ret1, frame1 = video_capture_1.read()

    if (ret0 & ret1):
        # Display the resulting frame
        both_frames = np.concatenate((frame0, frame1), axis = 1)
        cv2.imshow('Stereo Camera', both_frames)


    if cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite(calibration_path_l + str(fig_num) +'.png', frame0)
        cv2.imwrite(calibration_path_r + str(fig_num) +'.png', frame1)
        fig_num = fig_num + 1
        
    if cv2.waitKey(22) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
video_capture_0.release()
video_capture_1.release()
cv2.destroyAllWindows()