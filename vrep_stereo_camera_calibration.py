import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
vrep_calibration_image_path = 'C:/Users/User/OneDrive - The Hong Kong Polytechnic University/PhD Studies/Semester 4/3D Reconstruction/2017.03.20/v-repApi/virtual_chess_board_calibration/'

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5*9,3), np.float32)
objp[:,:2] = np.mgrid[0:25:5 , 0:45:5].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints_left = [] # 3d point in real world space
imgpoints_left = [] # 2d points in image plane.
objpoints_right = [] # 3d point in real world space
imgpoints_right = [] # 2d points in image plane.


images_left = glob.glob(vrep_calibration_image_path  + '/camera_1_raw/*.jpg')
images_right = glob.glob(vrep_calibration_image_path + '/camera_2_raw/*.jpg')

summarized_images = dict(zip(images_left, images_right))

i = 0
ii = 0

for fname_left, fname_right in summarized_images.items():
    img_left = cv2.imread(fname_left)
    gray_left = cv2.cvtColor(img_left,cv2.COLOR_BGR2GRAY)
    img_right = cv2.imread(fname_right)
    gray_right = cv2.cvtColor(img_right,cv2.COLOR_BGR2GRAY)
    
    i = i + 1
    # Find the chess board corners
    ret_left, corners_left = cv2.findChessboardCorners(gray_left, (5,9),None)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, (5,9),None)
    
    cv2.waitKey(0) 
    # If found, add object points, image points (after refining them)
    if ret_left and ret_right:
        objpoints_left.append(objp)
        objpoints_right.append(objp)

        corners2_left = cv2.cornerSubPix(gray_left,corners_left,(11,11),(-1,-1),criteria)
        imgpoints_left.append(corners2_left)
        corners2_right = cv2.cornerSubPix(gray_right,corners_right,(11,11),(-1,-1),criteria)
        imgpoints_right.append(corners2_right)
        # Draw and display the corners
        img_left = cv2.drawChessboardCorners(img_left, (5,9), corners2_left, ret_left)
        img_right = cv2.drawChessboardCorners(img_right, (5,9), corners2_right, ret_right)
        
        ii = ii + 1
        
        both_frames = np.concatenate((img_left, img_right), axis = 1)
        cv2.imshow('left image',both_frames)
        

cv2.destroyAllWindows()

# In[]

ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(objpoints_left, imgpoints_left, gray_left.shape[::-1], None, None)
ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(objpoints_right, imgpoints_right, gray_right.shape[::-1], None, None)

'''
print ("Ret_left:",ret_left)
print ("Mtx_left:",mtx_left," ----------------------------------> [",mtx_left.shape,"]")
print ("Dist:",dist_left," ----------> [",dist_left.shape,"]")
print ("rvecs:",rvecs_left," --------------------------------------------------------> [",rvecs_left[0].shape,"]")
print ("tvecs:",tvecs_left," -------------------------------------------------------> [",tvecs_left[0].shape,"]")
'''
# In[]
#cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, gray_left.shape[::-1], mtx_left, mtx_right, dist_left, dist_right)  
                    #cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), 
                    #CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST)
#flags = (cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6)
#stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)

stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
stereocalib_flags = cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_SAME_FOCAL_LENGTH | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5

#flags = (cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_FIX_FOCAL_LENGTH |
#         cv2.CALIB_FIX_INTRINSIC | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 |
#         cv2.CALIB_FIX_K6)

retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(objpoints_left,  imgpoints_left, imgpoints_right, 
                                                   mtx_left,dist_left,mtx_right,dist_right, 
                                                   (gray_left.shape[::-1]), 
                                                   #criteria = stereocalib_criteria,
                                                   #flags=flags)
                                                   #flags=cv2.CALIB_FIX_INTRINSIC
                                                   criteria = stereocalib_criteria, flags = stereocalib_flags)

# In[]
R1 = np.zeros((3, 3), np.float32)
R2 = np.zeros((3, 3), np.float32)
P1 = np.zeros((3, 4), np.float32)
P2 = np.zeros((3, 4), np.float32)

R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(mtx_left, dist_left, 
                                                  mtx_right, dist_right, 
                                                  (gray_left.shape[1], gray_left.shape[0]),
                                                  R, T, 
                                                  flags=cv2.CALIB_ZERO_DISPARITY, 
                                                  #None, None, None, None, None,
                                                  alpha = 1)

map1_x,map1_y=cv2.initUndistortRectifyMap(mtx_left, dist_left, R1, P1, (gray_left.shape[1], gray_left.shape[0]), cv2.CV_32FC1)
map2_x,map2_y=cv2.initUndistortRectifyMap(mtx_right, dist_right, R2, P2, (gray_left.shape[1], gray_left.shape[0]), cv2.CV_32FC1)


im_left  = cv2.imread(vrep_calibration_image_path + '/camera_1_raw/left_image_4.jpg')
im_right = cv2.imread(vrep_calibration_image_path + '/camera_2_raw/right_image_4.jpg')

'''
h_1,  w_1 = im_left.shape[:2]
newcameramtx_1, roi_1 = cv2.getOptimalNewCameraMatrix(mtx_left, dist_left, (w_1, h_1 ), 1, (w_1, h_1))
# undistort
dst_1 = cv2.undistort(im_left, mtx_left, dist_left, None, newcameramtx_1)

# crop the image
#x_1,y_1,w_1,h_1 = roi_1
#dst_1 = dst_1[y_1:y_1+h_1, x_1:x_1+w_1]
cv2.imshow('calibresult.png',dst_1)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''
## In[]

im_left_remapped=cv2.remap(im_left,map1_x,map1_y,cv2.INTER_CUBIC)
im_right_remapped=cv2.remap(im_right,map2_x,map2_y,cv2.INTER_CUBIC)
two_rectifications = np.concatenate((im_left_remapped, im_right_remapped), axis = 1)
cv2.imshow('two rectified images', two_rectifications)
cv2.waitKey(0)
cv2.destroyAllWindows()

# In[]

sift = cv2.xfeatures2d.SIFT_create()
#matcher = cv2.TYPE_create("BruteForce")


#pairs = dict(zip(im_left_remapped, im_right_remapped))

im_left_remapped = cv2.cvtColor(im_left_remapped,cv2.COLOR_BGR2GRAY)
im_right_remapped = cv2.cvtColor(im_right_remapped,cv2.COLOR_BGR2GRAY)

im_left_remapped_f  = np.float32(im_left_remapped)
im_right_remapped_f = np.float32(im_right_remapped)

left_kp  = cv2.cornerHarris(im_left_remapped_f, 2,3,0.04) 
right_kp = cv2.cornerHarris(im_right_remapped_f,2,3,0.04)  

kp_left  = sift.detect(im_left_remapped, None)
kp_right = sift.detect(im_left_remapped,None)

im_left_remapped  = cv2.drawKeypoints(im_left_remapped,  kp_left,  None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
im_right_remapped = cv2.drawKeypoints(im_left_remapped, kp_right, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

two_rectifications_circles = np.concatenate((im_left_remapped, im_right_remapped), axis = 1)
cv2.imshow('two rectified images', two_rectifications_circles)
cv2.waitKey(0)
cv2.destroyAllWindows()

# In[]
l_kp, l_d = extractor.compute(im_left_remapped, left_kp)
r_kp, r_d = extractor.compute(im_right_remapped, right_kp)
matches = matcher.match(l_d, r_d)
    #sel_matches = [m for m in matches if abs(l_kp[m.queryIdx].pt[1] - r_kp[m.trainIdx].pt[1]) &lt; 3]







