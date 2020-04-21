%%
PATH_camera_1 = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\virtual_chess_board_calibration\camera_1\';
PATH_camera_2 = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\virtual_chess_board_calibration\camera_2\';
for img_cali = 1 : 10
    pause;
    [left_image, right_image] = dVRK_virtual_sensor.get_stereo_vision;
    
    imwrite(left_image, [PATH_camera_1 num2str(img_cali) '.jpg'])
    imwrite(left_image, [PATH_camera_2 num2str(img_cali) '.jpg'])
    
end