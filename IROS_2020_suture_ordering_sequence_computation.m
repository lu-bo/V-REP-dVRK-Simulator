
clearvars -except stereoParams Hcam2marker_; clc; close all;
image_path = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\';
%---------------------------------------------------------------


%[left_image, right_image] = dVRK_virtual_sensor.get_stereo_vision;
left_image  = imread([image_path, 'left_image.jpg'] );
right_image = imread([image_path, 'right_image.jpg']);


seg_l = imread([image_path, 'segmented_suture_l.jpg'] );
seg_r = imread([image_path, 'segmented_suture_r.jpg'] );

left_image = imgaussfilt(left_image,1); right_image = imgaussfilt(right_image,1); pause(0.5);

%%
%---------------------------------------------------------------
[all_points_set_1, tip_1] = DL_based_intersected_suture_thread_detection_with_clicks(left_image, 'segmented_suture_l.jpg');  
[all_points_set_2, tip_2] = DL_based_intersected_suture_thread_detection_with_clicks(right_image, 'segmented_suture_r.jpg');


%
figure(1); imshow(left_image); hold on; 
dc1=hsv(length(all_points_set_1));

for i = 1 : length(all_points_set_1) - 1
    line([all_points_set_1(i, 1) all_points_set_1(i + 1 , 1)], [all_points_set_1(i, 2) all_points_set_1(i + 1, 2)] , 'Color', dc1(i, :), 'linewidth', 8);
end

figure(2); imshow(right_image); hold on; 
dc2=hsv(length(all_points_set_2));
for i = 1 : length(all_points_set_2) - 1
    line([all_points_set_2(i, 1) all_points_set_2(i + 1 , 1)], [all_points_set_2(i, 2) all_points_set_2(i + 1, 2)] , 'Color', dc2(i, :), 'linewidth', 8);
end
