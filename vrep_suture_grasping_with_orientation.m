Path='simulated_img/';
FolderName=datestr(now,30);   
mkdir(Path,FolderName);
Now_Path = [Path, FolderName, '/'];
%%
% ------- stereo camera cali
clearvars -except stereoParams; clc; close all;
PATH_left_images  = 'C:\Users\Bo\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\virtual_chess_board_calibration\camera_1_raw\';
PATH_right_images = 'C:\Users\Bo\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\virtual_chess_board_calibration\camera_2_raw\';
chess_board_size = 5;
stereoParams = stereo_camera_calibration(PATH_left_images, PATH_right_images, chess_board_size); disp(stereoParams.MeanReprojectionError)

global stereoParams; clearvars -except stereoParams; clc; close all;

%%
% ------- initial position setups of dVRK system and the suture 
setup_initialization;
vrep_dVRK.intial_config_setup
%%
image_path = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\';

[left_image, right_image] = dVRK_virtual_sensor.get_stereo_vision;
%left_image  = undistortImage(left_image,  stereoParams.CameraParameters1);
%right_image = undistortImage(right_image, stereoParams.CameraParameters2);

%[left_image, right_image] = rectifyStereoImages(left_image, right_image, stereoParams, 'OutputView', 'full');
%imwrite(left_image, [Now_Path, 'left_image.jpg']); imwrite(right_image, [Now_Path, 'right_image.jpg']);
imwrite(left_image, [image_path, 'left_image.jpg']); imwrite(right_image, [image_path, 'right_image.jpg']);
%%

[all_points_set_1, tip_1] = DL_based_intersected_suture_thread_detection_with_clicks(left_image, 'segmented_suture_l.jpg');
% figure; imshow(left_image); hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r');
[all_points_set_2, tip_2] = DL_based_intersected_suture_thread_detection_with_clicks(right_image, 'segmented_suture_r.jpg');
% figure; imshow(right_image); hold on; scatter(all_points_set_2(:, 1), all_points_set_2(:, 2), 5, 'r');

%%
multi_view_num = 1;
xyzsg = suture_thread_3D_computation_optimization(all_points_set_1, all_points_set_2, multi_view_num, 'vrep');
%%
figure; 
intensity = [0.1: 0.001: 0.1 + 0.001*(size(xyzsg, 1)-1)];
scatter3(xyzsg(:, 1), xyzsg(:, 2), xyzsg(:, 3), 10, intensity(:), 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
colormap(jet); colorbar;
%%

figure(5); 
scatter(xyzsg(:, 1), xyzsg(:, 2),10, intensity(:), 'filled')
xlabel('X'); ylabel('Y');
colormap(jet); colorbar;
grid on;

figure(6); 
scatter(xyzsg(:, 1), xyzsg(:, 3),10, intensity(:), 'filled')
xlabel('X'); ylabel('Z');
colormap(jet); colorbar;
grid on;

%%

accum_len = 0;
set_threshold = 0.00005;
for node_i = 2 : size(xyzsg, 1)
    accum_len = norm(xyzsg(node_i, :) - xyzsg(node_i - 1, :)) +accum_len;
    if (accum_len > set_threshold)
        break;
    end
end

vv3 = xyzsg(node_i, :) - xyzsg(node_i - 1, :); scale_vv3 = 1 / norm(vv3); v3_p = scale_vv3 * vv3; v3_n = -v3_p;
vv2 = [0 0 0]; vv2(2) = 0.1; vv2(1) = (-v3_p(2)/v3_p(1))*vv2(2); scale_vv2 = 1 / norm(vv2); v2_p = scale_vv2 * vv2; v2_n = -v2_p;

base1_v1 = v2_n; % y axis
base2_v1 = v3_p; % z axis 
v1 = [base1_v1(2)*base2_v1(3)-base1_v1(3)*base2_v1(2), ...
      base1_v1(3)*base2_v1(1)-base1_v1(1)*base2_v1(3), ...
      base1_v1(1)*base2_v1(2)-base1_v1(2)*base2_v1(1)]; % direction of v1 is determined by two bases. 
rotm_21 = [v1', base1_v1', base2_v1'];



%%
[rotm_32, ~, CoCo] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', xyzsg);
rotation_31 = rotm_32*rotm_21;

%%
eulXYZ_31 = rotm2eul(rotation_31,'XYZ');

vrep_dVRK.vrep_dVRK_IK(CoCo(node_i, :), eulXYZ_31)
%vrep_dVRK.vrep_grasper_open
%vrep_dVRK.vrep_grasper_close
%vrep_dVRK.intial_config_setup

%%
% pose validation
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
%if (clientID>-1)

[~, Current_system_handle] = vrep.simxGetObjectHandle(clientID, 'J3_dx_TOOL1', vrep.simx_opmode_blocking);
[~, Target_system_handle]  = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);
[~, EulerAngles] = vrep.simxGetObjectOrientation(clientID, Current_system_handle, Target_system_handle, vrep.simx_opmode_blocking);
[~, Position]    = vrep.simxGetObjectPosition(clientID, Current_system_handle, Target_system_handle, vrep.simx_opmode_blocking);
        
EulerAngles*180/pi
Position



