%%

%RT = dVRK_virtual_sensor.get_transformation_matrix_object1_to_object2('J2_TOOL1', 'Vision_sensor_left');


for i = 1 : size(xyzsg, 1)
moving_vector = xyzsg(i, :) * 0.001; % - [RT(1, 4), RT(2, 4), RT(3, 4)] 
%
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

[~, objectHandle_1] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);

[~, objectHandle_2] = vrep.simxGetObjectHandle(clientID, 'Vision_sensor_left', vrep.simx_opmode_blocking);

[~] = vrep.simxSetObjectPosition(clientID, objectHandle_1, objectHandle_2, moving_vector, vrep.simx_opmode_blocking);
vrep.delete(); % call the destructor!
pause(0.05);
end
%%

for i = 1 : size(Optimized_point_set, 1)
moving_vector = Optimized_point_set(i, :); % - [RT(1, 4), RT(2, 4), RT(3, 4)] 
%
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

[~, objectHandle_1] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);

[~, objectHandle_2] = vrep.simxGetObjectHandle(clientID, 'J1_ECM', vrep.simx_opmode_blocking);

[~] = vrep.simxSetObjectPosition(clientID, objectHandle_1, objectHandle_2, moving_vector, vrep.simx_opmode_blocking);
vrep.delete(); % call the destructor!
pause(0.01);
end

%%

for i = 1 : size(Coo, 1)
moving_vector = Coo(i, :); % - [RT(1, 4), RT(2, 4), RT(3, 4)] 
%
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

[~, objectHandle_1] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);

[~, objectHandle_2] = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);

[~] = vrep.simxSetObjectPosition(clientID, objectHandle_1, objectHandle_2, moving_vector, vrep.simx_opmode_blocking);
vrep.delete(); % call the destructor!
pause(0.01);
end

%%

% ------------ Apporach the suture and follow its shape -------------------

% 1. -------- individual translation and rotation angle calculation -------
%pause(5);
%coo = CoCo; 
coo = Coo;
T_J2_TOOL1_to_grasper = 0;
dVRK_grasper('open');

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

for i = 1 : size(coo, 1)   
    % [~, Coordinate_euler_angle] = vrep.simxGetObjectOrientation(clientID, J2_PSM1, J1_PSM1, vrep.simx_opmode_blocking);

    Target_translation(i, :) = norm(coo(i, :)); % for joint --> 'J2_PSM1'
    %[~] = vrep.simxSetJointPosition(clientID, J3_PSM1, Target_translation(i, :), vrep.simx_opmode_blocking);
    
    rotate_angle_wrt_J1_PSM1(i, :) = pi / 2 + atan(coo(i, 2) / coo(i, 1));
    %[~] = vrep.simxSetJointPosition(clientID, J1_PSM1, rotate_angle_wrt_J1_PSM1(i, :), vrep.simx_opmode_blocking);
    
    rotate_angle_wrt_J2_PSM1(i, :) = pi / 2 - acos(coo(i, 3) / Target_translation(i, :));
    %[~] = vrep.simxSetJointPosition(clientID, J2_PSM1, rotate_angle_wrt_J2_PSM1(i, :), vrep.simx_opmode_blocking);
    
end
% .........................................................................

% 2. ------------------- Approaching the suture thread --------------------
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

[~,J1_PSM1] = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);
[~,J2_PSM1] = vrep.simxGetObjectHandle(clientID, 'J2_PSM1', vrep.simx_opmode_blocking);
[~,J3_PSM1] = vrep.simxGetObjectHandle(clientID, 'J3_PSM1', vrep.simx_opmode_blocking);
[~, Initial_translation] = vrep.simxGetJointPosition(clientID, J3_PSM1, vrep.simx_opmode_blocking);
[~, Initial_rotation_1]  = vrep.simxGetJointPosition(clientID, J1_PSM1, vrep.simx_opmode_blocking);
[~, Initial_rotation_2]  = vrep.simxGetJointPosition(clientID, J2_PSM1, vrep.simx_opmode_blocking);

Steps = 80;
%D_translation = (Target_translation(1, :) - Initial_translation) / Steps;
D_translation = (Target_translation(1, :) + T_J2_TOOL1_to_grasper...
                - Initial_translation) / Steps;
D_rotation_1 = (rotate_angle_wrt_J1_PSM1(1, :) - Initial_rotation_1) / Steps;
D_rotation_2 = (rotate_angle_wrt_J2_PSM1(1, :) - Initial_rotation_2) / Steps;


for i = 1 : Steps
    C_translation = Initial_translation + (i - 0) * D_translation;
    C_rotation_1 = Initial_rotation_1 + (i - 0) * D_rotation_1;
    C_rotation_2 = Initial_rotation_2 + (i - 0) * D_rotation_2;

    [~] = vrep.simxSetJointPosition(clientID, J3_PSM1, C_translation, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J1_PSM1, C_rotation_1, vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID, J2_PSM1, C_rotation_2, vrep.simx_opmode_blocking);
    %pause(0.01);
end

% %%
% % 3. ---------------------- Follow the suture thread ----------------------
% dVRK_grasper('open');
% %%
% [~, J1_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J1_TOOL1', vrep.simx_opmode_blocking);
% [~, J2_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);
% for i = 0
%     
%     [~] = vrep.simxSetJointPosition(clientID, J1_TOOL1, pi * 1 * i / 180 , vrep.simx_opmode_blocking);
% 
% end
% %%
% for i = -1 : -1 : -90
%     
%     [~] = vrep.simxSetJointPosition(clientID, J2_TOOL1, pi * -1 * i / 180 , vrep.simx_opmode_blocking);
% 
% end
%%%
for point_seq = 2 : size(coo, 1)
      
    Steps = 2;
    D_translation = (Target_translation(point_seq, :) - Target_translation(point_seq - 1, :)) / Steps;
    D_rotation_1 = (rotate_angle_wrt_J1_PSM1(point_seq, :) - ...
                    rotate_angle_wrt_J1_PSM1(point_seq - 1, :)) / Steps;
                
    D_rotation_2 = (rotate_angle_wrt_J2_PSM1(point_seq, :) - ...
                    rotate_angle_wrt_J2_PSM1(point_seq - 1, :)) / Steps;
    
    for i = 1 : Steps
        C_translation = Target_translation(point_seq - 1, :) + T_J2_TOOL1_to_grasper...
                        + (i - 0) * D_translation;
        %C_translation = Target_translation(point_seq - 1, :) + (i - 0) * D_translation;
        C_rotation_1 = rotate_angle_wrt_J1_PSM1(point_seq - 1, :) + (i - 0) * D_rotation_1;
        C_rotation_2 = rotate_angle_wrt_J2_PSM1(point_seq - 1, :) + (i - 0) * D_rotation_2;

        [~] = vrep.simxSetJointPosition(clientID, J3_PSM1, C_translation, vrep.simx_opmode_blocking);
        [~] = vrep.simxSetJointPosition(clientID, J1_PSM1, C_rotation_1, vrep.simx_opmode_blocking);
        [~] = vrep.simxSetJointPosition(clientID, J2_PSM1, C_rotation_2, vrep.simx_opmode_blocking);
        %pause(0.01);
    end

end

%% ------------- adjust orientation -------------------

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


Direction_y_unit = [0 1 0]; % control the rotation of tool in "J1_PSM1"
Direction_z_unit = [0 0 1]; % control the translation in "J1_PSM1"
[~, ~, d_y] = dVRK_get_transformation_and_coordinates('J3_PSM1', 'J1_PSM1', Direction_y_unit);
[~, ~, d_z] = dVRK_get_transformation_and_coordinates('J3_PSM1', 'J1_PSM1', Direction_z_unit);









