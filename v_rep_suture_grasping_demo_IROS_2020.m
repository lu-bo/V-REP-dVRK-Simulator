% 2. ------------------- Approaching the suture thread --------------------


c_length = 0;
ear_length = 0.05;
for ii = 1 : size(Coo, 1)
    c_length = norm(Coo(ii + 1, :) - Coo(ii, :)) + c_length;
    
    if (c_length > ear_length)
        break
    end
end

GS = Coo(ii, :);
Target_translation = norm(Coo(ii, :)); % for joint --> 'J2_PSM1'
rotate_angle_wrt_J1_PSM1 = pi / 2 + atan(Coo(ii, 2) / Coo(ii, 1));
rotate_angle_wrt_J2_PSM1 = pi / 2 - acos(Coo(ii, 3) / Target_translation);



%%
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

[~,J1_PSM1] = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);
[~,J2_PSM1] = vrep.simxGetObjectHandle(clientID, 'J2_PSM1', vrep.simx_opmode_blocking);
[~,J3_PSM1] = vrep.simxGetObjectHandle(clientID, 'J3_PSM1', vrep.simx_opmode_blocking);

[~,J1_PSM2] = vrep.simxGetObjectHandle(clientID, 'J1_PSM2', vrep.simx_opmode_blocking);
[~,J2_PSM2] = vrep.simxGetObjectHandle(clientID, 'J2_PSM2', vrep.simx_opmode_blocking);
[~,J3_PSM2] = vrep.simxGetObjectHandle(clientID, 'J3_PSM2', vrep.simx_opmode_blocking);

% initial pose of end effector
vrep.simxSetJointPosition(clientID, J3_PSM1, 0.12, vrep.simx_opmode_blocking);
vrep.simxSetJointPosition(clientID, J1_PSM1, 60 * pi / 180, vrep.simx_opmode_blocking);
vrep.simxSetJointPosition(clientID, J2_PSM1, -15 * pi / 180, vrep.simx_opmode_blocking);

vrep.simxSetJointPosition(clientID, J3_PSM2, 0.08, vrep.simx_opmode_blocking);
vrep.simxSetJointPosition(clientID, J1_PSM2, -50 * pi / 180, vrep.simx_opmode_blocking);
vrep.simxSetJointPosition(clientID, J2_PSM2, 20 * pi / 180, vrep.simx_opmode_blocking);


[~, Initial_translation] = vrep.simxGetJointPosition(clientID, J3_PSM1, vrep.simx_opmode_blocking);
[~, Initial_rotation_1]  = vrep.simxGetJointPosition(clientID, J1_PSM1, vrep.simx_opmode_blocking);
[~, Initial_rotation_2]  = vrep.simxGetJointPosition(clientID, J2_PSM1, vrep.simx_opmode_blocking);


%%
%pause(5);
T_J2_TOOL1_to_grasper = 15.6/1000;
%dVRK_grasper('close');

Steps = 10;
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

%%
% error calculation
[~,gripper_tip_ref] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);
[~,object_1] = vrep.simxGetObjectHandle(clientID, 'Spherical_joint38', vrep.simx_opmode_blocking);
[~,object_2] = vrep.simxGetObjectHandle(clientID, 'Spherical_joint50', vrep.simx_opmode_blocking);


[~, Position_1]= vrep.simxGetObjectPosition(clientID, object_1 , gripper_tip_ref, vrep.simx_opmode_blocking);
[~, Position_2]= vrep.simxGetObjectPosition(clientID, object_2 , gripper_tip_ref, vrep.simx_opmode_blocking);
norm(Position_1)
norm(Position_2)
