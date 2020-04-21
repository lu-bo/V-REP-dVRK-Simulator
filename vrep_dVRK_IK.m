
%%
function vrep_dVRK_IK(Target_position, Target_orientation)
    % initialize the pose of dVRK 

    % list of the joint:
    % 1. J1_PSM1
    % 2. J2_PSM1
    % 3. J3_PSM1
    % 4. J1_TOOL1
    % 5. J2_TOOL1
    % 6. J3_dx_TOOL1 & J2_sx_TOOL1

    %clear all; clc;
    % Set intial angle and positon to these joints
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    Joint_list = {'J1_PSM1', 'J2_PSM1', 'J3_PSM1', 'J1_TOOL1', 'J2_TOOL1', 'J3_dx_TOOL1', 'J3_sx_TOOL1'};

    Joint_Handle = [];
    %initial_prismatic_length = 0.1;
    for i = 1 : 7

        [~, Joint_Handle(i, 1)] = vrep.simxGetObjectHandle(clientID, Joint_list{i}, vrep.simx_opmode_blocking);

        [~, Joint_Position(i, 1)] = vrep.simxGetJointPosition(clientID, Joint_Handle(i, 1), vrep.simx_opmode_blocking);

%         if (i ~= 3)
%             [~] = vrep.simxSetJointPosition(clientID, Joint_Handle(i, 1), 0, vrep.simx_opmode_blocking);
%         else
%             [~] = vrep.simxSetJointPosition(clientID, Joint_Handle(i, 1), initial_prismatic_length, vrep.simx_opmode_blocking);
%         end
    end

    %%
    [~, Relatieve_Position]  = vrep.simxGetObjectPosition(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);

    [~, EulerAngles] = vrep.simxGetObjectOrientation(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
    %[~, Position__leftVision_RCM]    = vrep.simxGetObjectPosition(clientID, Joint_Handle(7), Joint_Handle(1), vrep.simx_opmode_blocking);

    R_X = [1 0 0; ...
           0 cos(EulerAngles(1)) -sin(EulerAngles(1)); ...
           0 sin(EulerAngles(1)) cos(EulerAngles(1))];

    R_Y = [cos(EulerAngles(2)) 0 sin(EulerAngles(2)); ...
           0 1 0; ...
           -sin(EulerAngles(2)) 0 cos(EulerAngles(2))];

    R_Z = [cos(EulerAngles(3)) -sin(EulerAngles(3)) 0; ...
           sin(EulerAngles(3)) cos(EulerAngles(3)) 0; ...
           0 0 1];

    RotationMatrix = R_X*R_Y*R_Z;

    Initial_config = [RotationMatrix, Relatieve_Position'; 0 0 0 1];

    [~, init_q(1,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(1), vrep.simx_opmode_blocking);
    [~, init_q(2,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(2), vrep.simx_opmode_blocking);
    [~, init_q(3,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(3), vrep.simx_opmode_blocking);
    [~, init_q(4,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(4), vrep.simx_opmode_blocking);
     ;
    [~, init_q(6,1)]=vrep.simxGetJointPosition(clientID, Joint_Handle(6), vrep.simx_opmode_blocking);
    
    
    %[~, qq]=vrep.simxGetObjectOrientation(clientID, Joint_Handle(6), Joint_Handle(5), vrep.simx_opmode_blocking);
    
    
    
    %PSM_q = [       0;        0;       0;       0;        0;       0];
    PSM_q = init_q;
    PSM_M = [0.9984    0.0402    0.0402    0.0000
       -0.0402    0.9992   -0.0003    0.0934
       -0.0402   -0.0013    0.9992    0.0000
             0         0         0    1.0000];  
    PSM_S = [0,  1,  0,  0,         1,        0       
             0,  0,  0,  1,         0,        0       
             1,  0,  0,  0,         0,        1     
             0,  0,  0,  0,         0,   0.0934
             0,  0,  1,  0,         0,        0       
             0,  0,  0,  0,   -0.0844,        0];

    %%

    % set eulerangles for the end_effector
    Steps = 30;
    q = [];  
    Current_EulerAngles = []; Current_Position = [];
    Increase_Eulerangle = []; Increase_translation = [];


    %Target_orientation = [2*pi/4 pi/4 1*pi/4]; % angle value --> set the desired values

    [~, Current_EulerAngles] = vrep.simxGetObjectOrientation(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
    Increase_Eulerangle = (Target_orientation - Current_EulerAngles) / Steps; % angle value

    % -------------------------------------------------------------------------
    %Target_position = [-0.1 0.15 0.15];

    [~, Current_Position]  = vrep.simxGetObjectPosition(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
    Increase_translation = (Target_position - Current_Position) / Steps;


    for i = 2 : Steps % step number --> pay attention!
        %pause(0.01);
        P_Current_EulerAngles(i, :) = Current_EulerAngles(i - 1, :) + Increase_Eulerangle(i - 1, :);

        IK_EulerAngles = P_Current_EulerAngles(i, :);

        IK_R_X = [1 0 0; ...
               0 cos(IK_EulerAngles(1)) -sin(IK_EulerAngles(1)); ...
               0 sin(IK_EulerAngles(1)) cos(IK_EulerAngles(1))];

        IK_R_Y = [cos(IK_EulerAngles(2)) 0 sin(IK_EulerAngles(2)); ...
               0 1 0; ...
               -sin(IK_EulerAngles(2)) 0 cos(IK_EulerAngles(2))];

        IK_R_Z = [cos(IK_EulerAngles(3)) -sin(IK_EulerAngles(3)) 0; ...
               sin(IK_EulerAngles(3)) cos(IK_EulerAngles(3)) 0; ...
               0 0 1];

        IK_RotationMatrix = IK_R_X * IK_R_Y * IK_R_Z;


        P_Current_Position(i, :) = Current_Position(i - 1, :) + Increase_translation(i - 1, :);
        IK_translation = P_Current_Position(i, :)';

        IK_transformation = [IK_RotationMatrix IK_translation; 0 0 0 1];

        % skew
        ep_esp = 1e-5;
        eo_esp = 1e-3;

        % PSM parameters modify
        PSM_joint_size = 6;
        PSM_joint_type = [       0;        0;       1;       0;        0;       0];
        PSM_a          = [       0;        0;       0;       0;   -0.009;       0];
        PSM_alpha      = [    pi/2;    -pi/2;       0;  1.5708;   1.5708;   3.1416];
        PSM_d_base     = [       0;        0; -0.3677;  0.4521;  -0.0004;       0];
        PSM_q_base     = [    -pi/2;       0;  1.6110;  1.6110;   1.5306;   3.0611];

        if (i > 2)
           PSM_q = q(:, i - 1); 
        end

        PSM = robot(PSM_joint_size, PSM_joint_type, PSM_a, PSM_alpha, PSM_d_base, PSM_q_base);

        q(:,i) = PSM.IKinSpace(PSM_S, PSM_M, IK_transformation, PSM_q, eo_esp, ep_esp);

        % drive the joints in PSM 
        vrep.simxSetJointPosition(clientID, Joint_Handle(1, 1), q(1,i), vrep.simx_opmode_blocking); % revolute joint
        vrep.simxSetJointPosition(clientID, Joint_Handle(2, 1), q(2,i), vrep.simx_opmode_blocking); % revolute joint
        vrep.simxSetJointPosition(clientID, Joint_Handle(3, 1), q(3,i)+0.1, vrep.simx_opmode_blocking); %prismatic joint
        vrep.simxSetJointPosition(clientID, Joint_Handle(4, 1), q(4,i), vrep.simx_opmode_blocking); % revolute joint
        vrep.simxSetJointPosition(clientID, Joint_Handle(5, 1), q(5,i), vrep.simx_opmode_blocking); % revolute joint
        vrep.simxSetObjectOrientation(clientID, Joint_Handle(6, 1), Joint_Handle(6, 1), [0, 0, q(6,i) - q(6,i-1)], vrep.simx_opmode_blocking); % revolute joint
        vrep.simxSetObjectOrientation(clientID, Joint_Handle(7, 1), Joint_Handle(7, 1), [0, 0,-(q(6,i) - q(6,i-1))], vrep.simx_opmode_blocking); % revolute joint

        % ------------- real pose && pose error between planned pose and real pose in the current loop -------------
        [~, now_EulerAngles] = vrep.simxGetObjectOrientation(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
        [~, now_Position]  = vrep.simxGetObjectPosition(clientID, Joint_Handle(6), Joint_Handle(1), vrep.simx_opmode_blocking);
        error_im_eular_angle(i,:) = now_EulerAngles-P_Current_EulerAngles(i, :);
        error_im_position(i,:) = now_Position-P_Current_Position(i, :);

        % put current pose into the pose matrix
        Current_EulerAngles(i, :) = now_EulerAngles;
        Current_Position(i, :) = now_Position;

        % compute step incremental values for pose
%         if ((Steps - 1 - i) == 0)
%             Increase_Eulerangle(i, :) = (Target_orientation - now_EulerAngles);
%             Increase_translation(i, :)= (Target_position - now_Position);
%         end
%         if ((Steps - 1 - i) < 0)
%             break;
%         end
%         if ((Steps - 1 - i) > 0)
%             Increase_Eulerangle(i, :) = (Target_orientation - now_EulerAngles) / (Steps - 1 - i);
%             Increase_translation(i, :)= (Target_position - now_Position) / (Steps - 1 - i);
%         end
    end
    %%
    %[~, J3_sx_TOOL1_EulerAngles] = vrep.simxGetObjectOrientation(clientID, J3_sx_TOOL1, J2_TOOL1, vrep.simx_opmode_blocking);

    %vrep.simxSetJointPosition(clientID, Joint_Handle(5, 1), 0, vrep.simx_opmode_blocking); 
    %vrep.simxSetObjectOrientation(clientID, Joint_Handle(6, 1), Joint_Handle(6, 1), [-5,0,0]*pi/180, vrep.simx_opmode_blocking);
    %vrep.simxSetObjectOrientation(clientID, Joint_Handle(7, 1), Joint_Handle(7, 1), [0,0,-pi/2], vrep.simx_opmode_blocking);

end

