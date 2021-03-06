% --- initial pose definition ---
%%
%PSM_q = init_q;
clear; clc;

PSM_M = [0.9984    0.0402    0.0402    0.0000
        -0.0402    0.9992   -0.0003    0.0000
        -0.0402   -0.0013    0.9992    0.0000
         0         0         0         1.0000]; 
     
PSM_S = [0,  1,  0,  0,         1,        0       
         0,  0,  0,  1,         0,        0       
         1,  0,  0,  0,         0,        1     
         0,  0,  0,  0,         0,        0
         0,  0,  1,  0,         0,        0       
         0,  0,  0,  0,   -0.0844,        0];

motion_range = [-80*pi/180 80*pi/180            % -------- RCM point (reference coordinate) --------
                -80*pi/180 80*pi/180            % -------- at RCM point     --------
                0 0.15                          % -------- prismatic joint  --------
                -90*pi/180 90*pi/180          % -------- rotate shaft     --------
                -80*pi/180 80*pi/180            % -------- gripper rotation --------
                -70*pi/180 70*pi/180            % -------- grasper rotation --------
];     


PSM_joint_size = 6;
PSM_joint_type = [       0;        0;       1;       0;        0;       0];
PSM_a          = [       0;        0;       0;       0;   -0.009;       0];
PSM_alpha      = [    pi/2;    -pi/2;       0;  1.5708;   1.5708;   3.1416];
PSM_d_base     = [       0;        0; -0.3677;  0.4521;  -0.0004;       0];
PSM_q_base     = [    -pi/2;       0;  1.6110;  1.6110;   1.5306;   3.0611];


PSM = robot(PSM_joint_size, PSM_joint_type, PSM_a, PSM_alpha, PSM_d_base, PSM_q_base);

interval = 2;          
workSpace_N = 1;
point_size = 10^5;


q_cur = [rand(1, point_size) * (motion_range(1, 2) - motion_range(1, 1)) + motion_range(1, 1); ...
         rand(1, point_size) * (motion_range(2, 2) - motion_range(2, 1)) + motion_range(2, 1); ...
         rand(1, point_size) * (motion_range(3, 2) - motion_range(3, 1)) + motion_range(3, 1); ...
         rand(1, point_size) * (motion_range(4, 2) - motion_range(4, 1)) + motion_range(4, 1); ...
         rand(1, point_size) * (motion_range(5, 2) - motion_range(5, 1)) + motion_range(5, 1); ...
         rand(1, point_size) * (motion_range(6, 2) - motion_range(6, 1)) + motion_range(6, 1)];


for workSpace_N = 1: point_size

    PSM_q(1,1) = q_cur(1, workSpace_N);
    PSM_q(2,1) = q_cur(2, workSpace_N);
    PSM_q(3,1) = 0.1;
    PSM_q(4,1) = q_cur(4, workSpace_N);
    PSM_q(5,1) = q_cur(5, workSpace_N);
    PSM_q(6,1) = q_cur(6, workSpace_N);

    q_record(workSpace_N, :) = [PSM_q(1,1), PSM_q(2,1), PSM_q(3,1), ...
                                PSM_q(4,1), PSM_q(5,1), PSM_q(6,1)];
    T(:, :, workSpace_N) = PSM.FKinSpace(PSM_M, PSM_S, PSM_q);
    J(:, :, workSpace_N) = PSM.JacobianSpace(PSM_S, PSM_q);
    end_effector_xyz(workSpace_N, :) = T(1 : 3, 4, workSpace_N);

    H(:, :, workSpace_N) = J(:, :, workSpace_N)*(J(:, :, workSpace_N)');


    Manip_I(workSpace_N, 1) = sqrt(det(H(:, :, workSpace_N)));

    workSpace_N= workSpace_N+1;
    
end
%%
figure;
max_u = max(Manip_I);

Manip_Color = Manip_I*(1/max_u);
scatter3(end_effector_xyz(:, 1), end_effector_xyz(:, 2), end_effector_xyz(:, 3), 10, Manip_I(:), 'filled');
hold on; grid on; 
xlabel('X'); ylabel('Y'); zlabel('Z');

colormap(jet);
colorbar


