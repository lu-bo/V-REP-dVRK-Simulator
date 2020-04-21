%% basic parameters
ep_esp = 1e-5;
eo_esp = 1e-3;
iterate_times = 10;

%% generation trajectory
point_size = 50;
radius = 0.025;
Pt = zeros(3, point_size);
for i = 1:point_size
    Pt(1, i) = radius * cos(i * 2.0 * pi / point_size) - radius;
    Pt(2, i) = radius * sin(i * 2.0 * pi / point_size);
    Pt(3, i) = 0;
end

%% MTM parameters
MTM_joint_size = 7;
MTM_joint_type = [     0;     0;       0;       0;     0;     0;     0];
MTM_a          = [     0;     0; -0.2794; -0.3645;     0;     0;     0];
MTM_alpha      = [     0; -pi/2;       0;    pi/2; -pi/2;  pi/2;  pi/2];
MTM_d_base     = [     0;     0;       0;  0.1506;     0;     0;     0];
MTM_q_base     = [  pi/2; -pi/2;    pi/2;       0;     0;  pi/2;  pi/2];
MTM_q          = [     0;     0;       0;       0;     0;     0;     0];

%% 
MTM = robot(MTM_joint_size, MTM_joint_type, MTM_a, MTM_alpha, MTM_d_base, MTM_q_base);
%MTM.set_feo({@MTM.feo_quaternion}, ep_esp, eo_esp, iterate_times);
T0 = MTM.fk(MTM_q);
for i = 1:point_size
    MTM_q = MTM.ik(T0(1:3, 1:3), Pt(:, i) + T0(1:3, 4), MTM_q);
    hold off;
    figure(1);
    MTM.draw(MTM_q);
    drawnow;
    hold on;
    figure(1);
    plot3(Pt(1, 1:i) + T0(1, 4), Pt(2, 1:i) + T0(2, 4), Pt(3, 1:i) + T0(3, 4));
    hold on;
end

%% PSM parameters
PSM_joint_size = 6;
PSM_joint_type = [       0;        0;       1;       0;        0;       0];
PSM_a          = [       0;        0;       0;       0;   0.0091;       0]; % Offset --> Z axis
PSM_alpha      = [   -pi/2;     pi/2;       0;   -pi/2;    -pi/2;       0]; % X rotate
PSM_d_base     = [       0;        0;       0;    0.16;        0;       0]; % Offset --> X axis
PSM_q_base     = [       0;    -pi/2;       0;       0;    -pi/2;   -pi/2]; % Z rotate

% set orientation wrt. RCM coordinate
PSM_q          = [       0;        0;       0;       0;        0;       0];

%% 
PSM = robot(PSM_joint_size, PSM_joint_type, PSM_a, PSM_alpha, PSM_d_base, PSM_q_base);
%PSM.set_feo({@PSM.feo_quaternion}, ep_esp, eo_esp, iterate_times);
figure(2);
T0 = PSM.Sfk(PSM_q);
for i = 1:point_size
    PSM_q = PSM.ik(T0(1:3, 1:3), Pt(:, i) + T0(1:3, 4), PSM_q);
    hold off;
    figure(2);
    PSM.draw(PSM_q);
    drawnow;
    hold on;
    figure(2);
    plot3(Pt(1, 1:i) + T0(1, 4), Pt(2, 1:i) + T0(2, 4), Pt(3, 1:i) + T0(3, 4));
    hold on;
end