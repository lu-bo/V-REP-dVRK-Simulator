%% basic parameters
ep_esp = 1e-5;
eo_esp = 1e-3;
iterate_times = 10;

%% generation trajectory
point_size = 20;
radius = 0.025;
Pt = zeros(3, point_size);
for i = 1:point_size
    Pt(1, i) = radius * cos(i * 2.0 * pi / point_size) - radius;
    Pt(2, i) = radius * sin(i * 2.0 * pi / point_size);
    Pt(3, i) = 0;
end

%% PSM parameters modify
PSM_joint_size = 6;
PSM_joint_type = [       0;        0;       1;       0;        0;       0];
PSM_a          = [       0;        0;       0;       0;        0;  0.0091];
PSM_alpha      = [    pi/2;    -pi/2;    pi/2;       0;    -pi/2;   -pi/2];
PSM_d_base     = [       0;        0; -0.1318;  0.4162;        0;       0];
PSM_q_base     = [    pi/2;    -pi/2;       0;       0;    -pi/2;   -pi/2];


PSM_q          = [       0.5;        0.8;    0.02;       0.5;        0.7;       0];
%% 
PSM = robot(PSM_joint_size, PSM_joint_type, PSM_a, PSM_alpha, PSM_d_base, PSM_q_base);
figure(1);
PSM.mdraw(PSM_q);
PSM_T_MDH = PSM.mfk(PSM_q);

%% skew
PSM_M = [ 1,  0,  0,       0; 
          0,  1,  0,  0.0934; 
          0,  0,  1,       0;
          0,  0,  0,       1];  
      
PSM_S = [0,  1,  0,  0,         1,        0       
         0,  0,  0,  1,         0,        0       
         1,  0,  0,  0,         0,        1     
         0,  0,  0,  0,         0,   0.0934 
         0,  0,  1,  0,         0,        0       
         0,  0,  0,  0,   -0.0844,        0];

PSM_T_SK = PSM.FKinSpace(PSM_M, PSM_S, PSM_q);

q = PSM.IKinSpace(PSM_S, PSM_M, PSM_T_SK, PSM_q, eo_esp, ep_esp);

%%
T0 = PSM.mfk(PSM_q);

%{
for i = 1:point_size
%     PSM_q = PSM.mik(T0(1:3, 1:3), Pt(:, i) + T0(1:3, 4), PSM_q);
    PSM_T_SK(1:3, 1:3) = T0(1:3, 1:3);
    PSM_T_SK(1:3, 4) = Pt(:, i) + T0(1:3, 4);
    PSM_q = PSM.IKinSpace(PSM_S, PSM_M, PSM_T_SK, PSM_q, eo_esp, ep_esp);
    hold off;
    figure(2);
    PSM.mdraw(PSM_q);
    drawnow;
    hold on;
    figure(2);
    plot3(Pt(1, 1:i) + T0(1, 4), Pt(2, 1:i) + T0(2, 4), Pt(3, 1:i) + T0(3, 4));
    hold on;
end
%}