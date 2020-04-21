close all; clc;
clearvars -except stereoParams
multi_view_num = 1;

GUI_vision_sensor_control; 

%%
clearvars -except stereoParams multi_view_num multi_view_coordinates_RCM_vision;
close all; clc;

%%
[left_origin, right_origin] = dVRK_virtual_sensor.get_stereo_vision;
%load('C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\virtual_chess_board_calibration\virtual_chess_board_calibration.mat')
%
%[left_rectified, right_rectified] = rectifyStereoImages(left_origin,right_origin, stereoParams,'OutputView','full');

%
left_rectified = left_origin; right_rectified = right_origin;

left = rgb2gray(left_rectified);
[all_points_set_1, suture_tip1] = func_intersected_suture_thread_detection_with_clicks(left, left_rectified);
figure(2); imshow(left_rectified); line(all_points_set_1(:, 1), all_points_set_1(:, 2), 'Color', [1 0 0], 'linewidth', 2);

right = rgb2gray(right_rectified);
[all_points_set_2, suture_tip2] = func_intersected_suture_thread_detection_with_clicks(right, right_rectified);
figure(3); imshow(right_rectified); line(all_points_set_2(:, 1), all_points_set_2(:, 2), 'Color', [1 0 0], 'linewidth', 2);
cab(1);
%%
% Assumption: the optical axes of two cameras are nearly parallel to each
% other, we can match key points proportionally.
defined_key_points_num = min(size(all_points_set_1, 1), size(all_points_set_2, 1));
key_point_camera_1 = suture_key_points_aligment(all_points_set_1, defined_key_points_num);

key_point_camera_1_converse = suture_key_points_aligment(all_points_set_1(end: -1: 1, :), defined_key_points_num);
%{
max_num_set_2 = size(all_points_set_2, 1);
key_point_camera_2 = [];
range = 5;

for i = 1 : size(key_point_camera_1)
    evaluation_num = [];
    
    for j = i - range : 1 : i + range
        if (j > 0 && j < max_num_set_2)
            evaluation_num = [evaluation_num; j];
        end
    end
    
    repeat_size = size(evaluation_num, 1);
    % calculate epipolar constrains, and the results are the same in each
    % column in "stereo_focus"
    stereo_focus = repmat([key_point_camera_1(i, :), 1], repeat_size, 1)*stereoParams.FundamentalMatrix*[all_points_set_2(evaluation_num, :), repmat(1, repeat_size, 1)]';
    
    % take the first row and align the results in column style
    stereo_focus = abs(stereo_focus(1, :)');
    ind = find(stereo_focus ==min(min(stereo_focus)));
    
    if (i - range > 0)
        final_ind = ind + (i - range) - 1;
    else
        final_ind = ind;
    end
    
    key_point_camera_2(i, :) = all_points_set_2(final_ind, :);
end
%%
%}
key_point_camera_2 = suture_key_points_aligment(all_points_set_2, defined_key_points_num);
key_point_camera_2_converse = suture_key_points_aligment(all_points_set_2(end: -1: 1, :), defined_key_points_num);
%{
%%
%resolution = [size(left_rectified, 2), size(left_rectified, 1)];

resolution = stereoParams.CameraParameters1.PrincipalPoint;
resolution = [640 320];
for i = 1 : size(key_point_camera_1, 1)
    z(i, 1) = - (stereoParams.CameraParameters1.FocalLength(1) + stereoParams.CameraParameters1.FocalLength(2)) / 2 ...
              * abs(stereoParams.TranslationOfCamera2(1)) / (key_point_camera_2(i, 1) - key_point_camera_1(i, 1));

    x(i, 1) = (resolution(1) / 2 - key_point_camera_1(i, 1))* z(i, 1) / ((stereoParams.CameraParameters1.FocalLength(1) + stereoParams.CameraParameters2.FocalLength(1)) / 2);
    y(i, 1) = (resolution(2) / 2 - key_point_camera_1(i, 2))* z(i, 1) / ((stereoParams.CameraParameters1.FocalLength(2) + stereoParams.CameraParameters2.FocalLength(2)) / 2);
end
%figure; scatter3(x(: , 1) , y(: , 1) , z(: , 1) , 20 ); hold on;
xyz = ([x, y, z]);
%}
%
xyz = triangulate(key_point_camera_1, key_point_camera_2, stereoParams) * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];

xyz_converse = triangulate(key_point_camera_1_converse, key_point_camera_2_converse, stereoParams) * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];
%{
key_points_num = 50;
key_points_interval = size(xyz, 1) / key_points_num;

visualized_3D_points = [];

for i = 1 : key_points_num
    visualized_3D_points(i, :) = xyz(1 + round((i - 1) * key_points_interval), :);   
end
visualized_3D_points(i+1, :) = xyz(end, :);
%}

%%
% Based on the distribution of the 3D point to find the optimal Tip and End
% xyz = xyz_converse
End_point_gap_threshold = 3; % it is tunable parameter --> according to scale
End_point_neighbour_num = 8;

for i_begin_node = 1 : defined_key_points_num

    dis_i =  sqrt(sum((xyz - xyz(i_begin_node, :)) .^ 2, 2));
    marker = size(find(dis_i < End_point_gap_threshold), 1);

    if (marker > End_point_neighbour_num)
        break;
    end

end
for j_end_node = defined_key_points_num : -1 : 1

    dis_i =  sqrt(sum((xyz - xyz(j_end_node, :)) .^ 2, 2));
    marker = size(find(dis_i < End_point_gap_threshold), 1);

    if (marker > End_point_neighbour_num)
        break;
    end

end
%scatter3(xyz(i, 1), xyz(i, 2), xyz(i, 3), 80, 'r'); hold on;
%scatter3(xyz(j, 1), xyz(j, 2), xyz(j, 3), 80, 'r'); hold on;

% .........................................................................
% Give weight to each point accroding to the total number of the nearby
% element --> what is nearby? --> the distance to this point is smaller
% than "End_point_gap_threshold". 

for ii = 1 : defined_key_points_num
    dis_i =  sqrt(sum((xyz - xyz(ii, :)) .^ 2, 2));
    near_element(ii, 1) = size(find(dis_i < End_point_gap_threshold), 1);
    Element_weight_1(ii, 1) = 1 / near_element(ii, 1); %exp(near_element(ii, 1) -  mean(near_element));
end

Dis_coefficient = bsxfun(@times,Element_weight_1, Element_weight_1');

% .........................................................................
% Give larger penalty to two elements whose number lists are too far from
% each other.
Row_Row = repmat(linspace(1, defined_key_points_num, defined_key_points_num), defined_key_points_num, 1);
Col_Col = repmat(linspace(1, defined_key_points_num, defined_key_points_num)', 1, defined_key_points_num);
Row_Col_diff = abs(Row_Row - Col_Col);
Penalty_list = 1 + (exp(Row_Col_diff/20 - 2));

%%
%
%Position = 10*rand(10,3);
%Position = xyz;
%nodes_num = size(Position, 1);
% Calculate euclid distance between each nodes
d = pdist(xyz);

% -------------------------------------------------------------------------
% --------------------find propoer path length threshold-------------------
valid_path_length_maximum_length = 5;
    
list = find(d > valid_path_length_maximum_length); % !!!with the increase of scale, we should decrese with value for more accurate computation!!!

% Set distance to 0 for 30 node-pairs
%pt = randperm(numel(d),  3*nodes_num);
d(list) = 0;
% Convert to Adjacency matrix
d_Adjacency = squareform(d);

hyper_d = d_Adjacency .* Dis_coefficient;% .* Penalty_list;
% Generate Graph object and store XYZ coordinates in it
G = graph(hyper_d);
G.Nodes = array2table(xyz,'VariableNames',{'X','Y','Z'});
% Calculate shortest path between node 1 and end
[P,L] = shortestpath(G, i_begin_node, j_end_node);

% Visualize the result
%
figure;
h = plot(G,'XData',G.Nodes.X,'YData',G.Nodes.Y,'ZData',G.Nodes.Z);
highlight(h,P,'EdgeColor','r','LineWidth', 6);
% Display the path length
disp(L); grid on;
%
updated_nodes = xyz(P, :);
xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
hold on; grid on; box on
%
%%
%updaye_nodes = [x, y, z];
%{
length = 0;
for i = 1 : size(updaye_nodes, 1) - 1
    %plot3([x(i , 1) , x(i + 1 , 1)] , [y(i , 1) , y(i + 1 , 1)] , [z(i , 1) , z(i + 1 , 1)] , 'b', 'LineWidth' , 1 ); hold on;
    %scatter3(x(i , 1) , y(i , 1) , z(i , 1) , 20 ); hold on;
    length = norm(updaye_nodes(i + 1, :) - updaye_nodes(i, :));
end
%scatter3(x(i + 1 , 1) , y(i + 1 , 1) , z(i + 1 , 1) , 20 );
display(length); grid on; hold on;
%}

% Savitzky¨CGolay filter (sgolayfilt) - smoothing individual axes
xyzsg = dVRK_suture_thread_smoothing(updated_nodes);

xyzsg = xyzsg * 0.001;

length_optimize = dVRK_plot_suture_thread(xyzsg, 'r', 7); % output total length based on the optimized 3D coordinates
display(length_optimize * 1000);

   %%
[t1_RCM_vision, t2_RCM_vision, xyzco_RCM_vision] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_ECM', xyzsg);

%[t1_RCM_leftArm, t2_RCM_leftArm, xyzco_RCM_leftArm] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', xyzsg);

multi_view_coordinates_RCM_vision{multi_view_num} = xyzco_RCM_vision;
multi_view_num = multi_view_num + 1;

figure; 
plot3(xyzco_RCM_vision(:, 1), xyzco_RCM_vision(:, 2), xyzco_RCM_vision(:, 3), 'b', 'LineWidth', 7); hold on;
length_transformed_coordinate = 0;
for i = 1 : size(xyzco_RCM_vision, 1) - 1
    length_transformed_coordinate = norm(xyzco_RCM_vision(i + 1, :) - xyzco_RCM_vision(i, :)) + length_transformed_coordinate;
end

display(length_transformed_coordinate);

%title('Suture Thread Shape in 3D Space', 'fontsize', 12, 'fontweight', 'bold');
xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
hold on; grid on; box on






%%
% -------------------------------- raw data -------------------------------
Cell_size = size(multi_view_coordinates_RCM_vision, 2);
figure; hold on;

total_size = 0;
for i = 1 : Cell_size
    total_size = total_size + size(multi_view_coordinates_RCM_vision{1, i}, 1);
end

dc = hsv(round(1.1 * total_size)); 

c_size = 1;
for i = 1 : Cell_size
    Input_xyz = multi_view_coordinates_RCM_vision{i};
    
    for j = 2 : size(multi_view_coordinates_RCM_vision{i}, 1)
        plot3([Input_xyz(j - 1, 1), Input_xyz(j, 1)], ...
              [Input_xyz(j - 1, 2), Input_xyz(j, 2)], ...
              [Input_xyz(j - 1, 3), Input_xyz(j, 3)], ...
              'Color', dc(c_size, :), 'LineWidth', 2); hold on;
         c_size = c_size + 1; 
    end
    
    %plot3(Input_xyz(:, 1), Input_xyz(:, 2), Input_xyz(:, 3), 'Color', dc(i, :), 'LineWidth', 3); hold on;
end
grid on;

%%
point_data_set = multi_view_points_data_normalization(multi_view_coordinates_RCM_vision);

%%
Optimized_point_set = [];
for ij = 1 : size(point_data_set, 3)
    
    Optimized_point_set(ij, :) = kalman_filter_multiple_points_optimization(point_data_set(:, :, ij));
    
end
plot3(Optimized_point_set(:, 1), Optimized_point_set(:, 2), Optimized_point_set(:, 3), 'k', 'LineWidth', 8); hold on;

final_length = 0;
for i = 1 : size(Optimized_point_set, 1) - 1
    final_length = final_length + norm(Optimized_point_set(i + 1, :) - Optimized_point_set(i, :));
end
display(1000 * final_length);
%hold on; scatter3(Optimized_point_set(:, 1), Optimized_point_set(:, 2), Optimized_point_set(:, 3), 50, 'r')
xlabel('X/m', 'fontsize', 25, 'fontweight', 'bold');
ylabel('Y/m', 'fontsize', 25, 'fontweight', 'bold'); 
zlabel('Z/m', 'fontsize', 25, 'fontweight', 'bold');
hold on; grid on; box on

%%
[t11, t22, Coo] = dVRK_get_transformation_and_coordinates('J1_ECM', 'J1_PSM1', Optimized_point_set);


%%
[~, ~, Coo_J3_PSM1] = dVRK_get_transformation_and_coordinates('J1_PSM1', 'J3_PSM1', Coo);
Coo_J3_PSM1 = Coo_J3_PSM1 + [0, 0, 0];
[~, ~, Coo] = dVRK_get_transformation_and_coordinates('J3_PSM1', 'J1_PSM1', Coo_J3_PSM1);


%%
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

[~, J3_dx_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J3_dx_TOOL1', vrep.simx_opmode_blocking);
[~, J3_sx_TOOL1] = vrep.simxGetObjectHandle(clientID, 'J3_sx_TOOL1', vrep.simx_opmode_blocking);
[~, J1_PSM1] = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);

%[~, C_TOOL_Position] = vrep.simxGetObjectPosition(clientID, J3_dx_TOOL1, J1_PSM1, vrep.simx_opmode_blocking);

% The translation between gripper middle point and the base of "J2_TOOL1"
T_J2_TOOL1_to_grasper = 0.0156; % Unit --> "M"

% TOOL_Tip_length = 1.1933e-02 / 2;
v_1 = Coo(10, :) - Coo(1, :);
v_2 = Coo(10, :) - Coo(end, :);
V = cross(v_1, v_2);

Prop = -T_J2_TOOL1_to_grasper / norm(V);

Target_1 = Coo(1, :) - Prop * V;
%Target_2 = Coo(1, :) - Prop * V;
%Coo_orientated = min(Target_1, Target_2);
Coo_orientated = Target_1;

%Vector = Coo(1, :) - C_TOOL_Position;
%Proportion = 1 - TOOL_Tip_length / norm(Vector);
%Target = C_TOOL_Position + Proportion * Vector;
%Coo_orientated = Target;

for i = 2 : size(Coo, 1)
    
    if (i <= 3)
        %Prop = TOOL_Tip_length / norm(V);
        Target_1(i, :) = Coo(i, :) - Prop * V;
        %Target_2(i, :) = Coo(i, :) - Prop * V;
        %Coo_orientated(i, :) = min(Target_1(i, :), Target_2(i, :));
        Coo_orientated(i, :) = Target_1(i, :);
    else
        %v_1 = Coo(i - 1, :) - Coo(i, :);
        %v_2 = Coo(i - 2, :) - Coo(i - 1, :);
        %V = cross(v_1, v_2);
        %Prop = TOOL_Tip_length / norm(V);
        Target_1(i, :) = Coo(i, :) - Prop * V;
        %Target_2(i, :) = Coo(i, :) - Prop * V;
        %Coo_orientated(i, :) = min(Target_1(i, :), Target_2(i, :));
        Coo_orientated(i, :) = Target_1(i, :);
    end
     
end


%%
figure; plot3(Coo(:, 1), Coo(:, 2), Coo(:, 3), 'r', 'LineWidth', 4); hold on; scatter3(Coo(:, 1), Coo(:, 2), Coo(:, 3), 'b')
hold on; grid on;
plot3(Coo_orientated(:, 1), Coo_orientated(:, 2), Coo_orientated(:, 3), 'b', 'LineWidth', 4); hold on; scatter3(Coo_orientated(:, 1), Coo_orientated(:, 2), Coo_orientated(:, 3), 'r')

hold on; plot3(kk(:, 1), kk(:, 2), kk(:, 3), 'k')
