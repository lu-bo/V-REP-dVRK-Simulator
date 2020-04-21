%%
%---------------------------------------------------------------
clearvars -except stereoParams; clc; close all;
image_path = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\';
%---------------------------------------------------------------


%[left_image, right_image] = dVRK_virtual_sensor.get_stereo_vision;
left_image  = imread([image_path, 'left_image.jpg'] );
right_image = imread([image_path, 'right_image.jpg']);
left_image = imgaussfilt(left_image,1); right_image = imgaussfilt(right_image,1); pause(0.5);


%---------------------------------------------------------------
[all_points_set_1, tip_1] = DL_based_intersected_suture_thread_detection_with_clicks(left_image, 'segmented_suture_l.jpg');
%figure; imshow(left_image); hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r');

[all_points_set_2, tip_2] = DL_based_intersected_suture_thread_detection_with_clicks(right_image, 'segmented_suture_r.jpg');

left_image_seg = imread([image_path, 'segmented_suture_l.jpg']); 
psedo_left_seg(:,:,1) = left_image_seg; psedo_left_seg(:,:,2) = left_image_seg; psedo_left_seg(:,:,3) = left_image_seg;

right_image_seg = imread([image_path, 'segmented_suture_r.jpg']);
psedo_right_seg(:,:,1) = right_image_seg; psedo_right_seg(:,:,2) = right_image_seg; psedo_right_seg(:,:,3) = right_image_seg;

%
scale = 1;
iteration_steps = 1;
multi_view_num = 1;

while iteration_steps <= 1    
    %pause; 
    %cab(1); pause;
        
    left_image = imread([image_path, 'left_image.jpg']);
    right_image = imread([image_path, 'right_image.jpg']);
    
    %left_image = imsharpen(left_image,'Radius',2,'Amount',2);
    %right_image = imsharpen(right_image,'Radius',2,'Amount',2);
    
    left_image = imgaussfilt(left_image, 1);
    right_image = imgaussfilt(right_image, 1);
    pause(0.5); 
        
    iteration_steps = iteration_steps + 1;
    
    [all_points_set_1, tip_1] = DL_based_intersected_suture_thread_detection_with_clicks(left_image, 'segmented_suture_l.jpg', tip_1);
    [all_points_set_2, tip_2] = DL_based_intersected_suture_thread_detection_with_clicks(right_image, 'segmented_suture_r.jpg', tip_2);
    
    s_1 = size(all_points_set_1, 1); s_2 = size(all_points_set_2, 1);
    
    if (s_1 > 50 && s_2 > 50 && abs(s_1 - s_2) < 100)

        left_image_seg = imread([image_path, 'segmented_suture_l.jpg']); 
        psedo_left_seg(:,:,1) = left_image_seg; psedo_left_seg(:,:,2) = left_image_seg; psedo_left_seg(:,:,3) = left_image_seg;
        right_image_seg = imread([image_path, 'segmented_suture_r.jpg']);
        psedo_right_seg(:,:,1) = right_image_seg; psedo_right_seg(:,:,2) = right_image_seg; psedo_right_seg(:,:,3) = right_image_seg;

        %imshow(left_image,'Border','tight'); hold on; scatter(all_points_1(:, 1), all_points_1(:, 2), 5, 'r')
        Multi = cat(2, left_image, right_image);        
        %Multi = cat(2, psedo_left_seg, psedo_right_seg);
        Multi2 = cat(2, psedo_left_seg, psedo_right_seg);

        Multi_total = cat(1, Multi, Multi2);
        Multi_total = imresize(Multi_total, scale);
        
        figure(2); hold off; %view(0, 90);
        imshow(Multi_total, 'Border','tight');
        hold on; scatter(all_points_set_1(:, 1) * scale, all_points_set_1(:, 2) * scale, 5, 'r')
        hold on; scatter(all_points_set_2(:, 1) * scale + size(left_image, 2) * scale, all_points_set_2(:, 2) * scale, 5, 'r')
        
        Defined_key_points_num = min(size(all_points_set_1, 1), size(all_points_set_2, 1));
        key_point_camera_1 = suture_key_points_aligment(all_points_set_1, Defined_key_points_num);
        key_point_camera_1_converse = suture_key_points_aligment(all_points_set_1(end: -1: 1, :), Defined_key_points_num);

        key_point_camera_2 = suture_key_points_aligment(all_points_set_2, Defined_key_points_num);
        
        %----------------------------------------------------------------------
        [TR, TT] = icp_multi_dim(key_point_camera_1,key_point_camera_2); 
        
        RTdata = TR * key_point_camera_2' + repmat(TT, 1, size(key_point_camera_2, 1));
        RTdata = RTdata';
        %{
        figure; scatter(key_point_camera_1(:, 1), key_point_camera_1(:, 2),'r'); hold on; 
        scatter(key_point_camera_2(:, 1), key_point_camera_2(:, 2), 'b'); hold on; 
        scatter(RTdata(:, 1), RTdata(:, 2), 'y');
        %}
        %----------------------------------------------------------------------
        
        ss_1 = size(key_point_camera_1, 1);
        ss_2 = size(key_point_camera_2, 1);

        xyz = triangulate(key_point_camera_1, key_point_camera_2, stereoParams) * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];

        End_point_gap_threshold = 3; % it is tunable parameter --> according to scale
        End_point_neighbour_num = 5;

        for i_begin_node = 1 : Defined_key_points_num

            dis_i =  sqrt(sum((xyz - xyz(i_begin_node, :)) .^ 2, 2));
            marker = size(find(dis_i < End_point_gap_threshold), 1);

            if (marker > End_point_neighbour_num)
                break;
            end

        end
        for j_end_node = Defined_key_points_num : -1 : 1

            dis_i =  sqrt(sum((xyz - xyz(j_end_node, :)) .^ 2, 2));
            marker = size(find(dis_i < End_point_gap_threshold), 1);

            if (marker > End_point_neighbour_num)
                break;
            end

        end

        Element_weight_1 = [];
        for ii = 1 : Defined_key_points_num
            dis_i =  sqrt(sum((xyz - xyz(ii, :)) .^ 2, 2));
            near_element(ii, 1) = size(find(dis_i < End_point_gap_threshold), 1);
            Element_weight_1(ii, 1) = 1 / near_element(ii, 1); %exp(near_element(ii, 1) -  mean(near_element));
        end

        Dis_coefficient = bsxfun(@times, Element_weight_1, Element_weight_1');

        % .........................................................................
        % Give larger penalty to two elements whose number lists are too far from
        % each other.
        Row_Row = repmat(linspace(1, Defined_key_points_num, Defined_key_points_num), Defined_key_points_num, 1);
        Col_Col = repmat(linspace(1, Defined_key_points_num, Defined_key_points_num)', 1, Defined_key_points_num);
        Row_Col_diff = abs(Row_Row - Col_Col);
        Penalty_list = 1 + (exp(Row_Col_diff/20 - 2));

    % -------------------------------------------------------------------------
    % --------------------find propoer path length threshold-------------------
    
        valid_path_length_maximum_length = 8;
        L = Inf;
        while (L == Inf)
            d = pdist(xyz);
            valid_path_length_maximum_length = valid_path_length_maximum_length + 0.5;
            list = find(d > valid_path_length_maximum_length); % !!!with the increase of scale, we should decrese with value for more accurate computation!!!

            % Set distance to 0 for 30 node-pairs
            %pt = randperm(numel(d),  3*nodes_num);
            
            d(list) = 0;
            % Convert to Adjacency matrix
            d_Adjacency = squareform(d);

            hyper_d = d_Adjacency .* Dis_coefficient.* Penalty_list;
            % Generate Graph object and store XYZ coordinates in it
            G = graph(hyper_d);
            G.Nodes = array2table(xyz,'VariableNames',{'X','Y','Z'});
            % Calculate shortest path between node 1 and end
            [P,L] = shortestpath(G, i_begin_node, j_end_node);

            % Visualize the result
            % Display the path length
        end
        
        figure(3); hold off; view(45, 45);
        h = plot(G,'XData',G.Nodes.X,'YData',G.Nodes.Y,'ZData',G.Nodes.Z);
        highlight(h,P,'EdgeColor','r','LineWidth', 6);
        disp(L); grid on;
        %

        %xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
        %ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
        %zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
        %hold on; grid on; box on

        updated_nodes = xyz(P, :);
        xyzsg = dVRK_suture_thread_smoothing(updated_nodes);

        xyzsg = xyzsg * 0.001;
        length_optimize = 0;
        for i = 1 : size(xyzsg, 1) - 1
            length_optimize = norm(xyzsg(i + 1, :) - xyzsg(i, :)) + length_optimize;
        end
        display(length_optimize);
        [t1_RCM_vision, t2_RCM_vision, xyzco_RCM_vision] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_ECM', xyzsg);

        %[t1_RCM_leftArm, t2_RCM_leftArm, xyzco_RCM_leftArm] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', xyzsg);
        
        %
        %{
        figure(3); hold off;
        plot3(xyzsg(:, 1), xyzsg(:, 2), xyzsg(:, 3), 'r', 'LineWidth', 7);
        xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
        ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
        zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
        %xlim([-0.05 0.025]); ylim([-0.02 0.04]); zlim([0.08 0.14]);
        view(-0, 0);
        grid on; box on;

        figure(4); hold off;
        plot3(xyzsg(:, 1), xyzsg(:, 2), xyzsg(:, 3), 'r', 'LineWidth', 7);
        xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
        ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
        zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
        %xlim([-0.05 0.025]); ylim([-0.02 0.035]); zlim([0.08 0.15]); 
        view(-90, 90);
        grid on; box on;
        %}
        
        display(length_optimize * 1000);
        show_content = strcat('Iteration' , {32} , num2str(iteration_steps));
        display(num2str(show_content{1}));

        
        multi_view_coordinates_RCM_vision{multi_view_num} = xyzco_RCM_vision;
        multi_view_num = multi_view_num + 1;
        
    end    
end
disp('Finished');
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

% -------------------------------------------------------------------------
% Optimization using Kalman Filter
point_data_set = multi_view_points_data_normalization(multi_view_coordinates_RCM_vision);

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
% -------------------------------------------------------------------------
% Transform the coordinates into robotic frame 
[t11, t22, Coo] = dVRK_get_transformation_and_coordinates('J1_ECM', 'J1_PSM1', Optimized_point_set);


%%
% -------------------------------------------------------------------------
% Scene reconstruction using TPS model 
Suture_area = zeros(size(left_image, 1), size(left_image, 2));
for i = 1 : size(key_point_camera_1, 1)
    Suture_area(key_point_camera_1(i, 2), key_point_camera_1(i, 1)) = 255;  
end
J = imdilate(Suture_area, ones(25, 25));
[Construct_y_l, Construct_x_l] = find(J > 0);

%figure; imshow(J);
%hold on; scatter(Construct_x_l(1), Construct_y_l(1), 10)

%%
x_min = min(key_point_camera_1(:, 1)); x_max = max(key_point_camera_1(:, 1));
x_diff = x_max - x_min;

y_min = min(key_point_camera_1(:, 2)); y_max = max(key_point_camera_1(:, 2));
y_diff = y_max - y_min;

X_Size = round(1.05*x_diff); Y_Size = round(1.05*y_diff);
X_Start = 0.5 * (x_min + x_max - X_Size); Y_Start = 0.5 * (y_min + y_max - Y_Size);

Range = [X_Start, Y_Start, X_Size, Y_Size];
Dist_XY = [round(X_Size/3); round(Y_Size/3)];
 
% -------------------------------------------------------------------------
% we rotate the image and use it for the later scene reconstruction

im1 = right_image;

pos = [TR*[[0.5*size(right_image, 1), 0.5*size(right_image, 2)] - [TT(2), TT(1)]]']'; 
theta = asin(TR(1, 2));
sz = [size(right_image, 1), size(right_image, 2)];
rotated_image = get_pixels( im1, pos, sz, theta );
figure; imshow(rotated_image);
%hold on; scatter(key_point_camera_2(:, 1), key_point_camera_2(:, 2),'b');
%hold on; scatter(RTdata(:, 1), RTdata(:, 2), 'y');
%hold on; scatter(key_point_camera_1(:, 1), key_point_camera_1(:, 2),'r');

%[Tx_l, Ty_l, Tx, Ty] = dVRK_TPS(left_image, rotated_image, Range, Dist_XY);

% -------------------------------------------------------------------------
% highlight the influence of suture thread
left_suture = zeros(480, 640);
right_suture = zeros(480, 640);

for i = 1 : size(key_point_camera_1, 1)
    left_suture(key_point_camera_1(i, 2), key_point_camera_1(i, 1)) = 255; 
end
for i = 1 : size(key_point_camera_2, 1)
    right_suture(key_point_camera_2(i, 2), key_point_camera_2(i, 1)) = 255; 
end
right_suture = get_pixels( right_suture, pos, sz, theta );
left_suture = double(left_suture);
right_suture = double(right_suture);
%figure; imshow(right_suture);

[Tx_l, Ty_l, Tx, Ty] = dVRK_TPS_highlight_suture(left_image, rotated_image, Range, Dist_XY, left_suture, right_suture);
%[Tx_l, Ty_l, Tx, Ty] = dVRK_TPS(left_image, rotated_image, Range, Dist_XY);
% -------------------------------------------------------------------------
% Dense 3D plot
Point_l = ([Tx_l(:), Ty_l(:)]);
Point_l_standard = round(Point_l);

Point_r = ([Tx(:), Ty(:)]);
Point_r = inv(TR) * (Point_r' - repmat(TT, 1, size(Point_r, 1)));
Point_r = Point_r';
Point_r_standard = round(Point_r);

shape_points = triangulate(Point_l, Point_r, stereoParams) * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];

%%
showPoints = [];
Point_color = [];

for i = 1 : size(Construct_x_l, 1)
    currentLoop_numberList = find(abs(Point_l_standard(:, 1) -  Construct_x_l(i, 1)) == 0 & abs(Point_l_standard(:, 2) -  Construct_y_l(i, 1)) == 0);
    showPoints = [showPoints; shape_points(currentLoop_numberList, :)];
    
    for layer = 1 : 3
        Point_color(i, layer) = left_image(round(Construct_y_l(i, 1)), round(Construct_x_l(i, 1)), layer);
    end
    
end
%figure; imshow(right_image); hold on; scatter(Point_r(:, 1), Point_r(:, 2), 1, 'r')
 
Point_color = double(Point_color);
figure; 
scatter3(showPoints(:, 1), showPoints(:, 2), showPoints(:, 3), 10, Point_color/255);
set(gca,'ZDir','reverse');
xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
hold on;
Computed_3D_suture_part = plot3(1000*xyzsg(:,1), 1000*xyzsg(:,2), 1000*xyzsg(:,3), 'k');
Computed_3D_suture_part.LineWidth = 5;
grid on;

%%
%--------------------------------------------------------------------------
% Only with nearby position
Point_color = [];
for i = 1 : size(shape_points, 1)
    for layer = 1 : 3
        Point_color(i, layer) = left_image(round(Point_l(i, 2)), round(Point_l(i, 1)), layer);
    end
end

Point_color = double(Point_color);
%%
figure;
view(20 , 14);  
set(gca,'ZDir','reverse');
xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');

set(gca,'FontSize',20);
hold on;
Computed_3D_suture = plot3(1000*xyzsg(:,1), 1000*xyzsg(:,2), 1000*xyzsg(:,3), 'r');
Computed_3D_suture.LineWidth = 5;
grid on;
% xlim([-20, 80])
%ylim([-50, 40])
% zlim([130, 230])
pause;

alpha = 0.5;
p1 = scatter3(shape_points(:, 1), shape_points(:, 2), shape_points(:, 3), 10, Point_color/255);

%set(gca,'XDir','reverse');
%set(gca,'YDir','reverse');
%set(gca,'XDir','reverse');

% pause;
% for view_i = 1 : 180
%     for view_j = 45 : 45
%         view(view_i , view_j);
%         pause(0.1)
%     end
% end
%%
% coordinate transformation
[t1, t2, Coo] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', xyzsg);

[~, ~, T_shape_points] = dVRK_get_transformation_and_coordinates('Vision_sensor_left', 'J1_PSM1', 0.001*shape_points);

figure(6);
scatter3(T_shape_points(:, 1), T_shape_points(:, 2), T_shape_points(:, 3), 10, Point_color/255);

% set(gca,'XDir','reverse');
%set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
hold on;
plot3(Coo(:,1), Coo(:,2), Coo(:,3), 'r');
grid on;


%
SS = T_shape_points - Coo(1, :);
X = vecnorm(SS');
Numbering = find(X == min(X));

nearest_p = T_shape_points(Numbering, :);
hold on; scatter3(nearest_p(1,1), nearest_p(1,2), nearest_p(1,3), 70)
%

SS_2 = T_shape_points - nearest_p;
X_2 = vecnorm(SS_2');
Numbering_2 = find(X_2 < 0.001);

nearest_group = T_shape_points(Numbering_2, :);
hold on; scatter3(nearest_group(:,1), nearest_group(:,2), nearest_group(:,3), 7);
%%
d_v = nearest_group - nearest_p;
temp_r = find(d_v(:, 1) == 0);
d_v(temp_r, :) = [];
curve_surf_vector = cross(d_v(1,:), d_v(2,:));


plot_p = nearest_p + curve_surf_vector*100000;
%hold on; plot3([plot_p(1,1), nearest_p(1,1)], [plot_p(1,2), nearest_p(1,2)], [plot_p(1,3), nearest_p(1,3)], 'r')

%}
