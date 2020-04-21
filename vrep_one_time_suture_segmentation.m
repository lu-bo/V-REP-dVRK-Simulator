
clearvars -except stereoParams; clc; close all;
image_path = 'C:\Users\User\OneDrive - The Hong Kong Polytechnic University\PhD Studies\Semester 4\3D Reconstruction\2017.03.20\v-repApi\';

% -------------------------------------------------------------------------
[left_image, right_image] = dVRK_virtual_sensor.get_stereo_vision;
left_image = imgaussfilt(left_image,1); right_image = imgaussfilt(right_image,1);
imwrite(left_image, 'left_image.jpg'); imwrite(right_image, 'right_image.jpg');
pause(0.1);
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
%[all_points_set_1, tip_1] = DL_based_intersected_suture_thread_detection_with_clicks(left_image, 'segmented_suture_l.jpg');
%[all_points_set_2, tip_2] = DL_based_intersected_suture_thread_detection_with_clicks(right_image, 'segmented_suture_r.jpg');

%left_image_seg = imread([image_path, 'segmented_suture_l.jpg']); 
%psedo_left_seg(:,:,1) = left_image_seg; psedo_left_seg(:,:,2) = left_image_seg; psedo_left_seg(:,:,3) = left_image_seg;
%right_image_seg = imread([image_path, 'segmented_suture_r.jpg']);
%psedo_right_seg(:,:,1) = right_image_seg; psedo_right_seg(:,:,2) = right_image_seg; psedo_right_seg(:,:,3) = right_image_seg;
% -------------------------------------------------------------------------


%imshow(left_image,'Border','tight'); hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r')
%{
figure;
Multi = cat(2, left_image, right_image);
Multi2 = cat(2, psedo_left_seg, psedo_right_seg);

Multi_total = cat(1, Multi, Multi2);

imshow(Multi_total, 'Border','tight');
if ((size(all_points_set_1, 1) ~= 0) && (size(all_points_set_1, 1) ~= 0))
    hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r')
    hold on; scatter(all_points_set_2(:, 1) + size(left_image, 2), all_points_set_2(:, 2), 5, 'r')
end
%}

%%
scale = 1;
iteration_steps = 1;
multi_view_num = 1;

%GUI_vision_sensor_control; 

% -------------------------------------------------------------------------
    
%pause; cab(1); 

[left_image, right_image] = dVRK_virtual_sensor.get_stereo_vision;

%left_image = imsharpen(left_image,'Radius',2,'Amount',2);
%right_image = imsharpen(right_image,'Radius',2,'Amount',2);

left_image = imgaussfilt(left_image, 1);
right_image = imgaussfilt(right_image, 1);

imwrite(left_image, 'left_image.jpg'); imwrite(right_image, 'right_image.jpg');
pause(0.1); 
left_image = imread([image_path,'left_image.jpg']);
right_image = imread([image_path,'right_image.jpg']);

iteration_steps = iteration_steps + 1;

[all_points_set_1, tip_1] = DL_based_intersected_suture_thread_detection_with_clicks(left_image, 'segmented_suture_l.jpg');
[all_points_set_2, tip_2] = DL_based_intersected_suture_thread_detection_with_clicks(right_image, 'segmented_suture_r.jpg');

s_1 = size(all_points_set_1, 1); s_2 = size(all_points_set_2, 1);

if (s_1 > 50 && s_2 > 50 && abs(s_1 - s_2) < 20)

    left_image_seg = imread([image_path, 'segmented_suture_l.jpg']); 
    psedo_left_seg(:,:,1) = left_image_seg; psedo_left_seg(:,:,2) = left_image_seg; psedo_left_seg(:,:,3) = left_image_seg;
    right_image_seg = imread([image_path, 'segmented_suture_r.jpg']);
    psedo_right_seg(:,:,1) = right_image_seg; psedo_right_seg(:,:,2) = right_image_seg; psedo_right_seg(:,:,3) = right_image_seg;

    %figure; imshow(left_image,'Border','tight');  hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r')
    %figure; imshow(right_image,'Border','tight'); hold on; scatter(all_points_set_2(:, 1), all_points_set_2(:, 2), 5, 'r')
    
    %{
    Multi1 = cat(2, left_image, right_image);        
    Multi2 = cat(2, psedo_left_seg, psedo_right_seg);
    Multi_total = cat(1, Multi1, Multi2);
    Multi_total = imresize(Multi_total, scale);
    figure(2); hold off; %view(0, 90);
    imshow(Multi_total, 'Border','tight');
    hold on; scatter(all_points_set_1(:, 1) * scale, all_points_set_1(:, 2) * scale, 5, 'r')
    hold on; scatter(all_points_set_2(:, 1) * scale + size(left_image, 2) * scale, all_points_set_2(:, 2) * scale, 5, 'r')
    %}

    % ---------------------------------------------------------------------
    Defined_key_points_num = min(size(all_points_set_1, 1), size(all_points_set_2, 1));
    key_point_camera_1 = suture_key_points_aligment(all_points_set_1, Defined_key_points_num); %key_point_camera_1_converse = suture_key_points_aligment(all_points_set_1(end: -1: 1, :), defined_key_points_num);
    key_point_camera_2 = suture_key_points_aligment(all_points_set_2, Defined_key_points_num);
    
    %----------------------------------------------------------------------
    [TR, TT, RTdata] = icp_multi_dim(key_point_camera_1,key_point_camera_2); 
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
    
%     
%     new_key_point_1 = [];
%     new_key_point_2 = [];
%     for number_p = 1 : size(key_point_camera_1, 1)
%         
%         for i = -1 : 1 : 1
%             for j = -1 : 1 : 1
%                 new_key_point_1 = [new_key_point_1; key_point_camera_1(number_p, :)];
%                 new_key_point_2 = [new_key_point_2; [key_point_camera_2(number_p, 1) + 0.8 * i, key_point_camera_2(number_p, 2) + 0.8 * i] ];
%             end
%         end
%         
%     end
%     xyz = triangulate(new_key_point_1, new_key_point_2, stereoParams) * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];
    
    Defined_key_points_num = size(xyz, 1);
    %figure; hold on; scatter3(xyz(:, 1), xyz(:, 2), xyz(:, 3), 5); grid on;
%%
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

    Dis_coefficient = bsxfun(@times, Element_weight_1, Element_weight_1'); %!!! revised this function.

    %%
    % .........................................................................
    % Give larger penalty to two elements whose number lists are too far from
    % each other.
    Row_Row = repmat(linspace(1, Defined_key_points_num, Defined_key_points_num), Defined_key_points_num, 1);
    Col_Col = repmat(linspace(1, Defined_key_points_num, Defined_key_points_num)', 1, Defined_key_points_num);
    Row_Col_diff = abs(Row_Row - Col_Col);
    Penalty_list = 1 + (exp(Row_Col_diff/20 - 2));
    %Penalty_list = 1 + Row_Col_diff;
% -------------------------------------------------------------------------
% --------------------find propoer path length threshold-------------------

    valid_path_length_maximum_length = 5;
    valid_path_length_minimum_length = 3;
    L = Inf;
    while (L == Inf)
        d = pdist(xyz);
        valid_path_length_maximum_length = valid_path_length_maximum_length + 0.5;
        list = find(d > valid_path_length_maximum_length | d < valid_path_length_minimum_length); % !!!with the increase of scale, we should decrese with value for more accurate computation!!!

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

    figure; hold off; view(-110, 30);
    h = plot(G,'XData',G.Nodes.X,'YData',G.Nodes.Y,'ZData',G.Nodes.Z, 'Marker','none','EdgeColor',[0.8500 0.3250 0.0980],'LineStyle', '-', 'LineWidth', 0.2);  grid on;
    
    figure;
    highlight(h,P,'EdgeColor',[0.4940 0.1840 0.5560],'LineWidth', 10);
    disp(L);
    
    hold on; scatter3(G.Nodes.X, G.Nodes.Y,G.Nodes.Z, 120, 'MarkerFaceColor', [0.4660 0.6740 0.1880], 'MarkerEdgeColor', [0 0.4470 0.7410])
    %
    set(gca,'FontSize',20);
    xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
    ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
    zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
    hold on; grid on; box on

    %scatter3(xyz(i_begin_node, 1), xyz(i_begin_node, 2), xyz(i_begin_node, 3), 40, 'r');
    
    
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

    % Drawing codes
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
    %multi_view_num = multi_view_num + 1;

end
% -------------------------------------------------------------------------

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

%%

figure;
point_data_set = multi_view_points_data_normalization(multi_view_coordinates_RCM_vision);

%
Optimized_point_set = [];
for ij = 1 : size(point_data_set, 3)
    
    Optimized_point_set(ij, :) = kalman_filter_multiple_points_optimization(point_data_set(:, :, ij));
    
end
% Shape after 
%plot3(Optimized_point_set(:, 1), Optimized_point_set(:, 2), Optimized_point_set(:, 3), 'r', 'LineWidth', 8); hold on;
Optimized_point_set_smoothing = dVRK_suture_thread_smoothing(Optimized_point_set);

final_length = 0;
for i = 1 : size(Optimized_point_set_smoothing, 1) - 1
    final_length = final_length + norm(Optimized_point_set_smoothing(i + 1, :) - Optimized_point_set_smoothing(i, :));
end
display(1000 * final_length);
%hold on; scatter3(Optimized_point_set(:, 1), Optimized_point_set(:, 2), Optimized_point_set(:, 3), 50, 'r')
xlabel('X/m', 'fontsize', 25, 'fontweight', 'bold');
ylabel('Y/m', 'fontsize', 25, 'fontweight', 'bold'); 
zlabel('Z/m', 'fontsize', 25, 'fontweight', 'bold');
hold on; grid on; box on
%

plot3(Optimized_point_set_smoothing(:, 1), Optimized_point_set_smoothing(:, 2), Optimized_point_set_smoothing(:, 3), 'k', 'LineWidth', 8); hold on;


%%
[t11, t22, Coo] = dVRK_get_transformation_and_coordinates('J1_ECM', 'J1_PSM1', Optimized_point_set_smoothing);



