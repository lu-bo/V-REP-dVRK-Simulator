%%

Window_length_1 = 450;
Window_length_2 = 530;
length_to_height_ratio = 480 / 640;
Window_gap_x = 20; 
Window_gap_y = 80;


Initial_x = 25;
Initial_y = 580;

f = figure;

ax = axes(f); 
ax.Units = 'pixels'; ax.Position = [Initial_x, Initial_y, Window_length_1, length_to_height_ratio * Window_length_1];
ax = gca; imshow(left_image);
ax.Title.String = 'Left Image';
ax.Title.FontSize = 20;


ay = axes(f); 
ay.Units = 'pixels'; ay.Position = [Initial_x + Window_length_1 + Window_gap_x, Initial_y, ...
                                    Window_length_1, length_to_height_ratio * Window_length_1];
ay = gca; imshow(right_image);
ay.Title.String = 'Right Image';
ay.Title.FontSize = 20;
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

aa = axes(f); 
aa.Units = 'pixels';
aa.Position = [Initial_x, Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y, ...
               Window_length_1, length_to_height_ratio * Window_length_1];
aa = gca; imshow(left_image); hold on; %scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r');
%
dc1=hsv(length(all_points_set_1));

for i = 1 : length(all_points_set_1) - 1
    line([all_points_set_1(i, 1) all_points_set_1(i + 1 , 1)], [all_points_set_1(i, 2) all_points_set_1(i + 1, 2)] , ...
        'Color', dc1(i, :), 'linewidth', 6);
end
%
aa.Title.String = 'Segmented Suture - Left';
aa.Title.FontSize = 20;

ab = axes(f); 
ab.Units = 'pixels';
ab.Position = [Initial_x + Window_length_1 + Window_gap_x, Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y, ...
               Window_length_1, length_to_height_ratio * Window_length_1];
ab = gca; imshow(right_image); hold on; %scatter(all_points_set_2(:, 1), all_points_set_2(:, 2), 5, 'r');
%
dc2=hsv(length(all_points_set_2));
for i = 1 : length(all_points_set_2) - 1
    line([all_points_set_2(i, 1) all_points_set_2(i + 1 , 1)], [all_points_set_2(i, 2) all_points_set_2(i + 1, 2)] , ...
         'Color', dc2(i, :), 'linewidth', 6);
end
%
ab.Title.String = 'Segmented Suture - Right';
ab.Title.FontSize = 20;
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------


% -------------------------------------------------------------------------
%%
% one figure stands alone for 3D shape 
Window_length_1 = 450;
Window_length_2 = 530;
length_to_height_ratio = 480 / 640;
Window_gap_x = 20; 
Window_gap_y = 80;


Initial_x = 25;
Initial_y = 580;

f = figure;
ad = axes(f); 
ad.Units = 'pixels';
ad.Position = [Initial_x+200, (Initial_y + Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y)/2 - 150, ...
               Window_length_2 *1.25, length_to_height_ratio * Window_length_2 * 1.5];
ad = gca; hold off; 


c_size = 1;
for i = 1 : Cell_size
    Input_xyz = multi_view_coordinates_RCM_vision{i};
    Input_xyz = Input_xyz * 1000;
    for j = 2 : size(multi_view_coordinates_RCM_vision{i}, 1)
        plot3([Input_xyz(j - 1, 1), Input_xyz(j, 1)], ...
              [Input_xyz(j - 1, 2), Input_xyz(j, 2)], ...
              [Input_xyz(j - 1, 3), Input_xyz(j, 3)], ...
              'Color', dc(c_size, :), 'LineWidth', 8); hold on;
         c_size = c_size + 1; 
         %pause(0.2);
    end
    
    %plot3(Input_xyz(:, 1), Input_xyz(:, 2), Input_xyz(:, 3), 'Color', dc(i, :), 'LineWidth', 3); hold on;
end
grid on;

%highlight(H_3D_length, P_3D_length,'EdgeColor','r','LineWidth', 6);
%disp(L); grid on;
ad.Title.String = 'Optimized 3D Shape of Suture';
ad.Title.FontSize = 20;
set(gca, 'FontSize', 20);
grid on; box on;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
set(gca,'ZDir','reverse'); 
set(gca,'XDir','reverse');
axis([0 50 -55 5 160 210])
view(-110,70);

%%
pause(3);
view(-110,70);
pause(3);
for ijv = 1 : 650
    view(-110 + ijv*0.2, 70);
    pause(0.005);
    
end

%%
% =========================================================================
% only two 3D figures

f=figure;

% -------------------------------------------------------------------------
% grasping point + 3D shape
Input_xyz = multi_view_coordinates_RCM_vision{1};
Input_xyz = Input_xyz * 1000;
total_len = 0;
for ii = 2 : size(Input_xyz, 1)
    total_len = total_len + norm(Input_xyz(ii, : ) - Input_xyz(ii - 1, :));
    
    if (total_len >= 50 )
        break;
    end
end
grasping_pont = Input_xyz(ii, :);

ad = axes(f); 
ad.Units = 'pixels';
ad.Position = [Initial_x + 2 * (Window_length_1 + Window_gap_x) + Window_gap_x + 100, (Initial_y + Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y)/2 - 150, ...
               Window_length_2 *1.25, length_to_height_ratio * Window_length_2 * 1.5];
ad = gca; hold off; 

scatter3(grasping_pont(1), grasping_pont(2), grasping_pont(3), 200, 'MarkerEdgeColor','r',...
        'MarkerFaceColor', 'r')
hold on;

text(500,500,'Grasping Point'); hold on;


c_size = 1;
for i = 1 : Cell_size
    for j = 2 : size(multi_view_coordinates_RCM_vision{i}, 1)
        plot3([Input_xyz(j - 1, 1), Input_xyz(j, 1)], ...
              [Input_xyz(j - 1, 2), Input_xyz(j, 2)], ...
              [Input_xyz(j - 1, 3), Input_xyz(j, 3)], ...
              'Color', dc(c_size, :), 'LineWidth', 8); hold on;
         c_size = c_size + 1; 
    end
    
    %plot3(Input_xyz(:, 1), Input_xyz(:, 2), Input_xyz(:, 3), 'Color', dc(i, :), 'LineWidth', 3); hold on;
end
grid on; hold on; 
%scatter3(grasping_pont(1), grasping_pont(2), grasping_pont(3), 'r', 20)

%highlight(H_3D_length, P_3D_length,'EdgeColor','r','LineWidth', 6);
%disp(L); grid on;
ad.Title.String = 'Optimized 3D Shape of Suture';
ad.Title.FontSize = 40;
set(gca, 'FontSize', 20);
grid on; box on;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
set(gca,'ZDir','reverse'); 
set(gca,'XDir','reverse');
axis([-40 40 -30 30 20 110])
view(-70,70);

%--------------------------------------------------------------------------
% graph for optimization
ac = axes(f); 
ac.Units = 'pixels';
ac.Position = [Initial_x + 180, (Initial_y + Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y)/2 - 150, ...
               Window_length_2 *1.25, length_to_height_ratio * Window_length_2 * 1.5];
ac = gca; hold off; 


H_3D_length = plot(G_3D_length, 'XData', G_3D_length.Nodes.X,'YData', G_3D_length.Nodes.Y,'ZData', G_3D_length.Nodes.Z,...
    'LineWidth', 1, 'EdgeColor', 'r', 'EdgeAlpha', 0.05);


%highlight(H_3D_length, P_3D_length,'EdgeColor','r','LineWidth', 6);
%disp(L); grid on;
ac.Title.String = 'Constructed Spatial Grasph';
ac.Title.FontSize = 40;
set(gca, 'FontSize', 20);
grid on; box on;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
set(gca,'ZDir','reverse');   
set(gca,'XDir','reverse');   
axis([-40 40 -30 30 20 110])
view(-70,70);




