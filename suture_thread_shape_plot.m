function suture_thread_shape_plot
%%
global G_3D_length
global P_3D_length
global H_3D_length

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

ac = axes(f); 
ac.Units = 'pixels';
ac.Position = [Initial_x + 2 * (Window_length_1 + Window_gap_x) + Window_gap_x + 100, (Initial_y + Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y)/2 - 150, ...
               Window_length_2 *1.25, length_to_height_ratio * Window_length_2 * 1.5];
ac = gca; hold off; 


H_3D_length = plot(G_3D_length, 'XData', G_3D_length.Nodes.X,'YData', G_3D_length.Nodes.Y,'ZData', G_3D_length.Nodes.Z,...
    'LineWidth', 1, 'EdgeColor', 'r', 'EdgeAlpha', 0.05);


%highlight(H_3D_length, P_3D_length,'EdgeColor','r','LineWidth', 6);
%disp(L); grid on;
ac.Title.String = 'Constructed Spatial Grasph';
ac.Title.FontSize = 20;
set(gca, 'FontSize', 20);
grid on; box on;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
set(gca,'ZDir','reverse');   
set(gca,'XDir','reverse');   
%axis([-10 85 -70 45 200 300])
view(45,40);

end