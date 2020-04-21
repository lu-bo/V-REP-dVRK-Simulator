function suture_thread_shape_plot
%%
global G_3D_length
global P_3D_length
global H_3D_length

Window_length_1 = 380;
Window_length_2 = 450;
length_to_height_ratio = 480 / 640;
Window_gap_x = 20; 
Window_gap_y = 30;


Initial_x = 25;
Initial_y = 450;

f = figure;
ax = axes(f); 
ax.Units = 'pixels'; ax.Position = [Initial_x, Initial_y, Window_length_1, length_to_height_ratio * Window_length_1];
ax = gca; imshow(left_image);
ax.Title.String = 'Left Image';


ay = axes(f); 
ay.Units = 'pixels'; ay.Position = [Initial_x + Window_length_1 + Window_gap_x, Initial_y, ...
                                    Window_length_1, length_to_height_ratio * Window_length_1];
ay = gca; imshow(right_image);
ay.Title.String = 'Right Image';


aa = axes(f); 
aa.Units = 'pixels';
aa.Position = [Initial_x, Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y, ...
               Window_length_1, length_to_height_ratio * Window_length_1];
aa = gca; imshow(left_image); hold on; scatter(all_points_set_1(:, 1), all_points_set_1(:, 2), 5, 'r');
aa.Title.String = 'Segmented Suture - Left';

ab = axes(f); 
ab.Units = 'pixels';
ab.Position = [Initial_x + Window_length_1 + Window_gap_x, Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y, ...
               Window_length_1, length_to_height_ratio * Window_length_1];
ab = gca; imshow(right_image); hold on; scatter(all_points_set_2(:, 1), all_points_set_2(:, 2), 5, 'r');
ab.Title.String = 'Segmented Suture - Right';


ac = axes(f); 
ac.Units = 'pixels';
ac.Position = [Initial_x + 2 * (Window_length_1 + Window_gap_x) + Window_gap_x, (Initial_y + Initial_y - length_to_height_ratio * Window_length_1 - Window_gap_y)/2, ...
               Window_length_2, length_to_height_ratio * Window_length_2];
ac = gca; hold off; view(45, 45); 

H_3D_length = plot(G_3D_length, 'XData', G_3D_length.Nodes.X,'YData', G_3D_length.Nodes.Y,'ZData', G_3D_length.Nodes.Z);
highlight(H_3D_length, P_3D_length,'EdgeColor','r','LineWidth', 6);
%disp(L); grid on;
ac.Title.String = '3D Reconstruction';
grid on; box on;



end