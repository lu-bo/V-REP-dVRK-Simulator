function length = dVRK_plot_suture_thread(input_xyz, line_color, line_width)

    figure; plot3(input_xyz(:, 1), input_xyz(:, 2), input_xyz(:, 3), line_color, 'LineWidth', line_width); hold on;
    length = 0;
    for i = 1 : size(input_xyz, 1) - 1
        length = norm(input_xyz(i + 1, :) - input_xyz(i, :)) + length;
    end
    xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
    ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
    zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
    hold on; grid on; box on
    %scatter3(xyzsg(i + 1 , 1) , xyzsg(i + 1 , 1) , xyzsg(i + 1 , 1) , 60 );
    %display(length);
end