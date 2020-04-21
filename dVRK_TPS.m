function [Tx_l, Ty_l, Tx, Ty] = dVRK_TPS(I0_color, I1_color, Range1, Dist_XY)
    
    T_X_STAR = Range1(1,1);
    T_Y_STAR = Range1(1,2);
    T_X_SIZE = Range1(1,3);
    T_Y_SIZE = Range1(1,4);
    
    C_P_DIST_X = Dist_XY(1,1);
    C_P_DIST_Y = Dist_XY(2,1);
    
    display_grid_size = 30;
    max_iteration = 10;
    max_error = 0.01;
    lambda = 0.0;
    region_size = 5;
    start_frame = 1;

    %% Template
    % I0 = imread('./laparoscope_left0.jpg'); % face

    %I0_color = imread('v-repApi\left_image.jpg');

    I0 = rgb2gray(I0_color); % face

    Template = double(I0(T_Y_STAR:T_Y_STAR+T_Y_SIZE-1, T_X_STAR:T_X_STAR+T_X_SIZE-1));
    figure(1);
    imshow(uint8(Template));

    [Gx0, Gy0] = imgradientxy(Template);

    [Tx_l, Ty_l] = meshgrid(T_X_STAR:T_X_STAR+T_X_SIZE-1, T_Y_STAR:T_Y_STAR+T_Y_SIZE-1);
    
    % introduce RT and TT into the algorithm 
    [Tx, Ty] = meshgrid(T_X_STAR:T_X_STAR+T_X_SIZE-1, T_Y_STAR:T_Y_STAR+T_Y_SIZE-1);
    
    % select control points
    %C_Y_SIZE = round((T_Y_SIZE-1) / C_P_DIST_X) - 1;
    %C_X_SIZE = round((T_X_SIZE-1) / C_P_DIST_X) - 1;
    cx = T_X_STAR+C_P_DIST_X:C_P_DIST_X:T_X_STAR+T_X_SIZE-C_P_DIST_X/2;
    cy = T_Y_STAR+C_P_DIST_Y:C_P_DIST_Y:T_Y_STAR+T_Y_SIZE-C_P_DIST_Y/2;
    [Cx, Cy] = meshgrid(cx, cy);

    [c_r_size, c_c_size] = size(Cx);
    Cx = Cx(:);
    Cy = Cy(:);
    % TPS mapping
    MIK = thin_plate_spline(Cx(:), Cy(:), Tx(:), Ty(:), lambda);
    [p_size, c_size] = size(MIK);
    W = zeros(p_size, c_size * 2);

    %% simulation
    first = true;
    data_size = 100;
    error = zeros(data_size * max_iteration, 1);
    %I1 = double(rgb2gray(imread('v-repApi\right_image.jpg')));

    for k = start_frame+1:1:data_size
        % get iamges from video
    %     ob_2.CurrentTime = 4.2;
    %     I1 = rgb2gray(readFrame(ob_2));
    %     I1 = double(I1);
    %     [I0, I1] = rectifyStereoImages(I0,I1,stereoParams);

        I1 = double(rgb2gray(I1_color));

        for t = 1:max_iteration
            % 1st order, 2nd order   
            warped = interp2(I1, Tx, Ty, 'Bilinear');
            warped(isnan(warped)) = 0;
            % jacobian
            [Gx, Gy] = imgradientxy(warped);
            Ix = Gx(:) + Gx0(:);
            Iy = Gy(:) + Gy0(:);
            for i = 1:p_size
                for j = 1:c_size
                    W(i, j) = Ix(i) * MIK(i, j);
                    W(i, j + c_size) = Iy(i) * MIK(i, j);
                end
            end
            % dI
            It = warped(:) - Template(:);
            du = -2 * pinv(W) * (It);
            dx = du(1:c_size);
            dy = du(1 + c_size:2 * c_size);

            Cx = Cx + dx;
            Cy = Cy + dy;

            Tx = reshape(MIK * Cx(:), T_Y_SIZE, T_X_SIZE);
            Ty = reshape(MIK * Cy(:), T_Y_SIZE, T_X_SIZE);

            if sum(abs(du)) < max_error
                break
            end
        end
        error(k) = sum(abs(It));
        figure(2);
        imshow(uint8(warped));

        %% draw
        color = 0 ;
        color_2 = 255;
        % horizontal line
        for r = 1:display_grid_size:T_Y_SIZE
            for c = 1:T_X_SIZE
                I1(round(Ty(r, c)), round(Tx(r, c))) = color;
            end
        end
        % vertical line
        for r = 1:1:T_Y_SIZE
            for c = 1:display_grid_size:T_X_SIZE
                I1(round(Ty(r, c)), round(Tx(r, c))) = color_2;
            end
        end
        % control point
        for i = 1:c_size
            r = round(Cy(i));
            c = round(Cx(i));
            I1(r, c) = color;
            for d = 1:2
                I1(r-d, c-d) = color;
                I1(r-d, c+d) = color;
                I1(r+d, c-d) = color;
                I1(r+d, c+d) = color;
            end
        end
        figure(3);
        imshow(uint8(I1));
        drawnow;
        hold off;

        % save as a moving gif
        F = getframe(gcf);
        I = frame2im(F);
        [I,map] = rgb2ind(I, 256);
        gif_name = "TPS_tissue" + ...
                   num2str(T_Y_SIZE-1) + "x" + ...
                   num2str(T_X_SIZE-1) + "_" + ...
                   num2str(c_r_size) + "x" + ...
                   num2str(c_c_size);
        if abs(lambda) > 0.01
            gif_name = gif_name + "_" + num2str(lambda);
        end
        gif_name = char(gif_name + ".gif");
        if first
            imwrite(I,map,gif_name,'gif', 'Loopcount',inf,'DelayTime',1.0/30);
            first = false;
        else
            imwrite(I,map,gif_name,'gif', 'WriteMode','append','DelayTime',1.0/30);
        end

        if (k >= 2 && abs(error(k) - error(k-1)) < 0.5)
            break;
        end
    end
    figure(4);
    plot(2:data_size, error(2:data_size));


    %%
    % Dense 3D plot
    %{
    Point_l = ([Tx_l(:), Ty_l(:)]);
    Point_r = ([Tx(:), Ty(:)]);
    shape_points = triangulate(Point_l, Point_r, stereoParams) * [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];

    for i = 1 : size(shape_points, 1)
        for layer = 1 : 3
            Point_color(i, layer) = I0_color(round(Point_l(i, 2)), round(Point_l(i, 1)), layer);
        end
    end
    Point_color = double(Point_color);
    %}
%     figure(5);
%     scatter3(shape_points(:, 1), shape_points(:, 2), shape_points(:, 3), 10, Point_color/255);
% 
%     % set(gca,'XDir','reverse');
%     %set(gca,'YDir','reverse');
%     set(gca,'ZDir','reverse');
%     xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
%     ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
%     zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
%     grid on;

    %set(gca,'XDir','reverse');

end
