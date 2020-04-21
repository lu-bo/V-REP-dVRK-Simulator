function draw_3D_suture_thread(key_point_camera_1, key_point_camera_2, stereoParams)


    %key_point_camera_1 = updateKeyPointsImage1;
    %key_point_camera_2 = updateKeyPointsImage2;

    %key_point_camera_1 = [keyPoints1(: , 2) keyPoints1(: , 1)];
    %key_point_camera_2 = [keyPoints2(: , 2) keyPoints2(: , 1)] ;
    %image_coordinate = [key_point_camera_1 key_point_camera_2];

    for i = 1 : size(key_point_camera_1, 1)
        z(i, 1) = (stereoParams.CameraParameters1.FocalLength(1) + stereoParams.CameraParameters1.FocalLength(2)) / 2 ...
                  * abs(stereoParams.TranslationOfCamera2(1)) / (key_point_camera_1(i, 1) - key_point_camera_2(i, 1));

        x(i, 1) = key_point_camera_1(i, 1) * z(i, 1) / ((stereoParams.CameraParameters1.FocalLength(1) + stereoParams.CameraParameters2.FocalLength(1)) / 2);
        y(i, 1) = key_point_camera_1(i, 2) * z(i, 1) / ((stereoParams.CameraParameters1.FocalLength(2) + stereoParams.CameraParameters2.FocalLength(2)) / 2);
    end
    %figure; scatter3(x(: , 1) , y(: , 1) , z(: , 1) , 20 ); hold on;

    %
    xyz = abs([x, y, z]);
    for i_begin_node = 1 : size(key_point_camera_1, 1)

        dis_i =  sqrt(sum((xyz - xyz(i_begin_node, :)) .^ 2, 2));
        marker = size(find(dis_i < 2), 1);

        if (marker > 10)
            break;
        end

    end
    %scatter3(xyz(i, 1), xyz(i, 2), xyz(i, 3), 80, 'r'); hold on;

    for j_end_node = size(key_point_camera_1, 1) : -1 : 1

        dis_i =  sqrt(sum((xyz - xyz(j_end_node, :)) .^ 2, 2));
        marker = size(find(dis_i < 2), 1);

        if (marker > 10)
            break;
        end

    end
    %scatter3(xyz(j, 1), xyz(j, 2), xyz(j, 3), 80, 'r'); hold on;
    %
    for ii = 1 : size(key_point_camera_1, 1)
        dis_i =  sqrt(sum((xyz - xyz(ii, :)) .^ 2, 2));
        near_element(ii, 1) = 1/ size(find(dis_i < 1), 1);
    end

    dis_coefficient = bsxfun(@times,near_element, near_element');


    %
    %Position = 10*rand(10,3);
    Position = [x, y, z];
    nodes_num = size(Position, 1);
    % Calculate euclid distance between each nodes
    d = pdist(Position);
    list = find(d > 2);

    % Set distance to 0 for 30 node-pairs
    %pt = randperm(numel(d),  3*nodes_num);
    d(list) = 0;
    % Convert to Adjacency matrix
    d = squareform(d);

    hyper_d = d.*dis_coefficient;
    % Generate Graph object and store XYZ coordinates in it
    G = graph(hyper_d);
    G.Nodes = array2table(Position,'VariableNames',{'X','Y','Z'});
    % Calculate shortest path between node 1 and 10
    [P,L] = shortestpath(G,i_begin_node,j_end_node);
    % Visualize the result
    %
    figure;
    h = plot(G,'XData',G.Nodes.X,'YData',G.Nodes.Y,'ZData',G.Nodes.Z);
    highlight(h,P,'EdgeColor','r','LineWidth',8);
    % Display the path length
    disp(L); grid on;
    %
    updaye_nodes = xyz(P, :);
    %
    %
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
    windowWidth = 5; %Standard example values
    polynomialOrder = 3;
    length_optimize = 0;

    xsg = sgolayfilt(updaye_nodes(:,1), polynomialOrder, windowWidth);
    ysg = sgolayfilt(updaye_nodes(:,2), polynomialOrder, windowWidth);
    zsg = sgolayfilt(updaye_nodes(:,3), polynomialOrder, windowWidth);
    xyzsg = [xsg, ysg, zsg];
    figure;
    hold on;
    for i = 1 : size(xyzsg, 1) - 1
        plot3([xyzsg(i, 1), xyzsg(i + 1, 1)], [xyzsg(i, 2), xyzsg(i + 1, 2)], [xyzsg(i, 3), xyzsg(i + 1, 3)], 'r', 'LineWidth', 7); hold on;
        %scatter3(xyzsg(i , 1) , xyzsg(i , 1) , xyzsg(i , 1) , 60 ); hold on;
        length_optimize = norm(xyzsg(i + 1, :) - xyzsg(i, :)) + length_optimize;
    end
    %scatter3(xyzsg(i + 1 , 1) , xyzsg(i + 1 , 1) , xyzsg(i + 1 , 1) , 60 );
    display(length_optimize);

    %title('Suture Thread Shape in 3D Space', 'fontsize', 12, 'fontweight', 'bold');
    xlabel('X/mm', 'fontsize', 25, 'fontweight', 'bold');
    ylabel('Y/mm', 'fontsize', 25, 'fontweight', 'bold'); 
    zlabel('Z/mm', 'fontsize', 25, 'fontweight', 'bold');
    hold on; grid on; box on
end