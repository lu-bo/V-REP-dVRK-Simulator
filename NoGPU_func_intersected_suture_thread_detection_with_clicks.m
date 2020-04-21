function [all_points_set, suture_tip1]  = NoGPU_func_intersected_suture_thread_detection_with_clicks(image1 , image1_origin)

    %image1 = left;
    %image1_origin = left_rectified;
    
    %tic;
    %G = gpuArray(image1);
    outIm1 = FrangiFilter2D_NoGPU(double(image1));
    %[outIm1, whatScale1, Direction1] = FrangiFilter2D(double(G));
    %tCom = toc; disp(['Computational Time is: ' num2str(tCom)]);
    %figure; imshow(outIm1); pause(1); close;
    %
    
    %tic 
    outIm1 = gather(outIm1);
    biParameter1 = graythresh(outIm1); 
    Original_biOutIm1 = imbinarize(outIm1, biParameter1);
    %tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]);
    
    %tic
    %expansion_threshold = 1; di_kernal = ones(expansion_threshold, expansion_threshold, 'gpuArray');
    %expansion_threshold_2 = 3; er_kernal = ones(expansion_threshold_2, expansion_threshold_2, 'gpuArray');

    %diIm = imdilate(Original_biOutIm1 , di_kernal); diIm = imerode(diIm, er_kernal);
    % figure; imshow(diIm)

    % ---------------------------------------------------------------------
    % ----------------- add filter based on black image -------------------
    image_row_size = size(image1_origin, 1);
    image_col_size = size(image1_origin, 2);
    
    
    hsv_image = rgb2hsv(image1_origin);
    hsv_layer1 = hsv_image(:, :, 1); 
    hsv_layer2 = hsv_image(:, :, 2);
    hsv_layer3 = hsv_image(:, :, 3);
    layer1 = zeros(image_row_size, image_col_size); 
    layer2 = zeros(image_row_size, image_col_size);
    layer3 = zeros(image_row_size, image_col_size);
   
    Black_thre = 0.6;
    layer1(hsv_layer1 < Black_thre) = 1;
    layer2(hsv_layer2 < Black_thre) = 1;
    layer3(hsv_layer3 < Black_thre) = 1;
    
    Black_inverse = layer1 .* layer2 .* layer3; % figure; imshow(Black_inverse);

    % ---------------------------------------------------------------------
    % --------- filtering based on black color and frangi filter ----------
    % ========= the interested points should be 1 rather than 0 ===========
    % Black_inverse = gpuArray(Black_inverse); 
    % Original_biOutIm1 = gpuArray(Original_biOutIm1);
    
    %Black_inverse = Black * (-1) + 1;
    
    expansion_threshold = 2;
    er_kernal = ones(expansion_threshold, expansion_threshold);
    %er_kernal = ones(expansion_threshold, expansion_threshold, 'gpuArray');
    
    
    I = Original_biOutIm1 .* Black_inverse;
       
    biOutIm1 = I * (-1) + 1; %figure; imshow(biOutIm1); pause(1); close;
    biOutIm1 = gather(biOutIm1);
    biOutIm1 = imerode(biOutIm1, er_kernal);
    
    % write image data into files
    imwrite(double(biOutIm1), 'contour_pattern.jpg');

    % reload python function
    pyObj = py.importlib.import_module('contourCalculator'); % function --> 3 and 4 are sightly different.
    py.importlib.reload(pyObj); % python 3
    % py.reload(pyObj) % python 2
    
    detected_contours = py.contourCalculator.contour_function;
    contours_totoal_number = size(detected_contours, 2);
    
    % generate edge pattern
    max_row = size(biOutIm1, 1); max_col = size(biOutIm1, 2);
    
    edgeIm1 = ones(max_row, max_col);
    for contour_single_number = 1 : contours_totoal_number 
        data = double(py.array.array('d',py.numpy.nditer(detected_contours{contour_single_number}))); % d is for double, see link below on types
        data = reshape(data,[2 detected_contours{contour_single_number}.size/2])'; %Could incorporate x.shape here ...
    
        for ii = 1 : size(data, 1)
            if (data(ii, 2) < max_row && data(ii, 2) > 0 && data(ii, 1) < max_col && data(ii, 1) > 0)
                edgeIm1(data(ii, 2), data(ii, 1)) = 0;
            end
        end
    end
    %tCom = toc; disp(['Computational Time of Loop is: ' num2str(tCom)]);
    %figure; imshow(edgeIm1);
    %%
    %tCom = toc; disp(['Computational Time before click is: ' num2str(tCom)]);
    figure; imshow(image1_origin); hold on; title('Tip Point Indication'); suture_tip_point_image_1 = getrect; close;
    %tCom = toc; disp(['Computational Time after click is: ' num2str(tCom)]);
    %tic
    % ----- define the size centered by the clicked suture tip -----
    suture_area_length = 30; suture_area_width  = 30;
    
    % =========================================================================
    % in image 1 and 2, make iterative pattern
    % =========================================================================
    alpha1 = 1; alpha2 = 1;
    suture_local_area_image1 = round([...
        suture_tip_point_image_1(1) - alpha1 * suture_area_length, ...
        suture_tip_point_image_1(2) - alpha2 * suture_area_width,  ...
        2 * alpha1 * suture_area_length, 2 * alpha2 * suture_area_width]);
    %edgeIm1 = edgeIm1 * (-1) + 1;
    local_pattern_camera1 = imcrop(edgeIm1, suture_local_area_image1);
    %tCom = toc; disp(['Computational Time of Loop 1 is: ' num2str(tCom)]);
    %%
    %tic
    suture_tip1 = [];
    %local_pattern_camera1 = gpuArray(local_pattern_camera1); suture_local_area_image1 = gpuArray(suture_local_area_image1); edgeIm1 = gpuArray(edgeIm1);
    
    suture_tip1 = tip_detection_TMECH(local_pattern_camera1, suture_local_area_image1, edgeIm1);
    suture_tip1 = [suture_tip1(2), suture_tip1(1)];
    %figure; imshow(image1_origin); hold on; scatter(suture_tip1(1), suture_tip1(2), 80, 'r');
    %tCom = toc; disp(['Computational Time of Loop out is: ' num2str(tCom)]);
    %%
    
    min_dis = 100000;
    selected_contour_num = [];
    for contour_single_number = 1 : contours_totoal_number
        data = double(py.array.array('d',py.numpy.nditer(detected_contours{contour_single_number}))); % d is for double, see link below on types
        data = reshape(data,[2 detected_contours{contour_single_number}.size/2])'; % Could incorporate x.shape here ...
        current_min_dis_to_tip = min(vecnorm((data - suture_tip1)'));
        if (current_min_dis_to_tip < min_dis)
            min_dis = current_min_dis_to_tip;
            selected_contour_num = contour_single_number;
        end  
    end
    contour_members1 = double(py.array.array('d',py.numpy.nditer(detected_contours{selected_contour_num})));
    contour_members1 = reshape(contour_members1,[2 detected_contours{selected_contour_num}.size/2])';
    switch_value = contour_members1(:, 2);
    contour_members1(:, 2) = contour_members1(:, 1);
    contour_members1(:, 1) = switch_value;

    %hold on; scatter(contour_members1(:, 2), contour_members1(:, 1), 3, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    %pause(1); close;
    %%
    % Delete elements which are detected as contour members, but located on the
    % boundary. The criteria should include the elements that are 1 pixel away
    % from the boudary.
    
    del_num_row_min = find(contour_members1(:, 1) <= 1); del_num_row_max = find(contour_members1(:, 1) >= max_row - 1);
    del_num_col_min = find(contour_members1(:, 2) <= 1); del_num_col_max = find(contour_members1(:, 2) >= max_col - 1);
    contour_members1([del_num_row_min; del_num_row_max; del_num_col_min; del_num_col_max], :) = [];

    % Select elements with a specific interval
    clear updateKeyPointsImage1 updateKeyPoints1 patternEdgeCorner1;
    inter = 5; selected_k_p = contour_members1(1 : inter : end, :); Reversed_biOutIm1 = biOutIm1 * (-1) + 1;
    updateKeyPointsImage1(1, :) = [selected_k_p(1, 2) selected_k_p(1, 1)];
    Loop_num = size(selected_k_p, 1);
    %tCom = toc; disp(['Computational Time of Loop Begin is: ' num2str(tCom)]);
    
    %tic
    %edgeIm1 = gpuArray(edgeIm1);
    next_loop_edge_pattern = edgeIm1 * (-1) + 1;
    %parfor i = 2 : Loop_num
    for i = 2 : Loop_num
        if (selected_k_p(i, 1) < max_row - 1 && selected_k_p(i, 1) > 1 && ...
            selected_k_p(i, 2) < max_col - 1 && selected_k_p(i, 2) > 1 )  
            %
            [updateKeyPoints1(i, :), patternEdgeCorner1(i, :)] = optimized_keypoints_using_twopoints_TMECH_revision([selected_k_p(i - 1, 2), selected_k_p(i - 1, 1) ], ...
                                                                 [selected_k_p(i, 2), selected_k_p(i, 1)], next_loop_edge_pattern, Reversed_biOutIm1);

            updateKeyPointsImage1(i, :) = round([updateKeyPoints1(i, 1) + patternEdgeCorner1(i, 1), ...
            updateKeyPoints1(i, 2) + patternEdgeCorner1(i, 2)]);
            %
            %{
            [updateKeyPoints1, patternEdgeCorner1] = optimized_keypoints_using_twopoints_TMECH_revision([selected_k_p(i - 1, 2), selected_k_p(i - 1, 1) ], ...
                                                                 [selected_k_p(i, 2), selected_k_p(i, 1)], edgeIm1 * (-1) + 1, Reversed_biOutIm1);
            updateKeyPointsImage1(i, :) = round([updateKeyPoints1(1) + patternEdgeCorner1(1), ...
                                                 updateKeyPoints1(2) + patternEdgeCorner1(2)]);
            %}
        end
    end
    updateKeyPointsImage1 = [[selected_k_p(1 , 2) selected_k_p(1 , 1)] ; updateKeyPointsImage1];
    %tCom = toc; disp(['Computational Time of Loop Internal is: ' num2str(tCom)]);
    
    
    % ----- delect the number that are out of the image area              -----
    [delete_1, y] = find(updateKeyPointsImage1(: , 1) < 1);
    [delete_2, y] = find(updateKeyPointsImage1(: , 2) < 1);
    [delete_3, y] = find(updateKeyPointsImage1(: , 1) > max_col);
    [delete_4, y] = find(updateKeyPointsImage1(: , 2) > max_row);
    
    delete_set = [delete_1; delete_2; delete_3; delete_4];
    delete_set = unique(delete_set);
    updateKeyPointsImage1(delete_set, :) = [];
    %tCom = toc; disp(['Computational Time after 1st points adjustment is: ' num2str(tCom)]);
    %{
    i = 1; loop_num = size(updateKeyPointsImage1 , 1);
    while (i <= loop_num)
        if (updateKeyPointsImage1(i , 1) < 1 || updateKeyPointsImage1(i , 1) > max_col || ...
            updateKeyPointsImage1(i , 2) < 1 || updateKeyPointsImage1(i , 2) > max_row)
            updateKeyPointsImage1(i , :) = [];
            loop_num = loop_num - 1;
        end
        i = i + 1;
    end
    %}
    %figure ; imshow(Reversed_biOutIm1) ; hold on; scatter(updateKeyPointsImage1(: , 1), updateKeyPointsImage1(: , 2) , 5 , 'r')
    %%
    % set a tolerence parameter and remove the refined key points which are out
    % of the suture thread's body.
    %range = 4;
    %tolerance = range * range * 0.8;
    optimized_keypoints = [];
    for i = 1 : size(updateKeyPointsImage1, 1)
        %out_of_boundary(i , 1) = 0;
        %for near_row = - range : 1 : range
            %for near_col = - range : 1 : range
                if (Reversed_biOutIm1(updateKeyPointsImage1(i , 2), updateKeyPointsImage1(i , 1)) ~= 0)
                    optimized_keypoints = [optimized_keypoints ; updateKeyPointsImage1(i , :)];
                end
            %end
        %end
    
        %if (out_of_boundary(i , 1) <= tolerance)
            %optimized_keypoints = [optimized_keypoints ; updateKeyPointsImage1(i , :)];
        %end
    end
    updateKeyPointsImage1 = optimized_keypoints; updateKeyPointsImage1 = [suture_tip1; updateKeyPointsImage1];
    %figure ; imshow(Reversed_biOutIm1); hold on; scatter(updateKeyPointsImage1(: , 1) , updateKeyPointsImage1(: , 2) , 5 , 'r')
    %% 
    % -----        creterias usded to get "TRIM POINTS" set.              -----
    % ----- ============================================================= -----
    % ----- creteria one: trim the points which are too close to others.  -----
    %updateKeyPointsImage1 =  [selected_k_p(: , 2) , selected_k_p(: , 1)];
    TRIMMED_POINT = updateKeyPointsImage1(1 , :);

    POINT_SEQUENCE_FINAL = [];
    FINAL_ELEMENT_NUM = 0;
    % ---- important parameters ------
    % ---------------------------------------------------------------------
  
    THRE = 5;

    %THRE = 5 * TRIM_THRE;

    for i = 2 : size(updateKeyPointsImage1, 1)

        SIG = 0;
        for ID = 1 : size(TRIMMED_POINT, 1)
            DIST = norm(updateKeyPointsImage1(i, :) - TRIMMED_POINT(ID, :));
            if (DIST < THRE)
                SIG = 1;
                break;
            end
        end

        if (SIG == 0)% && r_biOutIm1(updateKeyPointsImage1(i , 2) , updateKeyPointsImage1(i , 1)) ~= 0)
            TRIMMED_POINT = [TRIMMED_POINT ; updateKeyPointsImage1(i, :)];
        end
    end

    TRIMMED_POINT = round(TRIMMED_POINT);
    %figure; imshow(Reversed_biOutIm1); hold on; scatter(TRIMMED_POINT(: , 1) , TRIMMED_POINT(: , 2) , 10 , 'r')
    %pause; close;

    %POINT_SEQUENCE_FINAL = [];
    %FINAL_ELEMENT_NUM = 0;

    % -----------------------------------------------------------------
    for angle_variation_group = 1 : 1
        for P1_variation = 1 : 1
            for P2_variation = 1 : 1    
                P1 = 10 * P1_variation;  % out of zone area point
                P2 = 0.2 * P2_variation;    % distance
                P3 = 0.05 * angle_variation_group;

                CURRENT_POINT  = TRIMMED_POINT(1, :);
                POINT_SEQUENCE = []; 
                POINT_SEQUENCE = [POINT_SEQUENCE; CURRENT_POINT]; % initialize the points in new sequence
                LAST_STEP_COORDINATES = suture_tip1;

                RADIUS = THRE * 10; % set evaluation range within all the trimmed points
                thre_out_of_zone = round(RADIUS / 3);
                thre_angle_variation = 60;

                LOOP_CANDIDATE = TRIMMED_POINT;
                LOOP_CANDIDATE (1, :) = []; % set the evaluation candidate. remove the first row
                i = 1;

                %{
                figure; imshow(r_biOutIm1);
                hold on; scatter(CURRENT_POINT(1) , CURRENT_POINT(2) , 'r');
                %}

                while (true)
                % for i = 1 : size(TRIMMED_POINT, 1) - 1
                % ----- Construct the two evaluation matrixes, one contains----
                % ----- the coordinates, and the other contains the row    ----
                % ----- number information.                                ----
                STRUCT_EVA_CANDIDATE = [];
                EVA_CANDIDATE = [];
                EVA_CANDIDATE_ROWNUM = [];

                % ----- Picking up the neareset neigbours from the trimmed -----
                % ----- points based on the previous calculations.         -----
                % ----- ================================================== -----
                % ----- We define "LOOP_CANDIDATE" set which is updated in -----
                % ----- each iteration, it remove the newest current point -----
                % ----- that the found points will not be duplicatly       -----
                % ----- evaluated.                                         -----
                for ID = 1 : size (LOOP_CANDIDATE, 1)
                    DIST = norm (LOOP_CANDIDATE(ID, :) - CURRENT_POINT); % distance between all other trimmed the points and the current one

                    % ----- If the point within certain range, which is    -----
                    % ----- defined by "RADIUS", this point is considered  -----
                    % ----- as the evaluation candidates.                  -----
                    if (DIST < RADIUS)
                        EVA_CANDIDATE = [EVA_CANDIDATE; LOOP_CANDIDATE(ID, :)]; % add the candidate to the evaluation set
                        EVA_CANDIDATE_ROWNUM = [EVA_CANDIDATE_ROWNUM; ID]; % record the cooresponding row number
                    end
                end

                if (size(EVA_CANDIDATE, 1) == 0)
                    break;
                end

                STRUCT_EVA_CANDIDATE = [EVA_CANDIDATE, EVA_CANDIDATE_ROWNUM];

                % select the next point from the candidates
                CAN_NUM = size(EVA_CANDIDATE, 1); % pick up the number of the next point candidate

                % ----- The algorithm used to pick up the next point from the     -----
                % ----- nearest candidates obtained above .                       -----
                % ----- ========================================================  -----
                % ----- Hierarchical evaluation creterias:                        -----
                % ----- 1. Select the lines that contains the least number of     -----
                % -----    black pixels;                                          -----
                % ----- 2. Select the nearest element.                            -----

                % -----                  Creteria 1                               -----
                OUT_ZONE_NUM = zeros (CAN_NUM, 1);
                for j = 1 : CAN_NUM
                    % ----- find pixel coordinates betewwen two points.           -----
                    % ----- plot coordinate format:                               -----
                    % ----- "EVA_CANDIDATE(j , 1), EVA_CANDIDATE (j , 2)"         -----
                    % -----                  ^                      ^             -----
                    % -----                  |                      |             -----
                    % -----                  x (col)                y (row)       -----
                    w = EVA_CANDIDATE(j, 1) - CURRENT_POINT(1, 1);
                    h = EVA_CANDIDATE(j, 2) - CURRENT_POINT(1, 2);
                    SLOP = h / w;
                    % OUT_ZONE_NUM (j , 1) = 0;
                    % subpoint = [];

                    if (norm (w) >= norm (h))
                       for STEP = w/norm(w) * 1 : w/norm(w) : w

                           x_coordinate = CURRENT_POINT(1, 1) + STEP;
                           y_coordinate = CURRENT_POINT(1, 2) + round(STEP * SLOP);

                           %subpoint = [subpoint; x_coordinate , y_coordinate];

                           if (Reversed_biOutIm1(y_coordinate, x_coordinate) == 0)
                               OUT_ZONE_NUM (j) = OUT_ZONE_NUM (j) + 1;
                           end

                       end
                    end

                    if (norm (w) < norm (h))
                       for STEP = h/norm(h) * 1 : h/norm(h) : h

                           x_coordinate = CURRENT_POINT(1, 1) + round(STEP / SLOP);
                           y_coordinate = CURRENT_POINT(1, 2) + STEP;

                           %subpoint = [subpoint; x_coordinate , y_coordinate];

                           if (Reversed_biOutIm1(y_coordinate, x_coordinate) == 0)
                               OUT_ZONE_NUM (j) = OUT_ZONE_NUM (j) + 1;
                           end

                       end
                    end

                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, OUT_ZONE_NUM];

                % -----                  Creteria 2                               -----
                % ----- evaluate the distance between the candidate and the       -----
                % ----- the current point.                                        -----
                DISTANCE_TO_CURRENT_POINT = [];
                for II = 1 : size(STRUCT_EVA_CANDIDATE, 1)
                    DIST = round(norm( STRUCT_EVA_CANDIDATE(II, 1 : 2) - CURRENT_POINT ));

                    DISTANCE_TO_CURRENT_POINT = [DISTANCE_TO_CURRENT_POINT; DIST];
                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, DISTANCE_TO_CURRENT_POINT];

                % -----                  Creteria 3                               -----
                % ----- the slop variance should be considered                    -----
                if (i == 1)
                    SLOP_VARIANCE = ones(CAN_NUM, 1);
                end

                if (i > 1)
                    SLOP_VARIANCE = ones(CAN_NUM, 1);
                    for j = 1 : size(STRUCT_EVA_CANDIDATE, 1)
                        CURRENT_VECTOR = [STRUCT_EVA_CANDIDATE(j, 1 : 2) - LAST_STEP_COORDINATES, 0];
                        %CURRENT_SLOP = (STRUCT_EVA_CANDIDATE(j , 2) - LAST_STEP_COORDINATES(2)) / (STRUCT_EVA_CANDIDATE(j , 1) - LAST_STEP_COORDINATES(1));
                        %CURRENT_ANGLE = atan(CURRENT_SLOP) * 180 / pi
                        %SLOP_VARIANCE(j , 1) = abs(CURRENT_ANGLE - LAST_STEP_ANGLE);
                        SLOP_VARIANCE(j, 1) = abs(atan2d(norm(cross(CURRENT_VECTOR, LAST_VECTOR)), dot(CURRENT_VECTOR, LAST_VECTOR)));
                    end
                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, SLOP_VARIANCE];

                % -----        set parameters for 3 creterias                     -----
                RANKING_MATRIX = [];

                % P3 = 0.2;  % angle variance 
                for ALLCAN = 1 : size(STRUCT_EVA_CANDIDATE, 1)
                    RANK = P1 * STRUCT_EVA_CANDIDATE(ALLCAN, 4) + P2 * STRUCT_EVA_CANDIDATE(ALLCAN, 5) + exp(P3 * STRUCT_EVA_CANDIDATE(ALLCAN, 6));
                    RANKING_MATRIX = [RANKING_MATRIX; RANK];
                end

                STRUCT_EVA_CANDIDATE = [STRUCT_EVA_CANDIDATE, RANKING_MATRIX];

                MARK = find(STRUCT_EVA_CANDIDATE(:, 7) == min(STRUCT_EVA_CANDIDATE(:, 7)));
                MARK = MARK(1);

                % ----- added criteria: if the minimal number of the out-of-zone  -----
                % ----- element is larger than a threshold, it means all candidate-----
                % ----- are not qualified, and now all candidates should belong   -----
                % ----- to another region, but within the search area we set.     -----
                if (STRUCT_EVA_CANDIDATE(MARK, 4) > thre_out_of_zone)
                    break;
                end

                if (STRUCT_EVA_CANDIDATE(MARK, 6) > thre_angle_variation)
                    break;
                end

                SELECT_POINT = STRUCT_EVA_CANDIDATE(MARK, 1 : 2);
                SELECT_ROW = STRUCT_EVA_CANDIDATE(MARK, 3);

                LOOP_CANDIDATE (SELECT_ROW, :)=[]; % remove the current point from the loop set using the recorded row number
                CURRENT_POINT = SELECT_POINT; 
                POINT_SEQUENCE = [POINT_SEQUENCE; CURRENT_POINT];

                %CURRENT_SLOP = (CURRENT_POINT(2) - LAST_STEP_COORDINATES(2)) / (CURRENT_POINT(1) - LAST_STEP_COORDINATES(1));
                %CURRENT_ANGLE = atan(CURRENT_SLOP) * 180 / pi;
                %LAST_STEP_ANGLE = CURRENT_ANGLE;
                LAST_VECTOR = [CURRENT_POINT - LAST_STEP_COORDINATES, 0];
                LAST_STEP_COORDINATES = CURRENT_POINT;

                %{
                scatter(CURRENT_POINT(1, 1), CURRENT_POINT(1, 2), 40 , 'r');
                hold on; pause(0.03);
                %}
                i = i + 1; 

                end

                %size(POINT_SEQUENCE, 1) pause(0.5);
                if (size(POINT_SEQUENCE, 1) > FINAL_ELEMENT_NUM)
                    FINAL_ELEMENT_NUM = size(POINT_SEQUENCE, 1);
                    POINT_SEQUENCE_FINAL = [];
                    POINT_SEQUENCE_FINAL = POINT_SEQUENCE;
                end
    % -----------------------------------------------------------------             
            end
        end
    end
    % ---------------------------------------------------------------------           

    %figure; imshow(Reversed_biOutIm1); hold on; scatter(POINT_SEQUENCE_FINAL(: , 1) , POINT_SEQUENCE_FINAL(: , 2) , 20 , 'r')
    POINT_SEQUENCE = [suture_tip1 ; POINT_SEQUENCE];
    %%
    % point_set_optimizer
    angle_opti_threshold = 5;

    for point_num = 2 : size(POINT_SEQUENCE_FINAL, 1) - 1

        point_last    = POINT_SEQUENCE_FINAL(point_num - 1, :);
        point_current = POINT_SEQUENCE_FINAL(point_num    , :);
        point_next    = POINT_SEQUENCE_FINAL(point_num + 1, :);

        point_current_AX = point_current;

        u = [point_current - point_last, 0];
        v = [point_next - point_current, 0];

        radium_opti = norm(u);

        angle_variation = atan2(norm(cross(u, v)), dot(u, v)) * 180 / pi;

        if (angle_variation > angle_opti_threshold)

            center_opti = point_last;
            angles_opti = 0 : 2 * pi / 360 : 2 * pi;

            points_opti = round(radium_opti * [cos(angles_opti') sin(angles_opti')] + center_opti(1 : 2));

            for i = 1 : size(points_opti, 1)

                if (points_opti(i, 1) <= max_col &&...
                    points_opti(i, 2) <= max_row &&...
                    Reversed_biOutIm1(points_opti(i, 2), points_opti(i, 1)) == 1)

                    u_opti = [points_opti(i, :) - point_last, 0];
                    v_opti = [point_next - points_opti(i, :), 0];

                    angle_when_opti = atan2(norm(cross(u_opti, v_opti)), dot(u_opti, v_opti)) * 180 / pi;

                    if (angle_when_opti < angle_variation)
                        angle_variation = angle_when_opti;
                        point_current_AX = points_opti(i, :);
                    end
                end

            end 
        end

        POINT_SEQUENCE_FINAL(point_num, :) = (point_current + point_current_AX) / 2;

    end

    suture_local_area_image1 = round([...
        POINT_SEQUENCE_FINAL(end, 1) - alpha1 * suture_area_length, ...
        POINT_SEQUENCE_FINAL(end, 2) - alpha2 * suture_area_width,  ...
        2 * alpha1 * suture_area_length, 2 * alpha2 * suture_area_width]);
    local_pattern_camera1 = imcrop(edgeIm1, suture_local_area_image1);
    suture_end_optimized = tip_detection_TMECH(local_pattern_camera1, suture_local_area_image1, edgeIm1);

    POINT_SEQUENCE_FINAL = [POINT_SEQUENCE_FINAL; [suture_end_optimized(2), suture_end_optimized(1)]];
    %tCom = toc; disp(['Computational Time before illustration is: ' num2str(tCom)]);
    %figure; imshow(Reversed_biOutIm1); hold on; scatter(POINT_SEQUENCE_FINAL(:, 1) , POINT_SEQUENCE_FINAL(:, 2) , 20 , 'b')
    %%
    %tic
    %POINT_SEQUENCE_FINAL = gpuArray(POINT_SEQUENCE_FINAL);
    all_points_set = map_shape_on_points(POINT_SEQUENCE_FINAL);
    %tCom = toc; disp(['Computational Time before illustration is: ' num2str(tCom)]);
    %%
    %figure; imshow(image1_origin); hold on;
    
    %all_points_set = gpuArray(all_points_set);
    all_points_set = gather(all_points_set);
    %dc=hsv(length(all_points_set));
    
    %total_point_for_show = size(all_points_set, 1) - 1;
    %line(all_points_set(:, 1), all_points_set(:, 2), 'Color', [1 0 0], 'linewidth', 2);
    
    %for i = 1 : total_point_for_show
    %    line([all_points_set(i, 1) all_points_set(i + 1 , 1)], [all_points_set(i, 2) all_points_set(i + 1, 2)] , 'Color', dc(i, :), 'linewidth', 2);
    %end
    %tCom = toc; disp(['Computational Time of illustration is: ' num2str(tCom)]);
end