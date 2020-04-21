function optimized_point = kalman_filter_multiple_points_optimization(input_points_data)
    
    Data_size = size(input_points_data, 1);
    % initialize the parameters
    x = []; % Current estimated value x_{n,n}
    X = []; % State Extrapolation x_{n, n-1}
    p = []; % Covariance Update
    P = []; % Covariance Extrapolation
    K = []; % Kalman gain
    Measure = []; % Visual sensor feedback

    x = input_points_data(1, :); % input the first element to initialize
    Measure = x; % the first measurement value equals to the initialization value
    
    p = 1; % Uncertainty in estimation p_(n,n-1), covariance at the initial step
    r = 0.1; % Covariance - uncertainty in measurement --> relative larger
    Q = 0.0001; % Process noise - simulation should be a small number
    
    P = p + Q; % The extrapolated estimate uncertainty / covariance 
    K = P / (P + r); % Kalman gain update
    X = x; % Constant dynamics(no dynamic motion)

    for i = 2 : Data_size
        
        Measure(i, :) = (input_points_data(i , :)); 
        x(i, :) = X(i - 1, :) + K(i - 1, :) * (Measure(i, :) - X(i - 1, :));
        p(i, :) = (1 - K(i - 1, :)) * P(i - 1, :);

        P(i, :) = p(i, :) + Q;
        K(i, :) = P(i, :) / (P(i, :) + r);
        X(i, :) = x(i, :);

        % hold on;
        % input_xyz = multi_view_coordinates_RCM{i};
        % plot3(input_xyz(:, 1), input_xyz(:, 2), input_xyz(:, 3), 'k', 'LineWidth', 3); hold on;
        %scatter3(multi_view_coordinates_RCM{i}(1, 1), ...
        %         multi_view_coordinates_RCM{i}(1, 2), ...
        %         multi_view_coordinates_RCM{i}(1, 3), 30, 'b')

        %starting_point_dis(i, :) = norm(multi_view_coordinates_RCM{i}(1, :)); 
        %end_point_dis(i, :) = norm(multi_view_coordinates_RCM{i}(end, :));
    end
    
    optimized_point = X(end, :);
    
end