function point_data_set = multi_view_points_data_normalization(multi_view_coordinates_RCM)
    
    % ---- input a cell structure with different size of point data -------
    cell_size = size(multi_view_coordinates_RCM, 2);
    individual_cell_size = [];

    for i = 1 : cell_size 
        individual_cell_size(i, 1) = size(multi_view_coordinates_RCM{i}, 1);
    end

    Referecen_data_number = find(min(individual_cell_size) == individual_cell_size);
    Optimization_size = min(individual_cell_size);

    Interval_individual_cell = [];
    for i = 1 : cell_size   
        Interval_individual_cell(i, 1) = (size(multi_view_coordinates_RCM{i}, 1) / Optimization_size);
    end

    point_data_set = [];
    for j = 1 : Optimization_size
        for i = 1 : cell_size   
            column_number = round( (j - 1) * Interval_individual_cell(i, 1) + 1);
            point_data_set(i , : , j) = multi_view_coordinates_RCM{i}(column_number, :);

        end
    end

    % attach end points to the data set.
    % NOTE: there may exist some overlapped elements
    %
    End_point_data = [];
    for i = 1 : cell_size

        End_point_data(i, :) = multi_view_coordinates_RCM{i}(end, :);

    end

    point_data_set(:, :, end + 1) = End_point_data;

end