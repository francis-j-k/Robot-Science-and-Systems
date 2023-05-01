%method calculates the number of collisions
function num_collisions = C6(robot, obstacles, q_path)
    num_collisions = 0;
    [path_rows, ~] = size(q_path);   
    %Loop through each pair of consecutive configurations in the path
    for i = 2:path_rows
        q1 = q_path(i, :);
        q_prev = q_path(i-1, :);
        %Convert the current and previous configurations into polygonal representations
        [poly1, poly2, ~, ~] = q2poly(robot, q1);
        [poly1_prev, poly2_prev, ~, ~] = q2poly(robot, q_prev);
         % Get the union of the current and previous configurations
        c1 = convhull(union(poly1_prev,poly1));
        c2 = convhull(union(poly2_prev,poly2));
        for j = 1:length(obstacles)
            % Check if the union of the current and previous configurations intersects with the obstacle
            int_1 = intersect(c1, obstacles(j));
            int_2 = intersect(c2, obstacles(j));
            % If there is an intersection, increment the number of collisions
            if(int_1.NumRegions ~= 0||int_2.NumRegions ~= 0)
                num_collisions = num_collisions + 1;
            end            

        end
    end
end