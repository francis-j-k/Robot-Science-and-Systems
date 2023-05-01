function cspace = C2(robot, obstacles, q_grid)
    % the rotation angle is between 0 and 2pi
    cspace = zeros(length(q_grid), length(q_grid));
    for i = 1:length(q_grid)
        for j = 1:length(q_grid)
            % check for multiple angles of q1 and q2
            q = [q_grid(i); q_grid(j)];
            [poly1, poly2, pivot1, pivot2] = q2poly(robot, q);
            for k = 1:length(obstacles)
                intersect_1 = intersect(poly1, obstacles(k));
                intersect_2 = intersect(poly2, obstacles(k));
                if isObstacle(intersect_1, intersect_2)
                    cspace(i,j) = 1;
                    break;
                end
            end
        end
    end
end
% method to check if obstacle is present
function result = isObstacle(intersect_1, intersect_2)
    if intersect_2.NumRegions > 0 || intersect_1.NumRegions > 0
        result = 1;
    else
        result = 0;
    end
end
