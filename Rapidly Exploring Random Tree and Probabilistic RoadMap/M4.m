% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles0
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found
% code by Francis Jacob Kalliath
function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    %Initializing the variables
    n = 5000;
    step_size = 0.25;
    i = 0;
    goal_sampling_freq = 0.25;
    % initialize the tree with the start configuration
    tree = zeros(n, 4);
    adjacency = zeros(n, n);
    tree(1, :) = q_start;

    path_found = false;
    pass = false;
    q_prev = q_start;
    edges = [];
    
    % generate n samples and attempt to connect them to the tree
    for i = 1:n
        if rand() < goal_sampling_freq
            q_target = q_goal;
        else
            %generating random samples
            q_target = M1(q_min, q_max, 1);
            if (check_collision(robot, q_target, link_radius, sphere_centers, sphere_radii))
                continue;
            end
        end
        % calculate the distances to all the configurations in the tree
        num_samples = size(tree, 1);
        distances = zeros(num_samples, 1);

        for j = 1:num_samples
            sample = tree(j, :);
            distance = norm(sample - q_target);
            distances(j) = distance;
        end
        %obtaining the closest neighbour
        [sorted_distances, sorted_indices] = sort(distances);

        nearest_point = sorted_indices(1);
        q_near = tree(nearest_point, :);

        q_new = q_near + step_size*(q_target-q_near)/norm(q_target - q_near);
        if ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)
            
            if check_edge(robot, q_new, q_near, link_radius, sphere_centers, sphere_radii)
                continue;
            end
            % add the new configuration to the tree and store the edge that connects it to its nearest neighbor
            tree = [tree; q_new];
            edges = [edges; [q_near, q_new]];
            if norm(q_new - q_goal) < 0.1
                    
                if ~check_edge(robot, q_new, q_goal, link_radius, sphere_centers, sphere_radii)
                    path_found = true;
                    edges = [edges; q_new, q_goal];
                    tree = [tree; q_goal];
                    break;
                end           
            end
        end
    end
        % check if the new configuration is close to the goal configuration
        if path_found
            path = [];
    
            path = [path; q_goal];
            q_current = q_goal;
                       
            while ~isequal(q_current, q_start)
                for i = 1:size(edges, 1)
                    edge_a = edges(i, :);
                    if isequal(edge_a(5:8),q_current)
                        disp(edge_a(5:8))
                        prev_node = edge_a(1:4);                       
                        path = [prev_node; path];
                        q_current = prev_node;
                    end
                end
            end
        end  
end