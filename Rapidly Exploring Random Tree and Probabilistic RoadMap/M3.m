% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found
% code by Francis Jacob Kalliath
function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
   %assigning the start and end points
   start_distance = zeros(size(samples,1),1);
   goal_distance  = zeros(size(samples,1),1);
   
   for i = 1:size(samples,1)
       start_distance(i) = norm(samples(i,:) - q_start);
       goal_distance(i)  = norm(samples(i,:) - q_goal);
   end
   %sort the start and end distances
   [~,sort_start_distance] = sort(start_distance); 
   [~,sort_goal_distance]  = sort(goal_distance);
   %identify the closest neighbor 
   start_neighbours = samples(sort_start_distance,:);
   goal_neighbours  = samples(sort_goal_distance,:);
   
   on_start = [];  
   %loop to check if there is collision for start and end neighbour values
   for i = 1:size(start_neighbours, 1)
        collision = check_edge(robot, q_start, start_neighbours(i, :), link_radius, sphere_centers, sphere_radii);        
        if ~collision
            on_start = sort_start_distance(i);
            break;
        end
   end
   off_goal = [];
   for i = 1:size(goal_neighbours, 1)  
        collision = check_edge(robot, q_goal,goal_neighbours(i, :), link_radius, sphere_centers, sphere_radii);
        if ~collision
     
            off_goal = sort_goal_distance(i);
            break;
        end    
   end
  
    path_indices = shortestpath(digraph(adjacency), on_start, off_goal);    
    path = samples(path_indices, :);
    disp(path);
    path = cat(1, q_start, path);
    path = cat(1, path, q_goal);

    if isempty(path_indices)
        path_found = false;
    else
        path_found = true;
    end
end
