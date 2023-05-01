% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)
% code by Francis Jacob Kalliathcode by Francis Jacob Kalliath
function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    
    samples = zeros(num_samples, 4);
    adjacency = zeros(num_samples, num_samples);
    V_num = 0; 
    %loop to generate random samples
    while(V_num <= num_samples)
       random_sample = rand(1, 4) .* (q_max - q_min) + q_min;
       if (check_collision(robot, random_sample, link_radius, sphere_centers, sphere_radii) == 0)
            V_num = V_num + 1;
            samples(V_num, :) = random_sample; 
       end
    end
    for i = 1:num_samples
        dist = zeros(num_samples, 1);
        for j = 1:num_samples            
            dist(j) = norm(samples(i, :) - samples(j, :));
        end
        %sorting the vertices 
        [sort_dist, index] = sort(dist);
        if (size(sort_dist,1) < num_neighbors)
            num_neighbors = size(sort_dist,1);
        end
        %loop to denoting edges between roadmap vertices
        for k = 1:num_neighbors
            if (check_edge(robot,samples(i, :), samples(index(k), :), link_radius, sphere_centers, sphere_radii) == 0)
                adjacency(i, index(k)) = sort_dist(k);
                adjacency(index(k), i) = sort_dist(k);
            end
        end
    end

end