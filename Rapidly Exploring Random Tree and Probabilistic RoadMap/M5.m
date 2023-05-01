% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

% the logic for the algorithm is:
% qstart= start point
% qend= end point
% check if from qstart to qend there is collision if yes then break else assign the point to point to smoothen
% repeat the loop to geth a smoothened path from start to end:
% %code by Francis Jacob Kalliathcode by Francis Jacob Kalliath
function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    smoothed_path =[];
    smoothed_path = [smoothed_path; path(1,:)];
    start = 1;
    end_ = size(path,1); 
    %loop to check if there is collision for the points between start and end
    while (start < end_)   
        %check if there is edge collision
        if ~check_edge(robot, path(start,:), path(end_,:), link_radius, sphere_centers, sphere_radii)
            smoothed_path = [smoothed_path; path(end_,:)];
            start = end_;
            end_ = size(path,1);
        else
            end_ = end_ -1;            
        end
    end
end