%method converts indices to 2d grid
function q_path = C5(q_grid, q_start, q_goal, path)
    [n,m] = size(path);
    q_path = zeros(n,m);
    q_path(1,:) = q_start;
    q_path(n,:) = q_goal;
    for i = 2:n-1
        q_path(i,:) = [q_grid(path(i,1)),q_grid(path(i,2))];
    
    end
end