%The method calculates the distance transform from the grid nearest to the
%goal
function distances = C3(cspace, q_grid, q_goal)
    %initialize the distance matrix with zero
    distances = zeros(length(q_grid), length(q_grid));
    [~, goal_idx] = min(abs(q_grid - q_goal(1)));
    [~, goal_idy] = min(abs(q_grid - q_goal(2)));
    distances(goal_idx,goal_idy) = 2;
    [rows,cols] = size(cspace);
    %initializing the distance matrix to 1
    for i = 1:rows 
        for j = 1:cols
            if cspace(i,j) == 1
                distances(i,j) = 1;
            end
        end
    end
    L = [goal_idx,goal_idy];
    while(~isempty(L))
        neighbors = [];
        current_cell = L(1,:);
        L(1,:) = [];
        x = current_cell(1);
        y = current_cell(2);
        %the if loop to detect the adjacent cell values 
        if (x > 1)
            neighbors = [neighbors; x - 1, y]; 
        end
        % check right cell
        if (x < rows)
            neighbors = [neighbors; x + 1, y]; 
        end
        % check the up cell
        if (y > 1)
            neighbors = [neighbors; x , y-1];
        end
        % check the down cell
        if (y < cols)
            neighbors = [neighbors; x, y+1]; 
        end
        % check the diagonal cells 
        if (x > 1 && y > 1)
            neighbors = [neighbors; x - 1, y - 1];
        end
        if (x > 1 && y < rows)
            neighbors = [neighbors; x - 1, y + 1];
        end
        if (x < rows && y > 1)
            neighbors = [neighbors; x + 1, y - 1];
        end
        if (x < rows && y < cols)
            neighbors = [neighbors; x + 1, y + 1];
        end
        for k = 1:size(neighbors, 1)
            cell = neighbors(k,:);
            if distances(cell(1),cell(2)) ~=0 || distances(cell(1),cell(2)) == 1
                continue;
            end
            distances(cell(1),cell(2)) = distances(x,y) + 1;
            L = [L;cell];
        end        
    end      
   
end