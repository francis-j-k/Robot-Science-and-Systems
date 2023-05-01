%The method generates the path
function path = C4(distances, q_grid, q_start)
    [~, start_x] = min(abs(q_grid - q_start(1)));
    [~, start_y] = min(abs(q_grid - q_start(2)));
    [rows,cols] = size(distances);
    checked = zeros(rows,cols);
    for i = 1:rows
        for j = 1:cols
            if distances(i,j) == 2
                q_goal = [i,j];
            end
        end
    end
    neighbors = [];
    current = [start_x,start_y];
    path = current;
    x = start_x;
    y = start_y;
    while ~(current == q_goal)
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
        neighbor_Len=size(neighbors,1);
        minDist = inf;
        %search and update minimum distance
        for k = 1:neighbor_Len
            if (distances(neighbors(k,1),neighbors(k,2))<minDist && distances(neighbors(k,1),neighbors(k,2)) ~= 1)
                if checked(neighbors(k,1),neighbors(k,2)) ~= 1
                    minDist = distances(neighbors(k,1),neighbors(k,2));
                    current = [neighbors(k,1),neighbors(k,2)];
                end
            end
        end
        path = [path; current];
        x = current(1);
        y = current(2);
        checked(current) = 1;
        
    end

end
