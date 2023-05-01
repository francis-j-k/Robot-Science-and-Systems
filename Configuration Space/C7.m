% the method generates the position of the arm after padding the obstacle
function padded_cspace = C7(cspace)
    [n,m] = size(cspace);
    padded_cspace =cspace;
    %loop throught the adjacent cells
    for i = 1:n
        for j = 1:m
            if cspace(i,j) == 1
                % check the left cell
                if (i > 1)
                    padded_cspace(i-1, j) = 1;
                end
                % check right cell
                if (i < n)
                    padded_cspace(i+1, j) = 1;
                end
                % check the up cell
                if (j > 1)
                    padded_cspace(i, j-1) = 1;
                end
                % check the down cell
                if (j < m)
                    padded_cspace(i, j+1) = 1;
                end
                % check the diagonal cells 
                if (i > 1 && j > 1)
                    padded_cspace(i-1, j-1) = 1;
                end
                if (i > 1 && j < m)
                    padded_cspace(i-1, j+1) = 1;
                end
                if (i < n && j > 1)
                    padded_cspace(i+1, j-1) = 1;
                end
                if (i < n && j < m)
                    padded_cspace(i+1, j+1) = 1;
                end
            end
        end     
    end  
end