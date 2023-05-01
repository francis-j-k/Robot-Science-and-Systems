%The method will generate position of the robot
function C1(robot, q)
    [poly1, poly2, pivot1, pivot2] = q2poly(robot, q);
    % links are plotted
    plot(poly1, 'FaceColor', 'r');
    plot(poly2, 'FaceColor', 'b');
    % the pivot points are plotted
    plot(pivot1(1), pivot1(2), 'k.', 'MarkerSize', 10);
    plot(pivot2(1), pivot2(2), 'k.', 'MarkerSize', 10);
end