function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    % Translate frame origins to base frame
    origin1_at0 = robot.pivot1;
    origin2_at0 = origin1_at0 + robot.pivot2;

    % rotate frames
    R1 = [cos(q(1)), -sin(q(1)); sin(q(1)), cos(q(1))];
    R2 = [cos(q(1) + q(2)), -sin(q(1) + q(2)); sin(q(1) + q(2)), cos(q(1) + q(2))];
    
    rotate_pivot2 = origin1_at0 + R1*robot.pivot2;

    pivot1 = origin1_at0;
    pivot2 = rotate_pivot2;

    % Compute link polygon corners
    link1_at0 = R1*robot.link1 + pivot1;
    % rotate link corners
    link2_at0 = R2*robot.link2 + rotate_pivot2;

    poly1 = polyshape(link1_at0(1,:), link1_at0(2,:));
    poly2 = polyshape(link2_at0(1,:), link2_at0(2,:));
end