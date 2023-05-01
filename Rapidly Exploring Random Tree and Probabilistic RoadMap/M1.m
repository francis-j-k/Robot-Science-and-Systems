% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits
% code by Francis Jacob Kalliath
function qs = M1(q_min, q_max, num_samples)
joints_number = length(q_min);    
    % sample generation
    qs = zeros(num_samples, joints_number);
    for i = 1:joints_number
        qs(:,i) = (q_max(i) - q_min(i)) * rand(num_samples, 1) + q_min(i);
    end
end