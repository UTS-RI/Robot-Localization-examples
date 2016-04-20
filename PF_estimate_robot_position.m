function robot_estimate = PF_estimate_robot_position(particle, check)
% this function is to estimate the robot pose using the particles
% check = 1 -- use the weighted sum of all the particles
% otherwise (e.g. check=0) -- use the particle with the largest weight

robot_estimate = struct('pos',{zeros(3,1)});

if check == 1 % weighted sum of particles
    sum_x = 0;
    sum_y = 0;
    sum_cos_phi = 0; % use cos and sin to avoid pi and -pi issue
    sum_sin_phi = 0;
    for j = 1:size(particle,2)
        sum_x = sum_x + particle(j).pos(1)*particle(j).weight;
        sum_y = sum_y + particle(j).pos(2)*particle(j).weight;
        sum_cos_phi = sum_cos_phi + cos(particle(j).pos(3))*particle(j).weight;
        sum_sin_phi = sum_sin_phi + sin(particle(j).pos(3))*particle(j).weight;
    end
    robot_estimate.pos(1) = sum_x;
    robot_estimate.pos(2) = sum_y;
    average_cos_phi = sum_cos_phi;
    average_sin_phi = sum_sin_phi;
    robot_estimate.pos(3) = atan2(average_sin_phi,average_cos_phi);
else % particle with the highest weight
    ii = find(max([particle.weight]));
    robot_estimate.pos = particle(ii).pos;
end
