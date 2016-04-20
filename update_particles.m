function particle = update_particles(particle, robot, sigma)

    % Likelihood
    n = size(particle,2);
    
    %extra = (1/(2*pi*sigma^2))^(n/2);
    
    for j = 1:n
    %     j
%         p_laser=particle(j).laser(3,:)
%         rob_laser= robot.laser(3,:)
diff_sq=(particle(j).laser(3,:)-robot.laser(3,:)).^2;

diff_sq_sort=sort(diff_sq);

diff_sq_5=diff_sq_sort(1:6);

%         
        particle(j).llh = exp(-sum(diff_sq_5)/(2*sigma^2));% * extra;
    % j_likilihood=particle(j).llh
%     
      %   pause
    end

 %    pause
    % New weight 1
    constant = 0;
    for j = 1:n
        particle(j).mid = particle(j).weight * particle(j).llh;
        constant = constant + particle(j).mid;
    end
    
%     % New Weight 2
%     [particle.mid] = [particle.weight]'.*[particle.llh]';
    
    % Normalise weight
    for j = 1:n
        particle(j).weight = particle(j).mid/constant;
%         j
%         p_weight_j=particle(j).weight
%         pause
    end
    
%     % Normalise weight 2
%     particle.weight = particle.mid/sum([particle.mid]);
    
end