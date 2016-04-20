function [resampledParticles] = resample_particles(particle,method,var_xy,var_angle)

% % resample of the particles based on their weights
% % particles with higher weights will get more copies
% % particle with very low weights may be eliminated
%
% if method = 1, number of copies of each particle is proportional to its weights
% otherwise, a bit more random
%
% after resampling, also add some noises on the particles
%
% var_xy -- variance of noises to be added to x and y
% var_angle -- variance of noises to be added to angle

num_Particles=size(particle,2); % number of particles

if (method == 1) % the number of particles is almost proportional to the weights
    M = 1/num_Particles;
    
    U = random_uniform(0,M);
    S = 0;
    k = 0;
    
    for j = 1:num_Particles
        S = S + particle(j).weight;
        while S > U
            if k == 0
                resampledParticles = particle(j);
                %   j=j
                k = 1;
            else
                resampledParticles(end+1) = particle(j);
                %   j=j
                %             resampledParticles(1,k) = struct('pos',{zeros(3,1)},'weight',{0},'llh',{0},'mid',{0},'laser',{[zeros(1,181)]});
                %             resampledParticles(1,k) = particle(j);
            end
            U = U + M;
        end
    end
else
    % use a random number between 0 and 1 to decide which particle to copy
    for i = 1:num_Particles
        Weight(i)=particle(i).weight;
    end
    % Weight=Weight
    
    % cumulated sum of weights -- from 0 to 1:
    CDF = cumsum(Weight)/sum(Weight);
    %random choose between 0 and 1
    ind_Select  = rand(num_Particles,1);
    %find the particle that fall into the interval
    ind_NextGeneration = zeros(num_Particles,1);
    for j=1:num_Particles
        index_particle=find(ind_Select(j)<CDF,1,'first');
        ind_NextGeneration(j) = index_particle;
    end
    % copy selected particles for next generation..
    resampledParticles = particle(ind_NextGeneration);
end

num_p_resample= size(resampledParticles,2);
%pause

%
%% add some noises to the resampled particles

for j = 1:size(resampledParticles,2)
    
    valid_particle = 0; % make sure the particle is not in the occupied area
    while valid_particle == 0
        NewParticles(j).pos(1) = resampledParticles(j).pos(1) + random_normal(0,var_xy);
        NewParticles(j).pos(2) = resampledParticles(j).pos(2) + random_normal(0,var_xy);
        NewParticles(j).pos(3) = resampledParticles(j).pos(3) + random_normal(0,var_angle);
        
        % check whether the particle is valid or not
        valid_particle = particle_validation(NewParticles(j).pos(1),NewParticles(j).pos(2));
    end
    resampledParticles(j).pos(1) = NewParticles(j).pos(1);
    resampledParticles(j).pos(2) = NewParticles(j).pos(2);
    resampledParticles(j).pos(3) = NewParticles(j).pos(3);
    
end
end