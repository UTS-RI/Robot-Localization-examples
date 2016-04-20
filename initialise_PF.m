function particle = initialise_PF(n, pose, r, angle, l_num)
    % n = Number of particles
    % pose = x,y,phi -- centre pose of particles 
    % r = radius of random particles
    % angle = degree of difference from Robot orientation
        
   x = pose(1);
    y = pose(2);
    phi = pose(3);
    
    % Particle structure:-
    % pos = [x,y,phi]
    % weight = weight of particle
    % llh = likelihood of particle
    % laser = distances of each laser beam cast from particle, -90 to 90
    
    particle(n) = struct('pos',{zeros(3,1)},'weight',{0},'llh',{0},'mid',{0},'laser',{[zeros(3,l_num)]});
    for j = 1:n
        valid_particle = 0; % make sure the particle is not in the occupied area
        while valid_particle == 0
        particle(j).pos(1) = random_uniform(x-r,x+r);
        particle(j).pos(2) = random_uniform(y-r,y+r);
        particle(j).pos(3) = wrap(random_uniform(phi-angle,phi+angle));
        % check whether the particle is in the obstacle or not
        valid_particle = particle_validation(particle(j).pos(1),particle(j).pos(2));
        end
        particle(j).weight = 1/n;
        particle(j).laser = [0;0;0]; % set to 0 value first
    end
end