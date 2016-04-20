function valid_particle = particle_validation(particle_x,particle_y);

% this is to check whether the particle is valid or not 
% based on the map information -- depends on the map used
valid_particle = 1;

% invalid if the particle is not inside the big rectangle
if particle_x<1.4 || particle_x>5.2 || particle_y<-5.1 || particle_y>5.7
    valid_particle = 0;
% invalid if the particle is inside the small rectangle
elseif particle_x<3.5 && particle_x>2.5 && particle_y<-1.7 && particle_y>-3.4
    valid_particle = 0;
end

