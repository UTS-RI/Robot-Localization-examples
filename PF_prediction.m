function newparticle = PF_prediction(particle, vel, tr, sig_v, sig_w, time)%, mapy, mapx)
% prediction using the process model
%
% vel = velocity
% tr = turn rate
% sig_v = variance of velocity noise
% sig_w = variance of turn rate noise
% time = timestamp
% [mapx,mapy] = map dimensions

newparticle = particle;

for i = 1:size(particle,2)
    x = particle(i).pos(1);
    y = particle(i).pos(2);
    phi = particle(i).pos(3);
    
    vnoise = randn*sig_v;
    trnoise = randn*sig_w;
    %pause
    
    x = x + ((vel+vnoise)*time*cos(phi));
    y = y + ((vel+vnoise)*time*sin(phi));
    phi = wrap(phi + (tr+trnoise)*time);
    
    %% to make sure the particle is valid (using the map information)
    % depends on the map used, below is for the map used in the paper
    
    %% to make sure the particle is in the big rectangle area
    
    if x<1.4
        x=1.4;
    end
    if x>5.2
        x=5.2;
    end
    
    if y<-5.1
        y=-5.1;
    end
    if y>5.7
        y=5.7;
    end
    
    %% to make sure the particle is not in the small rectangular obstacle
    if x<3.5 && x>2.5 && y<-1.7 && y>-3.4
        if abs(x-3.5)<abs(x-2.5)
            x=3.5;
        else
            x=2.5;
        end
        if abs(y+1.7)<abs(y+3.4)
            y=-1.7;
        else
            y=-3.4;
        end
    end
    newparticle(i).pos(1) = x;
    newparticle(i).pos(2) = y;
    newparticle(i).pos(3) = phi;
end
end