%% Robot localization with laser scan using particle filter
%
% This code perform particle filter localization using saved data
% for a simple environment -- robot moves a few steps
% Encyclopedia of EEE: Robot localization paper
%
% map is given by: data_PF/Lines.mat
%
% Data used include: 
% laser data: data_PF/laser_data.mat
% control data: data_PF/vel_turn_data.mat
%
% For comparison only:
% robot true trajectory: data_PF/robot_trajectory_true.mat
%
% Shoudong Huang, 2016 April

clear all
clf
close all

%% some parameters 
% variance of control noises
sig_v=0.2;
sig_w=0.2;

% likelihood variance for update step
sigma = 2;
%sigma = 10;

% parameters of laser
laser_start = 90; % laser range goes from -90 to 90 degrees
laser_jump = 30; % laser scan every 30 degree
laser_num = (2*laser_start)/laser_jump + 1; %total number of scans
max_range = 20; % maximal laser range

%% parameters for initialise particles

num_particles = 100; % number of particles
pf_radius = 5; % particle range (radius) 
pf_angle = 30*(pi/180); % angle range

% the centre pose for initialising particles
x_centre=2;
y_centre=-3;
phi_centre=pi/3;

%% load line data (map), the control data (velocity and turnrate) and actual laser data

% load the data
load data_PF/Lines.mat
load data_PF/laser_data.mat
load data_PF/vel_turn_data.mat

% for comparison only -- used in drawing figures
load data_PF/robot_trajectory_true.mat

laserdata = laser_data; % actual laser data
laserdata_size = size(laserdata,1)
num_steps = size(vel_turn_data,1)
%pause

linedata = [LineData;ULineData];

%% 

time_sum = 0;

resample_step = 1;

robot = struct('pos',{zeros(1,3)},'laser',{[zeros(3,laser_num)]});

%% Initialise particles centered around the given robot location
robot.initial(1)=x_centre;
robot.initial(2)=y_centre;
robot.initial(3)=phi_centre;

particle = initialise_PF(num_particles, robot.initial, pf_radius, pf_angle, laser_num);

%%fix the first particle to the correct one -- for debug
% 
% particle(1).pos(1)=x_centre;
% particle(1).pos(2)=y_centre;
% particle(1).pos(3)=phi_centre;

for step_no = 1:num_steps+1
    
    % draw the map   
    figure(2*step_no-1)
    draw_map()
    
    % draw Particles
    for j = 1:size(particle,2)
        plot(particle(j).pos(1),particle(j).pos(2),'go')
        hold on
        dx = 0.5*cos(particle(j).pos(3));
        dy = 0.5*sin(particle(j).pos(3));
        quiver(particle(j).pos(1),particle(j).pos(2),...
            dx, dy, 0, 'Color', 'g')
        %  plot(particle(j).laser(1,:),particle(j).laser(2,:),'g.-')
    end
       % plot true robot pose
        dx = 0.5*cos(robot_trajectory_true(step_no,3));
        dy = 0.5*sin(robot_trajectory_true(step_no,3));
        quiver(robot_trajectory_true(step_no,1),robot_trajectory_true(step_no,2),...
            dx, dy, 0, 'Color', 'b','linewidth',2)
        plot(robot_trajectory_true(step_no,1),robot_trajectory_true(step_no,2),'bo','linewidth',2)
    
        disp('------------------------------------------------');
disp('Running Particle Filter: Before update');
    pause
    
    % Get Robot Laser Scans
    robot.laser(3,:) = laserdata(step_no,:);
    
    % Raycast
    for j = 1:size(particle,2)
        angle = -laser_start*(pi/180);
                for k = 1:laser_num            
            [particle(j).laser(3,k) particle(j).laser(1,k) particle(j).laser(2,k)] = find_distance(max_range,linedata, particle(j).pos(1), particle(j).pos(2), wrap((particle(j).pos(3)+angle)));
            angle = angle + (laser_jump*(pi/180));
        end
    end
    
    % Update Particles
    particle = update_particles(particle, robot, sigma);
    
    % Estimate Robot
    robot_estimate = PF_estimate_robot_position(particle,1);
    
    % Resample
    %       
    %% compute N_eff to decide whether to do resampling or not
    
    sum_weight_sq = 0;
    for j = 1:num_particles
        sum_weight_sq = sum_weight_sq + particle(j).weight^2;
    end
     N_eff = 1/sum_weight_sq;  
    
    if N_eff < num_particles/2 % do resampling
    resample_method = 2;
    reample_var_xy = 0.1;
    resample_var_angle = 0.1;
    
    particle = resample_particles(particle,resample_method,reample_var_xy,resample_var_angle);
    end
    
    %% Plotting
    figure(2*step_no)
    % draw the map
    draw_map()
    % draw Particle
    for j = 1:size(particle,2)
        plot(particle(j).pos(1),particle(j).pos(2),'go')
        hold on
        dx = 0.5*cos(particle(j).pos(3));
        dy = 0.5*sin(particle(j).pos(3));
        quiver(particle(j).pos(1),particle(j).pos(2),...
            dx, dy, 0, 'Color', 'g')
        
      %  plot(particle(j).laser(1,:),particle(j).laser(2,:),'g.-')
    end
    % Robot
    % plot(robot.pos(1),robot.pos(2),'g.-');
    % plot(robot.laser(1,:),robot.laser(2,:),'m.-')
    
    % plot Estimated Robot
    plot(robot_estimate.pos(1),robot_estimate.pos(2),'b*','linewidth',2);
    dx = 0.5*cos(robot_estimate.pos(3));
        dy = 0.5*sin(robot_estimate.pos(3));
        quiver(robot_estimate.pos(1),robot_estimate.pos(2),...
            dx, dy, 0, 'Color', 'b','linewidth',2)
    
    % plot true robot pose
        dx = 0.5*cos(robot_trajectory_true(step_no,3));
        dy = 0.5*sin(robot_trajectory_true(step_no,3));
        quiver(robot_trajectory_true(step_no,1),robot_trajectory_true(step_no,2),...
            dx, dy, 0, 'Color', 'r','linewidth',2)
        plot(robot_trajectory_true(step_no,1),robot_trajectory_true(step_no,2),'ro','linewidth',2)
    
            disp('------------------------------------------------');
disp('Running Particle Filter: After update');
    
    step_no
    pause
    
    % Prediction
    if step_no<num_steps+1
        time = 1; % Time since previous data
        vel = vel_turn_data(step_no,2); % Velocity
        tr = vel_turn_data(step_no,3); % Turn Rate
       % particle = prediction(particle, vel, tr, time);
        particle = PF_prediction(particle, vel, tr, sig_v, sig_w, time);
    end    
end

 