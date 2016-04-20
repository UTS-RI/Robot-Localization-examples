%% Generate data for particle filter localization
%
% This code generate data for particle filter localization
%
% map is given by: data_PF/Lines.mat
% robot trajectory is given by: data_PF/robot_trajectory_true.mat
% true control data is given by: data_PF/vel_turn_data_true.mat
%
% Data generated include: 
% laser data: laser_data.mat
% control data: vel_turn_data.mat
%
% Shoudong Huang, 2016 April

clear all
close all

% variance of control noises
sig_v=0.2;
sig_w=0.2;
% variance for laser noise
sig_laser = 0.01;

% some parameters for laser 
laser_start = 90; % laser range goes from -90 to 90 degrees
laser_jump = 30; % laser scan every 30 degree
laser_num = (2*laser_start)/laser_jump + 1; %total number of scans
max_range = 20; % maximal laser range



% load the line data, the true robot trajectory and true control data
load data_PF/robot_trajectory_true.mat
load data_PF/vel_turn_data_true.mat
load data_PF/Lines.mat

linedata = [LineData;ULineData];

%% generate laser data
laser_data =[];

for k=1:length(robot_trajectory_true(:,1))
    robot.pose = robot_trajectory_true(k,:);
    %figure(k);
    % clf;
    % xlim([0 10]);ylim([0 10]);
    % draw_map()
    % plot(robot.pose(1),robot.pose(2),'ob'); %plot robot pose
%     dx = 0.5*cos(robot.pose(3));
%         dy = 0.5*sin(robot.pose(3));
%         quiver(robot.pose(1),robot.pose(2),dx, dy, 0, 'Color', 'b')
    
    laser_data_k=[];
    
    angle = -laser_start*(pi/180);
    
    for k = 1:laser_num
        [shortest_distance, x_found, y_found] = find_distance(max_range,linedata, robot.pose(1), robot.pose(2), wrap(robot.pose(3)+angle));
        laser_data_k=[laser_data_k shortest_distance];
        angle = angle + (laser_jump*(pi/180));
    end
    laser_noise = randn*sig_laser; % noise on laser reading
    laser_data =[laser_data; laser_data_k+laser_noise];
    %pause
end

save laser_data laser_data

%% generate control data
num_steps=size(vel_turn_data_true,1);

vel_turn_data=vel_turn_data_true;
% add noises to the true control data
vel_turn_data(:,2)=vel_turn_data(:,2)+randn(num_steps,1)*sig_v;
vel_turn_data(:,3)=vel_turn_data(:,3)+randn(num_steps,1)*sig_w;

save vel_turn_data vel_turn_data


