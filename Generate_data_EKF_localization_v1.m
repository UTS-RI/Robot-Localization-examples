%% Generate data and perform EKF localization
%  this code generate data for EKF localization and then run EKF
%
% Data generated include: 
% landmark data -- landmarkxy
% control data -- control_input_mea
% observation data -- obs_range_bearing
% true robot pose data -- xstate_true (for comparison)
%
% Shoudong Huang, 2016 April
%
clc
clear all
close all

% set this value to 1 to try circular trajectory with 100 steps
test100=0;

%% noise level setting -- for both generating data and EKF formula
%control: velocity, turnrate
sig_v = 0.1;
sig_omega = 0.1;
% observation: range, bearing
sig_r = 0.1;
sig_theta = 0.1;

%% landmark setting: 4 landmarks, format: ID, x, y
landmarkxy = [1 2 5;
    2 2 -2;
    3 4 5;
    4 4 -2];
%% save the landmark data
save landmarkxy landmarkxy

% number of move steps (starts from time step 0)
num_steps = 4;

%% control inputs, format: time_step, velocity, turnrate
control_input_true = [0 1 0;
    1 0.7 pi/6;
    2 1.1  pi/6;
    3 1 0]

%% try long cicular trajectory
if test100==1
    num_steps=100;
    control_input_true = zeros(num_steps,3);
    for i=1:num_steps
        control_input_true(i,:)=[i-1, 1, 2*pi/num_steps];
    end
end

%% save the data
% save control_input_true control_input_true

%generating measured control inputs by adding noises (for EKF to use)
control_input_mea=control_input_true;
%control noises
noises_v=randn(num_steps,1)*sig_v;
noises_omega=randn(num_steps,1)*sig_omega;
control_input_mea(:,2)=control_input_mea(:,2)+noises_v;
control_input_mea(:,3)=control_input_mea(:,3)+noises_omega;
%% save the control data
save control_input_mea control_input_mea
%pause

%% generate ground true robot poses
% format: pose ID, x, y, phi
xstate_true = [0, zeros(1,3)]; % pose at time 0

for i=1:num_steps
    control_i = control_input_true(i,2:3);
    control_noise = [0;0];
    Delta_T = 1;
    xstatet1 = motionmodel(xstate_true(end,2:4),control_i,control_noise,Delta_T);
    xstate_true = [xstate_true; i xstatet1];
end

%% save xstate_true for comparison
save xstate_true xstate_true


%% generating observation data
% the observed landmark ID at each time step
% format: time_step ID1 ID2 (assume always see two landmarks for simplication)
obs_landmark_ID = [1 1 2;
    2 1 2;
    3 1 3;
    4 3 4];

%% try longer circular trajecotry
if test100==1
    obs_landmark_ID =  zeros(num_steps,3);
    for i=1:num_steps
        obs_landmark_ID(i,:)=[i, 1, 2];
    end
end
%% range and bearing observations
% format: time_step ID1 r1 theta1 ID2 r2 theta2
obs_range_bearing = [];
for i=1:num_steps
    landmark1=landmarkxy(obs_landmark_ID(i,2),2:3);
    landmark2=landmarkxy(obs_landmark_ID(i,3),2:3);
    
    % observation noises
    noise_r=randn*sig_r;
    noise_theta=randn*sig_theta;
    sensor_noise = [noise_r  noise_theta];
    
    % range-bearing to one landmark
    z1 = sensormodel(landmark1,xstate_true(i+1,2:4),sensor_noise);
    
    % observation noises
    noise_r=randn*sig_r;
    noise_theta=randn*sig_theta;
    sensor_noise = [noise_r  noise_theta];
    % range-bearing to another landmark
    z2 = sensormodel(landmark2,xstate_true(i+1,2:4),sensor_noise);
    %pause
    % save the obs data
    obs_range_bearing = [obs_range_bearing;i obs_landmark_ID(i,2) z1 obs_landmark_ID(i,3) z2];
end

%obs_range_bearing
save obs_range_bearing obs_range_bearing
%pause

%% prepare for EKF
%Define the noise covariances for EKF
% for process
Q=[sig_v^2 0
    0 sig_omega^2];
% for one landmark observation
R_i=[sig_r^2  0
    0   sig_theta^2];

% for recording the result

xstate_EKF = [0, zeros(1,3)]; % pose at time 0
P_EKF = 0.01*eye(3);  % initial covariance matrix

%% start recursive EKF estimation

for step = 1:num_steps
    
    disp('Running step');
    disp(step);
    disp('------------------------------------------------');
    %% get the data needed for one-step EKF
    % EKF estimate at time t
    xstate_t = xstate_EKF(end,2:4)'
    P_t = P_EKF(end-2:end,:)
    % pause
    % control input at time t
    control_t= control_input_mea(step,2:3);
    % observation data at time t+1
    obs_t1 = obs_range_bearing(step,2:end);
    %pause
    %discretization time interval
    Delta_T=1;
    % because observing two landmarks each step
    R = [R_i,zeros(2,2);zeros(2,2),R_i]; 
    %using EKF function
    [xstateT1_T1,PT1_T1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxy,Delta_T,Q,R);
    
    %update
    xstate_EKF = [xstate_EKF; step, xstateT1_T1];
    P_EKF = [P_EKF; PT1_T1];
end

%% save the EKF result
% save EKF_result xstate_EKF P_EKF

% error in estimate
error_xstate = xstate_EKF - xstate_true

%% draw the estimated robot poses and uncertainty ellipses

figure(1)
arrow_length=0.3;
%axis([-1 5 -3 6])
hold on

plot(landmarkxy(:,2),landmarkxy(:,3),'k*','linewidth',6)
text(landmarkxy(:,2)+0.1,landmarkxy(:,3)+0.4,num2str(landmarkxy(:,1)),'fontweight','bold','fontsize',14)

for i=0:num_steps
    uncer_p = P_EKF(i*3+1:i*3+2, 1:2);        % get the xy covariance
    
    uncer_x = xstate_EKF(i+1,2);
    uncer_y = xstate_EKF(i+1,3);
    CV=GetCov(uncer_p,uncer_x,uncer_y);  % by wangzhan, make it large on purpose, not now
    plot(CV(1,:),CV(2,:),'-b');
    
    plot(xstate_EKF(i+1,2),xstate_EKF(i+1,3),'bo','linewidth',2);
    % draw the robot heading
    dx = arrow_length*cos(xstate_EKF(i+1,4));
    dy = arrow_length*sin(xstate_EKF(i+1,4));
    quiver(xstate_EKF(i+1,2),xstate_EKF(i+1,3),...
        dx, dy, 0, 'Color', 'b','linewidth',1.2)
    
    %draw the true robot poses for comparison
    
    plot(xstate_true(i+1,2),xstate_true(i+1,3),'ro','linewidth',2);
    
    dx = arrow_length*cos(xstate_true(i+1,4));
    dy = arrow_length*sin(xstate_true(i+1,4));
    quiver(xstate_true(i+1,2),xstate_true(i+1,3),...
        dx, dy, 0, 'Color', 'r','linewidth',1.2)
    
    %pause
end
