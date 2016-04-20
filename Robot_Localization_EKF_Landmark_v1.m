%% EKF localization using generated data
%
% Encyclopedia of EEE: Robot localization paper -- robot moves 4 steps
%
% Data generated include: 
% landmark data -- landmarkxy
% control data -- control_input_mea
% observation data -- obs_range_bearing
% true robot pose data -- xstate_true (for comparison)
%
% Shoudong Huang, 2016 April
%
% use saved data; 

close all
clear all
clc

% set this value to 1 to try circular trajectory with 100 steps
test100=0;

%% noise level setting -- for both generating data and EKF formula
%control
sig_v = 0.1;
sig_omega = 0.1;
% observation
sig_r = 0.1;
sig_theta = 0.1;

% load the landmark data
load data_EKF_4_steps/landmarkxy
if test100==1
    load data_EKF_100_steps/landmarkxy
end

% load the control data
load data_EKF_4_steps/control_input_mea
if test100==1
    load data_EKF_100_steps/control_input_mea
end

% number of move steps (starts from time step 0)
num_steps = size(control_input_mea,1);

% load observation data
load data_EKF_4_steps/obs_range_bearing
if test100==1
    load data_EKF_100_steps/obs_range_bearing
end

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

%% initial estimate in EKF

for step = 1:num_steps
    
    disp('Running step');
    disp(step);
    disp('------------------------------------------------');
    %% get the data needed for one-step EKF
    % EKF estimate at time t
    xstate_t = xstate_EKF(end,2:4)';
    P_t = P_EKF(end-2:end,:);
    %pause
    % control input at time t
    control_t= control_input_mea(step,2:3);
    % observation data at time t+1
    obs_t1 = obs_range_bearing(step,2:end);
    
    %discretization time interval
    Delta_T=1;
    R = [R_i,zeros(2,2);zeros(2,2),R_i]; % because observing two landmarks each step
    %using ekf function
    [xstateT1_T1,PT1_T1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxy,Delta_T,Q,R);
    
    %update
    xstate_EKF = [xstate_EKF; step, xstateT1_T1];
    P_EKF = [P_EKF; PT1_T1];
end

%% save the result
% save EKF_result xstate_EKF P_EKF

% load the ground truth for comparison and drawing
load data_EKF_4_steps/xstate_true
if test100==1
    load data_EKF_100_steps/xstate_true
end

error_xstate = xstate_EKF - xstate_true

%% draw the estimated robot poses and uncertainty ellipses

figure(1)
arrow_length=0.3;
axis([-1 5 -3 6])
if test100==1
    axis([-25 25 -10 40])
end

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
