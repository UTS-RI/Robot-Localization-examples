function draw_figure_for_scan_localization_prob()

% this is for drawing Figure 3 in the paper
% Encyclopedia of EEE: Robot localization paper
% -- localization problem using laser scan in an occupancy grid map
% Shoudong Huang, 2016 April

clear all
clf
close all

%% fixed particle position

x_fix=2;
y_fix=-3;
phi_fix=pi/3;

% robot trajectory
robot_trajectory=[2,-3,pi/3;2,-3,pi/2;2,-2,pi/2;2,-1,pi/2;2,0,pi/2;2,1,pi/2;2,1,pi/3];


%% load line data

load data_PF/Lines.mat

linedata = [LineData;ULineData];

%%%% parameters for initialsing particles

no_particles = 1; % number of particles
pf_radius = 2; % particle filter radius
pf_angle = 20*(pi/180); % particle filter angle

% laser range and number of scans
laser_start = 90; % laser range goes from -90 to 90
laser_jump = 1; % a laser beam every degree
laser_num = (2*laser_start)/laser_jump + 1; % number of laser beams
max_range = 20; % maximal laser range


robot = struct('pos',{zeros(1,3)},'laser',{[zeros(3,laser_num)]});


%% draw the lines

%   MAP_draw(LineData,'blue');
%  MAP_draw(ULineData,'red');

%% draw the occupied area

hold on
axis equal

X=[2.6;2.6;3.4;3.4];
Y=[-1.8; -3.3;-3.3;-1.8];

%  fill(X,Y,'k')
fill(X,Y,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X1=[-5;1.3;1.3;-5];
Y1=[1.7;1.7;7.5;7.5];
% fill(X1,Y1,'k')
fill(X1,Y1,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X2=[-5;1.3;1.3;-5];
Y2=[-2;-2;-9;-9];
%fill(X2,Y2,'k')
fill(X2,Y2,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X3=[1.3;1.3;3.6;3.6];
Y3=[7.5;5.7;5.7;7.5];
% fill(X3,Y3,'k')
fill(X3,Y3,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X4=[1.3;1.3;3.6;3.6];
Y4=[-5;-9;-9;-5];
%fill(X4,Y4,'k')
fill(X4,Y4,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X5=[5.3;5.3;6.7;6.7];
Y5=[-4.9;-9;-9;-4.9];
% fill(X5,Y5,'k')
fill(X5,Y5,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X6=[5.3;5.3;6.7;6.7];
Y6=[5.8;7.5;7.5;5.8];
%fill(X6,Y6,'k')
fill(X6,Y6,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X7=[5.3;5.3;6.7;6.7];
Y7=[-2.8;3.3;3.3;-2.8];
%fill(X7,Y7,'k')
fill(X7,Y7,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X8=[3.6;3.6;5.3;5.3];
Y8=[7.5;7.4;7.4;7.5];
%fill(X8,Y8,'k')
fill(X8,Y8,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');
pause


% Initialise particle
particle = initialise_PF(no_particles, robot.pos, pf_radius, pf_angle, laser_num);

% fix the particle to the start pose

particle(1).pos(1)=x_fix;
particle(1).pos(2)=y_fix;
particle(1).pos(3)=phi_fix;

% Raycast
for j = 1:size(particle,2)
    angle = -laser_start*(pi/180);
    for k = 1:laser_num
        [particle(j).laser(3,k) particle(j).laser(1,k) particle(j).laser(2,k)] = find_distance(max_range, linedata, particle(j).pos(1), particle(j).pos(2), (particle(j).pos(3)+angle));
        angle = angle + (laser_jump*(pi/180));
    end
end


%% Plotting
hold on;
% Particle and laser beam
for j = 1:size(particle,2)
    plot(particle(j).pos(1),particle(j).pos(2),'ro','linewidth',2)
    hold on
    dx = 0.5*cos(particle(j).pos(3));
    dy = 0.5*sin(particle(j).pos(3));
    quiver(particle(j).pos(1),particle(j).pos(2),...
        dx, dy, 0, 'Color', 'r','linewidth',2)
    
    plot(particle(j).laser(1,:),particle(j).laser(2,:),'r.-','LineWidth',2,...
        'MarkerFaceColor','g',...
        'MarkerSize',20)
end

% draw other poses
robot_trajectory_plot=robot_trajectory;
robot_trajectory_plot(6,:)=[];

for j=3:size(robot_trajectory_plot,1)
    plot(robot_trajectory_plot(j,1),robot_trajectory_plot(j,2),'bo','linewidth',2)
    hold on
    dx = 0.5*cos(robot_trajectory_plot(j,3));
    dy = 0.5*sin(robot_trajectory_plot(j,3));
    quiver(robot_trajectory_plot(j,1),robot_trajectory_plot(j,2),...
        dx, dy, 0, 'Color', 'b','linewidth',2)
end


end

