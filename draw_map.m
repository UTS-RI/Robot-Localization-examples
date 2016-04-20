function draw_map()

% draw the occupancy grid map for figure 3 in the
% Encyclopedia of EEE: Robot localization paper
% Shoudong Huang, 2016 April

clear all
clf
close all


%% load line data

load data_PF/Lines.mat

linedata = [LineData;ULineData];

% Draw Map

MAP_draw(LineData,'blue');
MAP_draw(ULineData,'red');

%  pause

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
Y3=[7.5;5.8;5.8;7.5];
% fill(X3,Y3,'k')
fill(X3,Y3,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X4=[1.3;1.3;3.6;3.6];
Y4=[-5;-9;-9;-5];
%fill(X4,Y4,'k')
fill(X4,Y4,'b','FaceAlpha', 0.2, 'EdgeColor', 'none');

X5=[5.3;5.3;6.7;6.7];
Y5=[-5;-9;-9;-5];
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
%pause

