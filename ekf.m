function [xstate_t1,P_t1] = ekf(xstate_t,P_t,control_t,obs_t1,landmarkxym,Delta_T,Q,R);

disp('------------------------------------------------');
disp('Running Extended Kalman Filter');

%% prediction step

%motion model
pzero =[0 0];        %set noies equal to 0

%motion model estimate
[xstatet1_t] = motionmodel(xstate_t,control_t,pzero,Delta_T);
%pause
%predicted covariance matrix (uncertainty)
%jacobian matrix for F

temp = -Delta_T*control_t(1)*sin(xstate_t(3));
temp2 =  -Delta_T*control_t(1)*cos(xstate_t(3));
Jfx=[1 0 temp
    0 1 temp2
    0 0 1];

temp3 =  Delta_T*control_t(1)*cos(xstate_t(3));
temp4 =  Delta_T*control_t(1)*sin(xstate_t(3));

Jfw=[temp3 0; temp4 0; 0 Delta_T];

Pt1_t= Jfx*P_t*Jfx'+Jfw*Q*Jfw';                                          %uncertainty

%% update step

% get the observations
z_all = [obs_t1(2:3)';obs_t1(5:6)']; % two observations as a column vector
%
% get the landmark positions
landmark1=landmarkxym(obs_t1(1),2:3);
landmark2=landmarkxym(obs_t1(4),2:3);
nzero = [0 0];                                              %set noies equal to 0
z_pred_1 = sensormodel(landmark1,xstatet1_t,nzero);
z_pred_2 = sensormodel(landmark2,xstatet1_t,nzero);
% predicted observation
z_pred = [z_pred_1'; z_pred_2'];
% innovation
innov = z_all-z_pred;
% wrap the angles to [-pi, pi]
innov(2)=wrap(innov(2)); 
innov(4)=wrap(innov(4));
%pause

%jacobian matrix
Jh1 = jacobi(landmark1,xstatet1_t(1),xstatet1_t(2));
Jh2 = jacobi(landmark2,xstatet1_t(1),xstatet1_t(2));

Jh = [Jh1;Jh2];

S = Jh*Pt1_t*Jh'+R;
K = Pt1_t*Jh'*inv(S);

%result
xstatet1_t1 = xstatet1_t'+K*innov;
Pt1_t1 = Pt1_t - K*Jh*Pt1_t;

xstate_t1 = xstatet1_t1';
P_t1 = Pt1_t1;

end


