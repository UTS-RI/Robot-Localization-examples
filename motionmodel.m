%nonlinear discrete time dynamic system motion model
function [xstatet1] = motionmodel(xstate,cin,pn,t)
xstatet1(1)=xstate(1)+(cin(1)+pn(1))*t*cos(xstate(3));
xstatet1(2)=xstate(2)+(cin(1)+pn(1))*t*sin(xstate(3));
xstatet1(3)=xstate(3)+(cin(2)+pn(2))*t;
end
