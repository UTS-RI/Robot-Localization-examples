function [H]= jacobi(xym,x,y)
a = (x-xym(1))/(sqrt((x-xym(1))^2+(y-xym(2))^2));
b = (y-xym(2))/(sqrt((x-xym(1))^2+(y-xym(2))^2));
c = 0;
d = -(y-xym(2))/(((y-xym(2))^2/(x-xym(1))^2+1)*(x-xym(1))^2);
e = 1/(((y-xym(2))^2/(x-xym(1))^2+1)*(x-xym(1)));
f = -1;
H = [a b c
     d e f];

