function [ x, y ] = obs( c, r )
%OBS Summary of this function goes here
%   Detailed explanation goes here
angle = 0:0.01:2*pi;

xc = c(1);
yc = c(2);

x = r*cos(angle);
y = r*sin(angle);

x = x + xc;
y = y + yc;
%plot(x,y)


end

