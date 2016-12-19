function [  ] = calibrate_dist( )
%CALIBRATE_DIST Summary of this function goes here
%   Detailed explanation goes here

L(1) = 0.053;
L(2) = 0.135;
L(3) = 0.160;
L(4) = 0.025;
L(5) = -0.017;

L = [L(1); L(4); L(5)];

[fval, L_opt] = fmincon(@calibrate_obj, L)


end

