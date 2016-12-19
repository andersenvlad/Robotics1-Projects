function [ q_deg ] = inverse_kinematics( P0T )
%INVERSE_KINE Summary of this function goes here
%   Detailed explanation goes here

[ P, H ] = get_arm_param( );

P01 = P(:,1);
P12 = P(:,2);
P23 = P(:,3);
P34 = P(:,4);
P4T = P(:,5);

% R04 is known so we can calulate P4T in the 0 frame

k1 = H(:,1);
k2 = H(:,2);
k3 = H(:,3);
k4 = H(:,4);


% get q1
q1 = atan(P0T(2)/P0T(1));
R01 = rot(k1, q1);

out = P0T-P01-R01*P4T;
a = norm(P34);
b = norm(P23);
c = norm(out);

C = acos((c^2 - a^2 - b^2)/(-2*a*b));

A = asin(a*sin(C)/c);

B = pi - A - C;

th = atan(out(3)/norm([out(1); out(2)]));

q2 = pi/2 - th - A;
q3 = pi/2 + q2 - C;
q4 = 0;
q_rad = [q1; q2; q3; q4];

q_deg = mapRad2Deg(q_rad);




end

