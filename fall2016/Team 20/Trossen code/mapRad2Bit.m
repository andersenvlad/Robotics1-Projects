function [ bit_q ] = mapRad2Bit( rad_q )
%MAPRAD2BIT Summary of this function goes here
%   Detailed explanation goes here

bit_q = zeros(length(rad_q),1);

for i = 1:length(bit_q)
% base q1
bounds_d = [-150 150; -90 90; -90 90; -90 90; -90 90];
bounds_r = bounds_d.*(pi/180);
bounds_b = bounds_d.*(512/150);

% -150 deg = 0
% 150 deg = 1024

if rad_q(i) < bounds_r(i,1)
    rad_q(i) = bounds_r(i,1);
else if rad_q(i) > bounds_r(i,2)
    rad_q(i) = bounds_r(i,2);
    end

bit_q(i) = rad_q(i)*(180/pi)*(512/150) + 512;

bit_q = int16(bit_q);

end

end

