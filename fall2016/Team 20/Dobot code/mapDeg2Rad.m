function [ rad_q ] = mapDeg2Rad( deg_q )
%MAPRAD2BIT Summary of this function goes here
%   Detailed explanation goes here

rad_q = zeros(length(deg_q),1);

for i = 1:length(rad_q)

rad_q(i) = double(deg_q(i))*(pi/180);

end
rad_q = double(rad_q);

end

