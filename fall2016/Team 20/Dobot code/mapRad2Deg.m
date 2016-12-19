function [ deg_q ] = mapRad2Deg( rad_q )
%MAPRAD2BIT Summary of this function goes here
%   Detailed explanation goes here

deg_q = zeros(length(rad_q),1);

for i = 1:length(deg_q)

deg_q(i) = rad_q(i)*(180/pi);

end


deg_q = int16(deg_q);

end

