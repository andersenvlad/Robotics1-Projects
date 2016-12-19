function [ rad_q ] = mapBit2Rad( bit_q )
%MAPBIT2RAD Summary of this function goes here
%   Detailed explanation goes here unfinished

rad_q = zeros(length(bit_q),1);

for i = 1:length(rad_q)
    % base q1
    bounds_d = [-150 150; -90 90; -90 90; -90 90; -90 90];
    bounds_r = bounds_d.*(pi/180);
    bounds_b = bounds_d.*(512/150);

    % -150 deg = 0
    % 150 deg = 1024
    bit_q = double(bit_q);

    rad_q(i) = (bit_q(i)-512)/((180/pi)*(512/150));


    if rad_q(i) < bounds_r(i,1)
        rad_q(i) = bounds_r(i,1);
    else if rad_q(i) > bounds_r(i,2)
        rad_q(i) = bounds_r(i,2);
        end



end

end


