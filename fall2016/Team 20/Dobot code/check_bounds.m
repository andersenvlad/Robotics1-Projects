function [ valid ] = check_bounds( q )
%GET_BOUNDS Summary of this function goes here
%   Detailed explanation goes here

    bounds_d = [-135 135; -5 85; -10 95; -90 90];
    bounds_r = bounds_d.*(pi/180);
    
    valid = 1;
    
    for i = 1:length(q)
        bounds_c = bounds_d(i,:);
        if q(i) < bounds_c(1)
            valid = 0;
        else
            
        end
        
        if q(i) > bounds_c(2)
            valid = 0;
        else
            
        end
        
        
    end
    
    
end

