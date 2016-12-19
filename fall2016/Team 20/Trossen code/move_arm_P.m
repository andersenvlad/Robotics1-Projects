function [ q_new, R0T ] = move_arm_P( P0T )
%MOVE_ARM_P Summary of this function goes here
global robot

[ P, H ] = get_arm_param( );

k1 = H(:,1);
k2 = H(:,2);
k3 = H(:,3);
k4 = H(:,4);

q1g = atan(P0T(2)/P0T(1));
R01 = rot(k1, q1g);

R1T = rot(k2, -pi/2);
R0T = R01*R1T;

% testing moving:
disp('Position given')
disp(P0T)
q_guess = inverse_kinematics(R0T,P0T)


%convert to degree for easy readability
q_deg = zeros(size(q_guess));
for i = 1:length(q_guess(:,1))
     %convert to degree for easy readability
    q_deg(i,:) = mapRad2Deg(q_guess(i,:));
end
q_deg = q_deg

pos = P0T;
for i = 1:length(q_guess(:,1))
    [R04, P0T] = forward_kinematics(q_guess(i,:));
   
    pos(:,i) = P0T;
end

disp('positions:')
disp(pos)

%filter out solutions not within bounds.
[ bounds_d, bounds_r, bounds_b] = get_bounds(  );

valid_flag = ones(length(q_guess(1,:)),1);

for j = 1:length(q_guess(:,1))
    
    % select row of solutions
    qt = q_guess(j,:);
    
    % for each of the 4 angles check which ones are valid
    for i = 1:length(qt);
        bounds_current = bounds_r(i,:);
                
        if qt(i) < bounds_current(1)
            valid_flag(j) = 0;
        else
            
        end
        
        if qt(i) > bounds_current(2)
            valid_flag(j) = 0;
        else
            
        end
        
    end

end

select = 0;
disp('The only solutions that are valid are:')
for i = 1:length(valid_flag)
    if valid_flag(i) == 1
        select = i;
        disp(i)
    end
end
if select == 0
    disp('none')
    q_new = [0; 0; 0; 0; 0];
else
        % add on prismatic joint unused angle
    q_new = [q_guess(select,:) 0];
end

q_new(2) = q_new(2)*-1;

pose = mapRad2Bit(q_new);
pose = int16(pose);
disp('moving to:')
disp(pose)
robot.setJointPositions(pose);
pause(0.5)
robot.setJointPositions(pose);
disp('Moved')



end

