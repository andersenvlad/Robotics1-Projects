%INVERSE_KIN Summary of this function goes here
%Arm a)*******************************************************************

%robot = initilize_arm( );

[ P, H ] = get_arm_param( );

q = [-pi/2; 0; 0; -pi/2];
J1 = jacobian(P,H,q);

% verify jacobian
% global Jp JR
% type = [0 0 0 0];
% n = 4;
% i = 1;
% R = eye(3);
% [p,R0n]=jcal(i,R,q,type,H,P,n);
% J2=[JR;Jp]

disp('original given angle')
q = q
[R04, P0T] = forward_kinematics(q);

disp('Position calculated:')
disp(P0T)
q_guess = inverse_kinematics(R04,P0T)

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
        end
        if qt(i) > bounds_current(2)
            valid_flag(j) = 0;
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



pose = mapRad2Bit(q_new);
pose = int16(pose);
disp('moving to:')
disp(pose)
robot.setJointPositions(pose);
