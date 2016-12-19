function [ q_new ] = move_arm( P0T_new, q_last, suck )
%MOVE_ARM Summary of this function goes here
global robot;

% figure out q needed to be at P_new
[ q_new ] = inverse_kinematics( P0T_new );

offset = 0;
q_new(4) = q_last - q_new(1)-offset;

[R0T_calc, P0T_calc] = forward_kinematics(mapDeg2Rad(q_new));


%check if q is in bounds
disp('moving to:')
disp(q_new)
disp(P0T_calc)
robot.setJointPositions(q_new(1),q_new(2),q_new(3),q_new(4),int16(suck))

end

