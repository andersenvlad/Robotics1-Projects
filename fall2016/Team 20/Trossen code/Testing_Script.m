% %connect
global robot
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

start_angles = int16( [0; 0; 0; 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4))
pause(0.5);

q_deg = start_angles;

q_rad = mapDeg2Rad(q_deg);

R0T = eye(3);

[R04, P0T] = forward_kinematics(q_rad);

disp('command:')
disp(q_deg)

disp('position calculated from command')
disp(P0T)

[ q_guess ] = inverse_kinematics( P0T );

% pos = P0T;
% for i = 1:length(q_guess(:,1))
%     [R04, P0T] = forward_kinematics(q_guess(i,:));
%     pos(:,i) = P0T;
% end

disp('inverse kinematics joint angles')
disp(q_guess)

% disp('measured joint angles')
% robot.getJointPositions()



%[ q_new, R0T ] = move_arm_P( P0T );
