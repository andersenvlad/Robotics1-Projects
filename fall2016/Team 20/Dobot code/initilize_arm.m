function [ P0T_zero, R04_zero ] = initilize_arm( )
%INITILIZE_ARM Summary of this function goes here
%   gets the arm connected and ready for use
global robot
robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

start_angles = int16( [10; 0; 0; 0; 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4),start_angles(5))
pause(0.5);
start_angles = int16( [0; 0; 0; 0; 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4),start_angles(5))

[ R04_zero, P0T_zero ] = forward_kinematics( mapDeg2Rad(start_angles) );

robot.getJointPositions()

end

