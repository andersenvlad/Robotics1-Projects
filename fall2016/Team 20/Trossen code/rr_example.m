robot = RobotRaconteur.Connect('tcp://localhost:10001/dobotRR/dobotController');

start_angles = int16( [0 0 0 0] )
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4))
pause(5);

start_angles = int16( [0 0 0 0] );
robot.setJointPositions(start_angles(1),start_angles(2),start_angles(3),start_angles(4))
pause(5);

robot.getJointPositions()