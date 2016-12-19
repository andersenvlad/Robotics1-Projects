% arm calibration demonstration

P0T_zero = [0.1950; 0; 0.002];

c = [0;0.01;0]
initilize_arm()
move_arm(P0T_zero-6*c, 0, 0)

for i = -6:9
    move_arm(P0T_zero+i*c, 0, 0)
    
    pause(1)
end