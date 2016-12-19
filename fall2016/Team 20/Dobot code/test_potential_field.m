%global robot
%[ robot ] = initilize_arm( );

%speed:
speed = 0.01; %m/s

%coordinate system:
x = [1;0;0];
y = [0;1;0];
z = [0;0;1];

%define line in 3D space:
P0T_start = [2.3;1.5;0.5]*0.1;
P0T_end = [2.3;-1.5;0.5]*0.1;

dl = 0.005;

%define obstical:
c = [0.30; 0; 0.1710];
r = 0.1225;

%[ q_new ] = move_arm( P0T_start, 0 );
figure(1)
plot(P0T_start(1),P0T_start(2),'xb')
hold on
plot(c(1),c(2),'xr')

[cx,cy] = obs(c,r-0.03);
plot(cx,cy,'r')

arrive_time = clock;
start_time = clock;

P0T_last = P0T_start;
dist_end = P0T_end - P0T_last;
P0T_current = P0T_start;

iter = 0

while norm(dist_end) > 0.01
    
    dist_end = P0T_end - P0T_last;
    P0T_change = (dist_end)*(dl/norm(dist_end));
    
    %find distance to obstacle
    dist_x = c(1) - P0T_current(1);
    dist_y = c(2) - P0T_current(2);
    dist = norm([dist_x dist_y]);
    
    force = [dist_x; dist_y; 0]*(0.001/(dist-r))^2;
    P0T_change = P0T_change - force
    
    P0T_current = P0T_last + P0T_change;
    
    
 
    % move the arm
    % find distance:
    travel_dist = norm(P0T_current - P0T_last);
    wait_time = travel_dist/speed;
    while (1)
        time = clock;
        disp(etime(time,arrive_time))
        if etime(time,arrive_time) > wait_time
            break
        end
    end
    
%    [ q_new ] = move_arm( P0T_current, 0, 0 );
    arrive_time = clock;

    
    plot(P0T_current(1),P0T_current(2),'xb')
    hold on
    xlim([0.1 0.5])
    ylim([-0.2 0.2])
    xlabel('x-position (meters)')
    ylabel('y-position (meters)')
    pause(0.2)
    P0T_last = P0T_current;
 
    iter = iter + 1;
end

final_time = etime(clock,start_time)
