global robot
%% Connect to RR Services
[ P0T_zero, R04_zero ] = initilize_arm( );
pause(1)
% get arm parameters


% Connect to Webcamera and load camera location
cam = RobotRaconteur.Connect('tcp://localhost:2355/WebcamServer/Webcam');
img = cam.SnapShot();

my_image = reshape(img.data,[480 640 3]);

% Connect to Image Processing Service and load/define parameters
%robot = RobotRaconteur.Connect('tcp://localhost:10001/phantomXRR/phantomXController');
IPH = RobotRaconteur.Connect('tcp://localhost:34567/ImageProcessingServer/ImageProcessingHost');
IP_ref = IPH.allocateImageProcessor();
IP = IPH.get_imageProcessor(IP_ref);

fc = [810 810];%[1463.93855 1462.28110]; % Guess values
cc = [329 325];%[685.06985 413.23271]; % Guess values
K = [fc(1) 0 cc(1) 0 fc(2) cc(2) 0 0 1]';
D = [.016 .38 0 0]';
marker_size = 0.02;
IP.setALVARMarkerSize(marker_size);

% Connect IP to Camera and Enable ALVAR Detection
IP.connectToRobotRaconteurCameraService('tcp://localhost:2355/WebcamServer/Webcam','SimpleWebcam_service.Webcam');
ALVARMultiMarkerBundle = struct('n_markers', int32(1), 'marker_ids', ...
    int32([56]), 'marker_translations', [0; 0; 0], ...
    'marker_orientations', [1; 0; 0; 0], 'marker_sizes', [marker_size]);
IP.connectToRobotRaconteurCameraService('tcp://localhost:2355/WebcamServer/Webcam','SimpleWebcam_service.Webcam');
ID = IP.addALVARMultiMarkerBundle(ALVARMultiMarkerBundle);
IP.setALVARMarkerSize(marker_size);
IP.setCameraCalibrationData(K,D);

IP.enableALVARDetection();

display('Begin Search!')

%% Search Algorithm%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Search for satellite, set initial offset distance, move on to approach
% once detected

PTC = [.03;.02;.01];

%define path to take:
    %coordinate system:
    x = [1;0;0];
    y = [0;1;0];
    z = [0;0;1];

    %define line in 3D space:
    P0T_start = [2.5;1.5;1]*0.1;
    P0T_end = [2.5;-1.5;1]*0.1;
    
    duck1 = [0.0350;0.1500;-0.1300];
    duck_pickup1 = P0T_zero+duck1;
    duck2 = [0.0350;-0.1500;-0.1300];
    duck_pickup2 = P0T_zero+duck2;
    
    lambda = 50;

    linex = linspace(P0T_start(1),P0T_end(1),lambda)-P0T_start(1);
    liney = linspace(P0T_start(2),P0T_end(2),lambda)-P0T_start(2);
    linez = linspace(P0T_start(3),P0T_end(3),lambda)-P0T_start(3);

    %define obstical:
    c = [0.1650; 0; 0.1710];
    r = 0.1225;
    

desiredx = 0.0056;%-0.047;
desiredy = -0.0057;%-0.1087;
desiredz = 0;%.3318;

% set arm to starting position:
i = 1;
suck  = int16(0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while true
   T = IP.getCameraPoseFromMultiMarker(int32(ID));
   T = [T(1) T(2) T(3) T(4); T(5) T(6) T(7) T(8); T(9) T(10) T(11) ...
       T(12); T(13) T(14) T(15) T(16)];

    
if T == eye(4)
    disp('can no longer see the Tag, Resuming Task')
    %disp(error)
    P0T_line = P0T_start + [linex(i);liney(i);linez(i)];
    
    %(P0T_line(3)) < c(3)
    %check if collision
    if (P0T_line(3)) < c(3)
        
        %calculated distance to obstican in x/y
        dist_x = c(1) - P0T_line(1);
        dist_y = c(2) - P0T_line(2);
        dist = norm([dist_x dist_y]);
       
        if dist < r
            %theta = cos(dist_y/r);
            %change = asin(theta)*r + dist_x
            %P0T_current(1) = P0T_current(1)-change;
            ex = P0T_line(1);
            ey = P0T_line(2);
            P0T_line(1) = -sqrt(-((ey-c(2))^2)+r^2)+ex+0.06;          
            
            
        end   
    end
 
    % move the arm
    [ ~ ] = move_arm( P0T_line, suck );
    plot(P0T_line(1),P0T_line(2),'xb')
    pause(0.2)
    %P0T_last = P0T_current;
    if i == length(linex)
        %reset everything:
        
        i=1;
        q_zero = mapDeg2Rad([0;0;0;0]);
        [R0T_zero, P0T_zero]=forward_kinematics(mapDeg2Rad(q_zero));
        [ ~ ] = move_arm( P0T_zero, suck );
        pause(0.5)
    elseif i == 1
        [ ~ ] = move_arm( duck_pickup1, suck );
        pause(1)
        if suck == 1
            suck = 0;
        else
            suck = 1;
        end
        [ ~ ] = move_arm( duck_pickup1, suck );
        pause(1)
        i = i+1;
    else
        i = i+1;
    end
    
    
else
    disp('Obstruction detected')
    % calculate error
    errorx = T(2,4)-desiredx;
    errory = T(1,4)-desiredy;
    errorz = T(3,4)-desiredz;  
    errorz = 0;
   
    %find R0T
    [ q_deg ] = inverse_kinematics( P0T_line );
    [R0T_current, P0T_current] = forward_kinematics(mapDeg2Rad(q_deg));
    error = (R0T_current.')*[-errorx; -errory; errorz];
    
    %P0T_t = P0T_current + ( 0.5*error);
    %P0T_t = [P0T_t(1); P0T_t(2); 5];
    %disp(P0T_t)
    
    disp([['errorx: ';'errory: ';'errorz: '] num2str(error)])
    disp(' ')
    %disp(P0T_t-P0T_current)
    P0T_avoid = P0T_line + [0;0;0.3];
    [~] = move_arm(P0T_avoid, suck);
    pause(2)
end

  
end
