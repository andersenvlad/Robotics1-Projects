global robot
%% Connect to RR Services

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
[ P0T_zero, R04_zero ] = initilize_arm( );
pause(1)
i = 1;

tickrate = 0.5;
tick_last = clock;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while true
   T = IP.getCameraPoseFromMultiMarker(int32(ID));
   T = [T(1) T(2) T(3) T(4); T(5) T(6) T(7) T(8); T(9) T(10) T(11) ...
       T(12); T(13) T(14) T(15) T(16)];
    
if T == eye(4)
    disp('can no longer see the Tag, stopping')
    
    
else
    disp('Tag in view.. following')
    % calculate error
    errorx = T(2,4)-desiredx;
    errory = T(1,4)-desiredy;
    errorz = T(3,4)-desiredz;  
    errorz = 0;
   
    error = (R0T.')*[-errorx; -errory; errorz];
    
    q_current = robot.getJointPositions();
    [R0T_current, P0T_current]=forward_kinematics(mapDeg2Rad(q_current));
    
    % Proportinal control law
    P0T_t = P0T_current + ( 0.5 * error);

    disp([['errorx: ';'errory: ';'errorz: '] num2str(error)])
    disp(' ')
    disp(P0T_t-P0T_current)
    q_new = move_arm(P0T_t);
    [R0T,P0T_old] = forward_kinematics(mapDeg2Rad(q_new));
end

% time each cylce
tick_current = clock;
while etime(tick_current,tick_last) < tickrate
    % wait
end
tick_last = tick_current;


  
end
