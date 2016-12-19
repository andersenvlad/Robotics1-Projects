function [ R04, P0T ] = forward_kinematics( q )
%FORWARD_KINEMATICS Summary of this function goes here
%   Detailed explanation goes here

[ P, H ] = get_arm_param( );


P01 = P(:,1);
P12 = P(:,2);
P23 = P(:,3);
P34 = P(:,4);
P4T = P(:,5);

k1 = H(:,1);
k2 = H(:,2);
k3 = H(:,3);
k4 = H(:,4);

R01 = rot(k1, q(1));
R02 = R01*rot(k2, q(2));
R03 = R01*rot(k3, q(3));
R04 = R01*rot(k4, q(4));

P0T = P01 + R01*P12 + R02*P23 + R03*P34 + R04*P4T;

Data = [0;0;0];
Data = [Data P01 R01*P12 R02*P23 R03*P34 R04*P4T];

%plot3(Data(1,:),Data(2,:),Data(3,:))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% jointtype=[0;0;0;0];
% n=4;scale=.5;
% [my_robot,my_robot_structure]=defineRobot(jointtype,H,P,n,scale)
% 
% maxP = sqrt(max(sum(P.*P,1)));
% if sum(jointtype)>0
%   fixaxis=[-1 1 -1 1 0 2]*maxP*1.5;
% else
%   fixaxis=[-1 1 -1 1 0 2]*maxP;
% end
% viewpoint=[-32 26];

%figure(1); clf; 
%h_my_robot = createCombinedRobot(my_robot,my_robot_structure);
%axis equal;axis(fixaxis);view(viewpoint);grid   
%h_my_robot = showRobot(q,h_my_robot,fixaxis,viewpoint);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% 
% xlim([-0.2 0.2])
% ylim([-0.2 0.2])
% zlim([-0.2 0.2])

end

