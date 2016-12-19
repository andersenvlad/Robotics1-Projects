function [ J ] = jacobian( P, H, q )
%JACOBIAN Summary of this function goes here
% calculates the jacobian symbolicly

%%%%%% FOR SYMBOLIC CALULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(class(q),'sym') == 1 %check if angles are symbolic
    % syms q1 q2 q3 q4
    % q = [q1;q2;q3;q4]; %this makes the answer a mess
    J = sym(zeros(6,length(q))); 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x = [1;0;0];
y = [0;1;0];
z = [0;0;1];
    
% initilize
Pf = P(:,1);
Rf = eye(3);
% run forward kinematics
for j = 1:length(q)
    Rf = Rf*rot(H(:,j),q(j));
    Pf = Pf + Rf*P(:,j+1);
end

% disp('Vector from O to T')
% disp(Pf)

%initilize
R = eye(3);
for i = 1:length(q)
    hj = R*H(:,i);
    
    %get Pf run forward kinematics
    % initilize
    Pf = [0;0;0]; %P(:,1);
    Rf = eye(3);
    % run forward kinematics from i'th frame to task frame
    for j = i:length(q)
        Rf = Rf*rot(H(:,j),q(j));
        Pf = Pf + Rf*P(:,j+1);
    end
    
    % rotate result from forward kinematics to 0 frame
    % result will be expressed in the i'th frame
     
    Pj = R*Pf;
    R = R*rot(H(:,i),q(i));   
    J(:,i) = [hj; hat(hj)*Pj];

end

J=J; %assign output

end

