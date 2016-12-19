function [ P, H ] = get_arm_param( )
    %GET_PARAM Summary of this function goes here
    %   gets all the arm parameters, segment/link vectors, rotation axis of
    %   joints

    %coordinate system:
    x = [1;0;0];
    y = [0;1;0];
    z = [0;0;1];

    % legit values
    L1 = 0.103;
    L2 = 0.135;
    L3 = 0.160;
    L4 = 0.055;
    L5 = -0.068;

    %all expressed in their own frames
    P01 = [0; 0; L1];
    P12 = [0; 0; 0];
    P23 = [0.00; 0; L2];
    P34 = [L3; 0; 0];
    P4T = [L4; 0; L5];

    P = [P01 P12 P23 P34 P4T];
    H = [z y y z]; 

    % Normalize
    for i = 1:length(H)
        H(:,i) = H(:,i)/norm(H(:,i));
    end


end

