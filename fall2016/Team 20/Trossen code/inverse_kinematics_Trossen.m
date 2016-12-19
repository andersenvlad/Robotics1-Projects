function [ sol ] = inverse_kinematics( R04, P0T )
%INVERSE_KINE Summary of this function goes here
%   Detailed explanation goes here

[ P, H ] = get_arm_param( );

P01 = P(:,1);
P12 = P(:,2);
P23 = P(:,3);
P34 = P(:,4);
P4T = P(:,5);

% R04 is known so we can calulate P4T in the 0 frame
P4T0 = R04*P4T;

k1 = H(:,1);
k2 = H(:,2);
k3 = H(:,3);
k4 = H(:,4);

%subproblem 3: 2 solutions.
q3 = subproblem3(k3, P34, -P23, norm(P0T-P01-P4T0));

%subproblem 2: 2 solutions for each previous (4 solutions total)
sol = [q3(1);q3(1);q3(2);q3(2)];
sol = [zeros(4,2) sol];
for i = 1:length(q3)
    
    [q1, q2] = subproblem2(k1,k2,(P23 + rot(k3, q3(i))*P34),(P0T-P01-P4T0));
    sol(2*i-1,1) = q1(1);
    sol(2*i,1) = q1(2);
    sol(2*i-1,2) = q2(1);
    sol(2*i,2) = q2(2);
    
end

% subproblem 1 1 solution per previous solution

q4 = 0;
for i = 1:length(sol(:,1))
    qt = sol(i,:); % take first row of previous 4 solutions
    q1t = qt(1);
    q2t = qt(2);
    q3t = qt(3);
    
    R01 = rot(k1,q1t);
    R02 = R01*rot(k2,q2t);
    R03 = R02*rot(k3,q3t);
    
    P120 = R01*P12;
    P230 = R02*P23;
    P340 = R03*P34;
    
    P120 = R01*P12;
    
    Q = (R03^-1)*(P0T-P01-P120-P230-P340);
    
    [q4t] = subproblem1(k4, P4T, Q);
    
    q4(i,1) = q4t;
end

sol = [sol q4];

% filter out solutions that are not possible to reach.




end

