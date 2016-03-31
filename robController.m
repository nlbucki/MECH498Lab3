function [ tau ] = robController( trajectory, Theta, Theta_dot, t , robot )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    t = current time, scaler
%    Theta = currect angle, 3x1 vector
%    Theta_dot = current speed, 3x1 vector
%    trajectory = desired angle and speed for all time points, 7xn matrix
%
%    DESCRIPTION - This function sends command torques in the vector tau to move the
%    robot. The controller determines tau based on the desired position and
%    velocity and the actual position and velocity. Desired position and
%    velocity are obtained from trajectory and the current time t. The
%    actua position is Theta, and the actual velocity is Theta_dot.
%    Calculate the torques needed to add gravity compensation using the
%    robot parameters from the structure rob.
%

% Robot Parameters from rob
g = robot.parameters.g;...
m_1 = robot.m_1;
m_2 = robot.m_2;
m_r1 = robot.m_r1;
m_r2 = robot.m_r2;
% M_1 = m_1 + m_r1;
% M_2 = m_2 + m_r2;
L_1 = robot.l_1;
L_2 = robot.l_2;

    
% Gravity Compensation Vector
G = [0; 0; g]; %[3x1] vector

% Trajectory interpolation (DO NOT CHANGE)
Theta_ref = zeros(3,1);
Theta_dot_ref = zeros(3,1);
for i = 1:3
    Theta_ref(i) = interp1(trajectory(1,:),trajectory(i+1,:),t);
    Theta_dot_ref(i) = interp1(trajectory(1,:),trajectory(i+4,:),t);
end

% Gravity Compensation Control

K_p1 = 1;
K_p2 = 1;
K_p3 = 1;
K_v1 = 1;
K_v2 = 1;
K_v3 = 1;

K_p = [K_p1; K_p2; K_p3]; % Proportional gain matrix containing gains K_p1 to K_p3
K_p = diag(K_p);
K_v = [K_v1; K_v2; K_v3]; % Derivative gain matrix containing gains K_v1 to K_v3
K_v = diag(K_v);

tau = - K_p*(Theta - Theta_ref) - K_v*(Theta_dot); % control input (torque)

end

