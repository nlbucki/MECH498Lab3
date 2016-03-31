function [ tau ] = robController( trajectory, Theta, Theta_dot, t , rob )
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
g = rob.parameters.g;
b = rob.parameters.b;
m1 = rob.parameters.m1;
m2 = rob.parameters.m2;
m3 = rob.parameters.m3;
m4 = rob.parameters.m4;
l1 = rob.parameters.l1*1e-3;
l2 = rob.parameters.l2*1e-3;
l3 = rob.parameters.l3*1e-3;
% lc3 = l3*(m3/2+m4)/(m3+m4);
% I1 = rob.parameters.I1;
% I2 = rob.parameters.I2;
% J1 = rob.parameters.J1;
% J2 = rob.parameters.J2;
% J3 = rob.parameters.J3;

% Gravity Compensation Vector
% G = [0;
%     (m3+m4)*lc3*g*cos(Theta(2) + Theta(3)) + (m2+m3+m4)*(l2/2)*g*cos(Theta(2));
%     (m3+m4)*lc3*g*cos(Theta(2) + Theta(3))]; %[3x1] vector

G = [0;
    g*m2*l2/2*cos(Theta(2)) + g*m3*(l2*cos(Theta(2)) + l3/2*cos(Theta(2)+Theta(3))) + g*m4*(l2*cos(Theta(2)) + l3*cos(Theta(2) + Theta(3)));
    g*m3*l3/2*cos(Theta(2)+Theta(3)) + g*m4*l3*cos(Theta(2) + Theta(3))];

% Trajectory interpolation (DO NOT CHANGE)
Theta_ref = zeros(3,1);
Theta_dot_ref = zeros(3,1);
for i = 1:3
    Theta_ref(i) = interp1(trajectory(1,:),trajectory(i+1,:),t);
    Theta_dot_ref(i) = interp1(trajectory(1,:),trajectory(i+4,:),t);
end

% Gravity Compensation Control

K_p1 = 100;
K_p2 = 100;
K_p3 = 100;
K_v1 = 100;
K_v2 = 100;
K_v3 = 100;

K_p = [K_p1; K_p2; K_p3]; % Proportional gain matrix containing gains K_p1 to K_p3
K_p = diag(K_p);
K_v = [K_v1; K_v2; K_v3]; % Derivative gain matrix containing gains K_v1 to K_v3
K_v = diag(K_v);

tau = - K_p*(Theta - Theta_ref) - K_v*(Theta_dot - Theta_dot_ref) + G; % control input (torque)

end

