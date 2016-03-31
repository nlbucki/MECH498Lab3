function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

close all;

% Initialize robot
robot = RRInit();
m_1 = robot.m_1;
m_2 = robot.m_2;
m_r1 = robot.m_r1;
m_r2 = robot.m_r2;
M_1 = m_1 + m_r1;
M_2 = m_2 + m_r2;
L_1 = robot.l_1;
L_2 = robot.l_2;
L_c1 = (m_1 + 0.5*m_r1)*L_1/M_1;
L_c2 = (m_2 + 0.5*m_r2)*L_2/M_2;
I_1 = (1/12)*m_r1*L_1^2 + m_1*(L_1/2)^2;
I_2 = (1/12)*m_r2*L_2^2 + m_2*(L_2/2)^2;
g = 9.81; % m/s^2

% Joint Torque Limit
tau_max = 20; % [N-m] (Scalar)

% Time
dt = 0.01; % [s]
t_f = 10; % [s]

% Initial Conditions
X_0 = [pi/3 pi/2 0 0]; % [theta_1, theta_2, theta_dot_1, theta_dot_2]

% Control Gains (Scalar)
K_p = [];
K_v = [];

% Numerical Integration
t = 0:dt:t_f;
X = zeros(length(t),4); % initialize variable to hold state vector
X_dot = zeros(length(t),4); % initialize variable to hold state vector derivatives
for i = 1:length(t)
    if i == 1
        X(i,:) = X_0;
    else
        
    end
    
    % Control torques
    tau = [0 0]';
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M_1*L_c1^2 + M_2*(L_1 + L_c2*cos(X(2)))^2 + (I_1 + I_2), 0;
        0, M_2*L_c2^2 + I_2];
    C = [-2*M_2*L_c2*sin(X(2))*(L_1 + L_c2*cos(X(2)))*X(3)*X(4);
        M_2*L_c2*sin(X(2))*(L_1 + L_c2*cos(X(2)))*X(3)^2];
    G = [0;
        M_2*g*L_c2*cos(X(2))];

    X_dot(i,1:2) = [X(3) X(4)];
    X_dot(i,3:4) = (M\(tau - C - G))';
    
    % Trapezoidal Integration
    if i > 1
        X(i,3:4) = X(i-1,3:4) + 0.5*(X_dot(i-1,3:4) + X_dot(i,3:4))*dt;
        X(i,1:2) = X(i-1,1:2) + 0.5*(X_dot(i-1,1:2) + X_dot(i,1:2))*dt;
    end
    
    % Plot Energy
    
end

% Graphical Simulation
robot.handles = drawRR(X_0,robot);
for i = 2:length(t)
    setRR(X(i,:),robot);
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output





end

