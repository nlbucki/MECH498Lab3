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
g = robot.g; % m/s^2

% Joint Torque Limit
tau_max = 20; % [N-m] (Scalar)

% Time
dt = 0.01; % [s]
t_f = 10; % [s]

% Initial Conditions
X_0 = [pi/3 pi/2 0 0]; % [theta_1, theta_2, theta_dot_1, theta_dot_2]

% Control Gains (Scalar)
K_p1 = 20;
K_p2 = 200;
K_d1 = 10;
K_d2 = 8;
K_p = [K_p1 0; 0 K_p2];
K_v = [K_d1 0; 0 K_d2];
X_d = [0, pi/2];

% Numerical Integration
t = 0:dt:t_f;
X = zeros(length(t),4); % initialize variable to hold state vector
X_dot = zeros(length(t),4); % initialize variable to hold state vector derivatives

kinetic_energy = zeros(1,length(t));
potential_energy = zeros(1,length(t));
total_energy = zeros(1,length(t));

for i = 1:length(t)
    if i == 1
        X(i,:) = X_0;
    else
        X(i,:) = X(i-1,:);
    end
    
    % Control torques
    tau = -K_p*(X(i,1:2) - X_d)' - K_v*X(i,3:4)';
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M_1*L_c1^2 + M_2*(L_1 + L_c2*cos(X(i,2)))^2 + (I_1 + I_2), 0;
        0, M_2*L_c2^2 + I_2];
    C = [-2*M_2*L_c2*sin(X(i,2))*(L_1 + L_c2*cos(X(i,2)))*X(i,3)*X(i,4);
        M_2*L_c2*sin(X(i,2))*(L_1 + L_c2*cos(X(i,2)))*X(i,3)^2];
    G = [0;
        M_2*g*L_c2*cos(X(i,2))];

    X_dot(i,1:2) = [X(i,3) X(i,4)];
    X_dot(i,3:4) = (M\(tau - C - G))';
    
    % Trapezoidal Integration
    if i > 1
        X(i,3:4) = X(i-1,3:4) + 0.5*(X_dot(i-1,3:4) + X_dot(i,3:4))*dt;
        X(i,1:2) = X(i-1,1:2) + 0.5*(X_dot(i-1,1:2) + X_dot(i,1:2))*dt;
    end
    
    % Record Energy
    kinetic_energy(i) = 0.5*X_dot(i,1:2)*M*X_dot(i,1:2)';
    potential_energy(i) = M_2*g*L_c2*sin(X(i,2)) + M_2*g*L_c2;
    total_energy(i) = kinetic_energy(i) + potential_energy(i);
end

% Graphical Simulation

% Set up energy plot
energy_figure = figure('Name','Energy Plot');
movegui(energy_figure,'northwest');
energy_axes = axes('Parent',energy_figure)
axis(energy_axes,[0, t_f, -200, 200]);
kinetic_line = animatedline(0,kinetic_energy(1),'Color','red','Parent',energy_axes);
potential_line = animatedline(0,potential_energy(1),'Color','green','Parent',energy_axes);
total_line = animatedline(0,total_energy(1),'Color','black','Parent',energy_axes);
legend('Kinetic Energy','Potential Energy','Total Energy','Location','southeast');

% Set up angle plot
angle_figure = figure('Name','Angle Plot');
movegui(angle_figure,'southwest');
angle_axes = axes('Parent',angle_figure)
axis(angle_axes,[0, t_f, -3*pi/2, 3*pi/2]);
theta_1_line = animatedline(0,X(1,1),'Color','red','Parent',angle_axes);
theta_2_line = animatedline(0,X(1,1),'Color','green','Parent',angle_axes);
line([0,t_f],[X_d(1) X_d(1)],'Color','red','LineStyle','--','Parent',angle_axes);
line([0,t_f],[X_d(2) X_d(2)],'Color','green','LineStyle','--','Parent',angle_axes);
legend('Theta 1','Theta 2','Desired Theta 1','Desired Theta 2','Location','southeast');

% Set up robot animation
robot.handles = drawRR(X_0,robot);

for i = 2:length(t)
    % Update robot animation
    setRR(X(i,:),robot);
    
    % Update energy plot
    addpoints(kinetic_line, t(i), kinetic_energy(i));
    addpoints(potential_line,t(i),potential_energy(i));
    addpoints(total_line,t(i),total_energy(i));
    
    % Update angle plot
    addpoints(theta_1_line,t(i),X(i,1));
    addpoints(theta_2_line,t(i),X(i,2));
    
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output





end

