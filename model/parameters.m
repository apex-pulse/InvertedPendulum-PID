% parameters.m
% Physical constants for cart + inverted pendulum

% Cart mass (kg)
M = 0.5;        % mass of cart
% Pendulum mass (kg)
m_p = 0.2;      % mass of pendulum bob
% Pendulum length (m) -- distance from pivot to center of mass
l = 0.3;
% gravitational acceleration (m/s^2)
g = 9.81;

% Derived inertia for point mass at length l (approx)
I = m_p * l^2;

% Simulation settings
dt = 0.01;      % time step (s)
Tend = 10;      % simulation end time (s)

% Initial conditions [x; x_dot; theta; theta_dot]
% theta measured from upright (0 = upright). Small perturbation to test.
x0 = 0.0;
x_dot0 = 0.0;
theta0 = deg2rad(5);     % small initial angle (5 degrees)
theta_dot0 = 0.0;

% PID gains (start values — tune as needed)
Kp = 150;   % proportional on theta (angle)
Ki = 5;     % integrator
Kd = 20;    % derivative

% Force limits (saturation)
F_max = 20;    % N
F_min = -20;   % N

% Export variables to workspace when script run
save('params_workspace.mat','M','m_p','l','g','I','dt','Tend', ...
    'x0','x_dot0','theta0','theta_dot0','Kp','Ki','Kd','F_max','F_min');

fprintf('parameters loaded: M=%.3f kg, m=%.3f kg, l=%.3f m\n',M,m_p,l);
