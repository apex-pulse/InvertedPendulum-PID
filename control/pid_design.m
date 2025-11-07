% pid_design.m
% Simulate nonlinear cart-pendulum with PID controller on pendulum angle.

clearvars; close all; clc;
run(fullfile('..','model','parameters.m'));
% load variables from saved mat to ensure path independence
S = load('params_workspace.mat');
fnames = fieldnames(S);
for k=1:numel(fnames), assignin('base',fnames{k},S.(fnames{k})); end
% Copy into local scope
M = S.M; m = S.m_p; l = S.l; g = S.g; dt = S.dt; Tend = S.Tend;
Kp = S.Kp; Ki = S.Ki; Kd = S.Kd; F_max = S.F_max; F_min = S.F_min;

% Desired angle (upright = 0)
theta_des = 0;

% Simulation setup
time = 0:dt:Tend;
N = length(time);

% allocate
x = zeros(1,N); x_dot = zeros(1,N);
theta = zeros(1,N); theta_dot = zeros(1,N);
F = zeros(1,N);

% initial cond
x(1) = S.x0; x_dot(1) = S.x_dot0;
theta(1) = S.theta0; theta_dot(1) = S.theta_dot0;

% PID internal states
integral = 0;
prev_err = theta_des - theta(1);

% helper: dynamics function using standard derived expressions
    function dd = dynamics(state, force)
        % state = [x; x_dot; theta; theta_dot]
        x = state(1); x_dot = state(2); th = state(3); th_dot = state(4);
        % use the common nonlinear closed-form for inverted pendulum
        denom = l*(4/3 - (m*cos(th)^2)/(M + m));
        theta_ddot = (g*sin(th) + cos(th)*((-force - m*l*th_dot^2*sin(th))/(M + m))) / denom;
        x_ddot = (force + m*l*(th_dot^2*sin(th) - theta_ddot*cos(th))) / (M + m);
        dd = [x_dot; x_ddot; th_dot; theta_ddot];
    end

% Fixed-step RK4 integration with discrete PID update each step
for k=1:N-1
    % current state
    state = [x(k); x_dot(k); theta(k); theta_dot(k)];

    % compute PID on angle error (control law outputs force)
    err = theta_des - theta(k);
    integral = integral + err*dt;
    derivative = (err - prev_err)/dt;
    u = Kp*err + Ki*integral + Kd*derivative; % this is force command
    prev_err = err;
    % saturate
    if u > F_max, u = F_max; end
    if u < F_min, u = F_min; end
    F(k) = u;

    % RK4 steps
    k1 = dynamics(state, u);
    k2 = dynamics(state + 0.5*dt*k1, u);
    k3 = dynamics(state + 0.5*dt*k2, u);
    k4 = dynamics(state + dt*k3, u);
    state_next = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    x(k+1) = state_next(1);
    x_dot(k+1) = state_next(2);
    theta(k+1) = state_next(3);
    theta_dot(k+1) = state_next(4);
end

% Save results
results.time = time; results.x = x; results.x_dot = x_dot;
results.theta = theta; results.theta_dot = theta_dot; results.F = F;
save(fullfile('..','results','sim_results.mat'),'results');

fprintf('Simulation finished. Results saved to results/sim_results.mat\n');
