% response_plot.m
% Visualization script for inverted pendulum PID control simulation
% Loads results from sim_results.mat and generates publication-quality plots

clearvars; close all; clc;

% Load simulation results
load(fullfile('..','results','sim_results.mat'),'results');

t = results.time;
theta = results.theta;
x = results.x;
F = results.F;

% Create figure with three subplots
figure('Units','pixels','Position',[100 100 900 600],'Color','w');

% Subplot 1: Pendulum angle
subplot(3,1,1);
plot(t, rad2deg(theta), 'LineWidth', 1.5);
grid on;
ylabel('\theta (deg)');
title('Pendulum Angle Response');
xlabel('');

% Subplot 2: Cart position
subplot(3,1,2);
plot(t, x, 'LineWidth', 1.5);
grid on;
ylabel('x (m)');
title('Cart Position Response');
xlabel('');

% Subplot 3: Control force
subplot(3,1,3);
plot(t, F, 'LineWidth', 1.5);
grid on;
ylabel('F (N)');
xlabel('Time (s)');
title('Control Force Applied');

% Add overall title
sgtitle('Inverted Pendulum - PID Closed-Loop Response', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure to results directory
if ~exist(fullfile('..','results'),'dir')
    mkdir(fullfile('..','results'));
end

saveas(gcf, fullfile('..','results','step_response.png'));
fprintf('✓ Plot saved to results/step_response.png\n');
