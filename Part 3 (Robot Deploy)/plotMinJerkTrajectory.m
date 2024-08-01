clc; clear; close all;

% Read the data from the file using readtable
data = readtable('minJerkTrajectory.txt');

% Extracting relevant columns
time = data{:, 'Time'};
positions = data{:, {'X', 'Y', 'Z'}};
velocities = data{:, {'Vx', 'Vy', 'Vz'}};
accelerations = data{:, {'Ax', 'Ay', 'Az'}};
jerks = data{:, {'Jx', 'Jy', 'Jz'}};

% Plotting the XY trajectory
figure;
plot(positions(:, 1), positions(:, 2)); % Plot X vs Y positions
title('XY Trajectory');
xlabel('X [m]');
ylabel('Y [m]');
axis equal; % Ensuring the aspect ratio is equal for x and y

% Determine the number of steps and time step
num_steps = height(data);
time_step = mean(diff(time)); % Assumes uniform time steps

% Plotting results in a separate figure
figure;
subplot(4, 1, 1);
plot(time, positions); % Plot positions over time
title('Position');
xlabel('Time [s]');
ylabel('Position [m]');
legend('x', 'y', 'z');

subplot(4, 1, 2);
plot(time, velocities); % Plot velocities over time
title('Velocity');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('vx', 'vy', 'vz');

subplot(4, 1, 3);
plot(time, accelerations); % Plot accelerations over time
title('Acceleration');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
legend('ax', 'ay', 'az');

subplot(4, 1, 4);
plot(time, jerks); % Plot jerks over time
title('Jerk');
xlabel('Time [s]');
ylabel('Jerk [m/s^3]');
legend('jx', 'jy', 'jz');
