### Description of the minJerkTrajectory.m

#### Overview
The provided MATLAB code simulates the motion of an object along a predefined rectangular path in the XY plane using a minimum jerk trajectory. Minimum jerk trajectories are used to generate smooth motion paths by minimizing the rate of change of acceleration (jerk).

#### Key Components and Structure

1. **Initialization and Global Variables:**
    ```matlab
    clc; clear; close all;
    
    % Global variables for current position and velocity
    global pos vel;
    
    % Initial conditions (can be modified)
    pos = [0, 0, 0]; % Initial position [x, y, z]
    vel = [0, 0, 0]; % Initial velocity [vx, vy, vz]
    ```
    - **clc, clear, close all:** Clears the command window, workspace, and closes all figure windows respectively.
    - **global pos vel:** Declares `pos` and `vel` as global variables to store the current position and velocity.
    - **pos, vel:** Initialize the position and velocity vectors to zero.

2. **Defining the Rectangular Path:**
    ```matlab
    % Rectangle vertices in the XY plane
    rect_vertices = [pos;              
                     0.1, 0.1, 0;      
                     0.5, 0.1, 0;      
                     0.5, 0.5, 0;      
                     0.1, 0.5, 0;      
                     0.1, 0.1, 0];     
    ```
    - Defines the vertices of a rectangle in the XY plane. The object will move through these vertices sequentially.

3. **Simulation Parameters:**
    ```matlab
    % Time settings
    time_step = 0.008; % 8 milliseconds
    total_time = 10; % Total time for the simulation in seconds
    num_steps = total_time / time_step; % Number of steps
    
    % Preallocate arrays for storing position, velocity, acceleration, and jerk
    positions = zeros(num_steps, 3);
    velocities = zeros(num_steps, 3);
    accelerations = zeros(num_steps, 3);
    jerks = zeros(num_steps, 3);
    ```
    - **time_step, total_time, num_steps:** Defines the time step for the simulation, the total duration, and calculates the number of steps.
    - **Preallocate arrays:** Initializes arrays to store the position, velocity, acceleration, and jerk data for each simulation step.

4. **Main Simulation Loop:**
    ```matlab
    step_index = 1; % Index to keep track of the step
    for i = 1:length(rect_vertices)-1
        % Calculate the current vertex and the next vertex
        current_vertex = rect_vertices(i, :);
        next_vertex = rect_vertices(i+1, :);
        
        % Calculate the minimum jerk trajectory for the current segment
        [segment_pos, segment_vel, segment_acc, segment_jerk] = minJerk(current_vertex, next_vertex, vel, time_step, total_time / (length(rect_vertices)-1));
        
        % Store the results for the current segment
        for j = 1:length(segment_pos)
            if step_index <= num_steps
                positions(step_index, :) = segment_pos(j, :); % Store position
                velocities(step_index, :) = segment_vel(j, :); % Store velocity
                accelerations(step_index, :) = segment_acc(j, :); % Store acceleration
                jerks(step_index, :) = segment_jerk(j, :); % Store jerk
                step_index = step_index + 1; % Increment step index
            end
        end
        
        % Update the global position and velocity to the end of the current segment
        pos = next_vertex;
        vel = segment_vel(end, :);
    end
    ```
    - **step_index:** Keeps track of the current simulation step.
    - **Loop through vertices:** Iterates through each segment of the rectangle, calculates the minimum jerk trajectory between the current and next vertex, and stores the results.

5. **Plotting the Results:**
    ```matlab
    % Plotting the XY trajectory
    figure;
    plot(positions(:, 1), positions(:, 2)); % Plot X vs Y positions
    title('XY Trajectory');
    xlabel('X [m]');
    ylabel('Y [m]');
    axis equal; % Ensuring the aspect ratio is equal for x and y
    
    % Plotting results in a separate figure
    figure;
    subplot(4, 1, 1);
    plot((0:num_steps-1)*time_step, positions); % Plot positions over time
    title('Position');
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('x', 'y', 'z');
    
    subplot(4, 1, 2);
    plot((0:num_steps-1)*time_step, velocities); % Plot velocities over time
    title('Velocity');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    legend('vx', 'vy', 'vz');
    
    subplot(4, 1, 3);
    plot((0:num_steps-1)*time_step, accelerations); % Plot accelerations over time
    title('Acceleration');
    xlabel('Time [s]');
    ylabel('Acceleration [m/s^2]');
    legend('ax', 'ay', 'az');
    
    subplot(4, 1, 4);
    plot((0:num_steps-1)*time_step, jerks); % Plot jerks over time
    title('Jerk');
    xlabel('Time [s]');
    ylabel('Jerk [m/s^3]');
    legend('jx', 'jy', 'jz');
    ```
    - **XY Trajectory Plot:** Plots the trajectory of the object in the XY plane.
    - **Detailed Plots:** Plots the position, velocity, acceleration, and jerk over time in separate subplots.

6. **Minimum Jerk Trajectory Function:**
    ```matlab
    function [new_pos, new_vel, new_acc, new_jerk] = minJerk(start_pos, end_pos, start_vel, time_step, segment_time)
        T = segment_time; % Duration of the segment
        tau = (0:time_step:T)'/T; % Normalized time vector
        
        pos_diff = end_pos - start_pos; % Position difference
        vel_diff = -start_vel; % Velocity difference (initial velocity assumption)
        
        % Minimum jerk position trajectory considering initial velocity
        new_pos = start_pos + start_vel .* tau * T + (pos_diff - start_vel * T) .* (10*tau.^3 - 15*tau.^4 + 6*tau.^5);
        
        % Minimum jerk velocity trajectory considering initial velocity
        new_vel = start_vel + (pos_diff - start_vel * T) .* (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T;
        
        % Minimum jerk acceleration trajectory
        new_acc = (pos_diff - start_vel * T) .* (60*tau - 180*tau.^2 + 120*tau.^3) / T^2;
        
        % Minimum jerk jerk trajectory
        new_jerk = (pos_diff - start_vel * T) .* (60 - 360*tau + 360*tau.^2) / T^3;
    end
    ```
    - **minJerk function:** Calculates the minimum jerk trajectory between two points given the start position, end position, initial velocity, time step, and segment time.
    - **new_pos, new_vel, new_acc, new_jerk:** Computes and returns the position, velocity, acceleration, and jerk trajectories respectively.
