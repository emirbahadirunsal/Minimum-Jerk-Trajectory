### Minimum Jerk Trajectory

#### Function Inputs:

- `start_pos`: Initial position vector $[x_0, y_0, z_0]$.

- `end_pos`: Final position vector $[x_f, y_f, z_f]$.

- `start_vel`: Initial velocity vector $[v_{x0}, v_{y0}, v_{z0}]$.

- `time_step`: Time increment for simulation.

- `segment_time`: Total duration of the segment.

#### Function Outputs:
- `new_pos`: Position trajectory over time.
- `new_vel`: Velocity trajectory over time.
- `new_acc`: Acceleration trajectory over time.
- `new_jerk`: Jerk trajectory over time.

### Mathematical Formulation

1. **Normalized Time Vector:**

   $$\tau = \frac{t}{T}, \quad \text{where } 0 \leq t \leq T$$

   $\tau$ is a normalized time variable that scales the actual time $t$ to the range $[0, 1]$ over the segment duration $T$.

3. **Position Difference and Initial Velocity:**

    $$\text{pos\_diff} = \text{end\_pos} - \text{start\_pos}$$

    $$\text{vel\_diff} = -\text{start\_vel}$$

4. **Position Trajectory:**
    The position as a function of normalized time $\tau$ is given by:

    $$x(\tau) = x_0 + v_0 T \tau + \left( x_f - x_0 - v_0 T \right) \left(10\tau^3 - 15\tau^4 + 6\tau^5\right)$$

    The polynomial $10\tau^3 - 15\tau^4 + 6\tau^5$ ensures that the jerk is minimized. The coefficients are chosen such that the position starts at $x_0$ and ends at $x_f$ with smooth transitions.

5. **Velocity Trajectory:**
    Taking the first derivative of the position function with respect to time $t$:

    $$v(\tau) = v_0 + \frac{1}{T} \left( x_f - x_0 - v_0 T \right) \left(30\tau^2 - 60\tau^3 + 30\tau^4\right)$$

    Here, $\frac{d\tau}{dt} = \frac{1}{T}$.

6. **Acceleration Trajectory:**
    Taking the second derivative of the position function:

    $$a(\tau) = \frac{1}{T^2} \left( x_f - x_0 - v_0 T \right) \left(60\tau - 180\tau^2 + 120\tau^3\right)$$

7. **Jerk Trajectory:**
    Taking the third derivative of the position function:

    $$j(\tau) = \frac{1}{T^3} \left( x_f - x_0 - v_0 T \right) \left(60 - 360\tau + 360\tau^2\right)$$

### MATLAB Code Implementation

The provided MATLAB code implements these equations to compute the position, velocity, acceleration, and jerk trajectories over time.

#### Implementation:
```matlab
function [new_pos, new_vel, new_acc, new_jerk] = minJerk(start_pos, end_pos, start_vel, time_step, segment_time)
    T = segment_time; % Duration of the segment
    tau = (0:time_step:T)' / T; % Normalized time vector
    
    pos_diff = end_pos - start_pos; % Position difference
    vel_diff = -start_vel; % Velocity difference (initial velocity assumption)
    
    % Minimum jerk position trajectory considering initial velocity
    new_pos = start_pos + start_vel .* tau * T + (pos_diff - start_vel * T) .* (10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5);
    
    % Minimum jerk velocity trajectory considering initial velocity
    new_vel = start_vel + (pos_diff - start_vel * T) .* (30 * tau.^2 - 60 * tau.^3 + 30 * tau.^4) / T;
    
    % Minimum jerk acceleration trajectory
    new_acc = (pos_diff - start_vel * T) .* (60 * tau - 180 * tau.^2 + 120 * tau.^3) / T^2;
    
    % Minimum jerk jerk trajectory
    new_jerk = (pos_diff - start_vel * T) .* (60 - 360 * tau + 360 * tau.^2) / T^3;
end
```

#### Explanation:
1. **Normalized Time Vector (`tau`):**
    - $\tau$ scales the actual time steps to a normalized range.
2. **Position Difference (`pos_diff`) and Velocity Difference (`vel_diff`):**
    - Compute the differences between the start and end conditions.
3. **Position Trajectory (`new_pos`):**
    - Calculate the position at each time step using the minimum jerk polynomial.
4. **Velocity Trajectory (`new_vel`):**
    - Compute the velocity by differentiating the position polynomial.
5. **Acceleration Trajectory (`new_acc`):**
    - Compute the acceleration by further differentiating the position polynomial.
6. **Jerk Trajectory (`new_jerk`):**
    - Compute the jerk by taking the third derivative of the position polynomial.
