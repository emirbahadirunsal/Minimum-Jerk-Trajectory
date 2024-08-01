**Minimum Jerk Trajectory Task**

You need to write codes for minimum jerk trajectory following a rectangle. Details are given below.

### Part 1 (MATLAB):
1. **Define Initial Conditions**:
   - Define initial velocity and initial position as variables. These should be easily changeable.
   - Velocities and positions are in three dimensions (x, y, z).
   - Current positions should be global variables, so they can be accessed in every function.

2. **Create `minJerk()` Function**:
   - Write a function named `minJerk()` that calculates positions, velocities, accelerations, and jerks for a minimum jerk trajectory, considering the initial velocity.

3. **Main Script Execution**:
   - The main script should iterate every 0.008 seconds (8 milliseconds) to simulate the movement along the rectangle path.

4. **Assumption**:
   - Assume that there are no actuator limitations, meaning the current position is the desired position.

5. **Plotting**:
   - At the end of the simulation, plot the XY trajectory in one figure.
   - Plot the positions, velocities, accelerations, and jerks over time in a separate figure with subplots.

### Part 2 (C++):
1. **Write a C++ Program**:
   - Create a C++ program that calculates and writes the minimum jerk trajectory data to a text file named `minJerkTrajectory.txt`.

2. **Write to File**:
   - Output the calculated data to `minJerkTrajectory.txt` in the format: `Time;X;Y;Z;Vx;Vy;Vz;Ax;Ay;Az;Jx;Jy;Jz`.

3. **MATLAB Plotting**:
   - Write a MATLAB code to read and plot the data from `minJerkTrajectory.txt` to visualize the trajectory.
   - The MATLAB code should use `readtable` to read the data and plot the XY trajectory, as well as positions, velocities, accelerations, and jerks over time in separate subplots.

### Part 3 (Real-life Deployment with Actuator Limitations):
1. **Global Definitions**:
   - Define global variables for actual robot position and velocity as follows:
   ```cpp
   double fullvelData[endRun][6]; // fullvelData[iterationNum][j] = actualSpeedVector[j]; // Cartesian velocity
   double fullactualSpeedVector[endRun][6]; // Not used
   double fulljointPos[endRun][6]; // Joint position
   double fulltcpPos[endRun][6]; // TCP (tool) position, Cartesian position
   ```

2. **Main Loop**:
   - The main loop should iterate while `iterNum` is less than `endRun` or until an ESC condition is met:
   ```cpp
   int main(){
       while(iterNum < endRun || ESC){
           if (target - current > tol)
               moveRect();
           iterNum++;
       }
   }
   ```

3. **Sending Messages to the Robot**:
   - Implement the function to send messages to the robot to move to the desired position:
   ```cpp
   sprintf(message, "speedl([%1.5f,%1.5f,%1.5f,%1.5f,%1.5f,%1.5f],15.0,0.1)\n", Vx_ref, Vy_ref, Vz_ref, 0.0, 0.0, 0.0);
   toUR5.sendMessageToServer(message);
   ```

4. **Convert Previous C++ Code for Real-life Implementation**:
   - Adjust your previous C++ code to implement real-life behavior with actuator limitations. The main steps include:
     - Calculating the minimum jerk trajectory.
     - Sending the velocity command to the robot.
     - Reading the actual position and velocity from the robot.
     - Updating global position and velocity based on feedback from the robot.
