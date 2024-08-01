#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <thread>

// Placeholder for the robot's communication function
struct RobotCommunicator {
    int messageCount = 0; // Counter for the number of messages sent
    std::chrono::time_point<std::chrono::steady_clock> previous_time = std::chrono::steady_clock::now();

    void sendMessageToServer(const char* message) {
        // Increment message counter
        ++messageCount;

        // Print the message and average time per iteration at every 50 iterations
        if (messageCount % 20 == 0) {
            // Get the current time
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - previous_time).count();
            double average_time_per_iteration = duration / 20.0;

            std::cout << "Average time per iteration: " << average_time_per_iteration << " ms - Sending message: " << message << std::endl;

            // Update the previous time to the current time
            previous_time = now;
        }
    }
} toUR5;

// Global variables for actual position and velocity
const int endRun = 10000; // Define appropriately
double fullvelData[endRun][6]; // Cartesian velocity
double fulljointPos[endRun][6]; // Joint position
double fulltcpPos[endRun][6]; // TCP (tool) position, Cartesian position

int iterNum = 0;
std::vector<double> pos = {0.0, 0.0, 0.0}; // Initial position
std::vector<double> vel = {0.0, 0.0, 0.0}; // Initial velocity
double tolerance = 0.01; // Tolerance for movement

bool ESC = false; // Escape flag for terminating the loop

// File for writing results
std::ofstream outfile("minJerkTrajectory.txt");

// Function to send a message to the robot
void sendMessageToRobot(double Vx_ref, double Vy_ref, double Vz_ref) {
    char message[256];
    sprintf(message, "speedl([%1.5f,%1.5f,%1.5f,0.0,0.0,0.0],15.0,0.1)\n", Vx_ref, Vy_ref, Vz_ref);
    toUR5.sendMessageToServer(message);
}

// Optimized function to calculate and apply minimum jerk trajectory incrementally
void moveRect(const std::vector<double>& target_pos, double time_step, double segment_time) {
    static size_t step = 0;
    static double total_time = 0.0; // Total elapsed time
    static std::vector<double> start_pos = pos;
    static std::vector<double> start_vel = vel;
    static double T = segment_time;
    static std::vector<double> pos_diff(3);
    static size_t steps = static_cast<int>(T / time_step) + 1;

    if (step == 0) {
        // Initialize variables for a new segment
        start_pos = pos;
        start_vel = vel;
        T = segment_time;
        pos_diff = {target_pos[0] - start_pos[0], target_pos[1] - start_pos[1], target_pos[2] - start_pos[2]};
        steps = static_cast<int>(T / time_step) + 1;
    }

    if (step < steps) {
        double tau = (step * time_step) / T;
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;

        double common_term_1 = 10 * tau3 - 15 * tau4 + 6 * tau5;
        double common_term_2 = 30 * tau2 - 60 * tau3 + 30 * tau4;
        double common_term_3 = 60 * tau - 180 * tau2 + 120 * tau3;
        double common_term_4 = 60 - 360 * tau + 360 * tau2;

        std::vector<double> new_pos(3), new_vel(3), new_acc(3), new_jerk(3);

        for (int j = 0; j < 3; ++j) {
            new_pos[j] = start_pos[j] + start_vel[j] * tau * T + (pos_diff[j] - start_vel[j] * T) * common_term_1;
            new_vel[j] = start_vel[j] + (pos_diff[j] - start_vel[j] * T) * common_term_2 / T;
            new_acc[j] = (pos_diff[j] - start_vel[j] * T) * common_term_3 / (T * T);
            new_jerk[j] = (pos_diff[j] - start_vel[j] * T) * common_term_4 / (T * T * T);
        }

        // Send velocity command to the robot
        sendMessageToRobot(new_vel[0], new_vel[1], new_vel[2]);

        // Write data to the file
        outfile << std::fixed << std::setprecision(6);
        outfile << total_time << ";"; // Use total_time instead of step * time_step
        for (int j = 0; j < 3; ++j) outfile << new_pos[j] << ";";
        for (int j = 0; j < 3; ++j) outfile << new_vel[j] << ";";
        for (int j = 0; j < 3; ++j) outfile << new_acc[j] << ";";
        for (int j = 0; j < 3; ++j) outfile << new_jerk[j] << (j < 2 ? ";" : "\n");

        // Update global position and velocity variables
        pos = new_pos;
        vel = new_vel;

        // Increment the step and total_time
        ++step;
        total_time += time_step;
    } else {
        // Reset step for the next segment
        step = 0;
    }
}

int main() {
    // Rectangle vertices in the XY plane
    std::vector<std::vector<double>> rect_vertices = {
        pos,
        {0.1, 0.1, 0},
        {0.5, 0.1, 0},
        {0.5, 0.5, 0},
        {0.1, 0.5, 0},
        {0.1, 0.1, 0}
    };

    // Time settings
    double time_step = 0.008; // 8 milliseconds
    double segment_time = 2.0; // Time to complete each segment

    size_t current_vertex = 0;

    // Write header to the file
    outfile << "Time;X;Y;Z;Vx;Vy;Vz;Ax;Ay;Az;Jx;Jy;Jz\n";

    while (iterNum < endRun && !ESC) {

        if (current_vertex < rect_vertices.size()) {
            std::vector<double>& current_target = rect_vertices[current_vertex];
            if (std::fabs(pos[0] - current_target[0]) > tolerance || std::fabs(pos[1] - current_target[1]) > tolerance || std::fabs(pos[2] - current_target[2]) > tolerance) {
                moveRect(current_target, time_step, segment_time);
            } else {
                ++current_vertex;
                if (current_vertex < rect_vertices.size()) {
                    // Update the position to the current target for the next segment
                    pos = current_target;
                }
            }
        }

        ++iterNum;
    }

    outfile.close();
    return 0;
}
