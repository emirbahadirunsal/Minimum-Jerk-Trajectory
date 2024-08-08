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
const int endRun = 3000; // Define appropriately
double fullvelData[endRun][6]; // Cartesian velocity
double fulljointPos[endRun][6]; // Joint position
double fulltcpPos[endRun][6]; // TCP (tool) position, Cartesian position

double time_step = 0.008; // 8 milliseconds
int iterNum = 0;
std::vector<double> pos = {0.0, 0.0, 0.0}; // Initial position
std::vector<double> vel = {0.1, 0.0, 0.0}; // Initial velocity
double tolerance = 0.001; // Tolerance for movement

bool ESC = false; // Escape flag for terminating the loop

// File for writing results
std::ofstream outfile("minJerkTrajectory.txt");

// Function to send a message to the robot
void sendMessageToRobot(double Vx_ref, double Vy_ref, double Vz_ref) {
    char message[256];
    sprintf(message, "speedl([%1.5f,%1.5f,%1.5f,0.0,0.0,0.0],15.0,0.1)\n", Vx_ref, Vy_ref, Vz_ref);
    toUR5.sendMessageToServer(message);
}

// Global variables for the rectangle vertices and current vertex
std::vector<std::vector<double>> rect_vertices;
size_t current_vertex = 0;
std::vector<double> current_target;

// Function to calculate polynomial coefficients for minimum jerk trajectory
std::vector<double> calculateCoefficients(double x_i, double v_i, double a_i, double x_f, double v_f, double a_f, double T) {
    std::vector<double> coeffs(6);
    coeffs[0] = x_i;
    coeffs[1] = v_i;
    coeffs[2] = 0.5 * a_i;
    coeffs[3] = (20 * x_f - 20 * x_i - (8 * v_f + 12 * v_i) * T - (3 * a_i - a_f) * T * T) / (2 * T * T * T);
    coeffs[4] = (30 * x_i - 30 * x_f + (14 * v_f + 16 * v_i) * T + (3 * a_i - 2 * a_f) * T * T) / (2 * T * T * T * T);
    coeffs[5] = (12 * x_f - 12 * x_i - (6 * v_f + 6 * v_i) * T - (a_i - a_f) * T * T) / (2 * T * T * T * T * T);
    return coeffs;
}

// Function to calculate distance between two points
double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2) {
    return std::sqrt(std::pow(point2[0] - point1[0], 2) + std::pow(point2[1] - point1[1], 2) + std::pow(point2[2] - point1[2], 2));
}

// Function to calculate segment time based on distance
double calculateSegmentTime(const std::vector<double>& start, const std::vector<double>& end, double base_speed) {
    double distance = calculateDistance(start, end);
    return distance / base_speed;
}

// Optimized function to calculate and apply minimum jerk trajectory incrementally
void moveRect() {
    static size_t step = 0;
    static double total_time = 0.0; // Total elapsed time
    static std::vector<double> start_pos = pos;
    static std::vector<double> start_vel = vel;
    static double T = 2;
    static std::vector<double> pos_diff(3);
    static size_t steps = static_cast<int>(T / time_step) + 1;

    static std::vector<std::vector<double>> coeffs(3, std::vector<double>(6));

    // Initialize variables for a new segment if necessary
    if (step == 0) {
        if (current_vertex < rect_vertices.size()) {
            current_target = rect_vertices[current_vertex];
            if (std::fabs(pos[0] - current_target[0]) <= tolerance && std::fabs(pos[1] - current_target[1]) <= tolerance && std::fabs(pos[2] - current_target[2]) <= tolerance) {
                ++current_vertex;
                if (current_vertex < rect_vertices.size()) {
                    current_target = rect_vertices[current_vertex];
                }
            }
        }

        start_pos = pos;
        start_vel = vel;
        pos_diff = {current_target[0] - start_pos[0], current_target[1] - start_pos[1], current_target[2] - start_pos[2]};

        // Calculate new segment time
        T = calculateSegmentTime(start_pos, current_target, 0.2); // 0.1 is the base speed (adjust as necessary)
        steps = static_cast<int>(T / time_step) + 1;

        // Calculate coefficients for the minimum jerk polynomial
        for (int j = 0; j < 3; ++j) {
            coeffs[j] = calculateCoefficients(start_pos[j], start_vel[j], 0.0, current_target[j], 0.0, 0.0, T); // Assuming zero acceleration
        }
    }

    if (step < steps) {
        double tau = (step * time_step) / T;
        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;

        std::vector<double> new_pos(3), new_vel(3), new_acc(3), new_jerk(3);

        for (int j = 0; j < 3; ++j) {
            new_pos[j] = coeffs[j][0] + coeffs[j][1] * tau * T + coeffs[j][2] * tau2 * T * T + coeffs[j][3] * tau3 * T * T * T + coeffs[j][4] * tau4 * T * T * T * T + coeffs[j][5] * tau5 * T * T * T * T * T;
            new_vel[j] = coeffs[j][1] + 2 * coeffs[j][2] * tau * T + 3 * coeffs[j][3] * tau2 * T * T + 4 * coeffs[j][4] * tau3 * T * T * T + 5 * coeffs[j][5] * tau4 * T * T * T * T;
            new_acc[j] = 2 * coeffs[j][2] + 6 * coeffs[j][3] * tau * T + 12 * coeffs[j][4] * tau2 * T * T + 20 * coeffs[j][5] * tau3 * T * T * T;
            new_jerk[j] = 6 * coeffs[j][3] + 24 * coeffs[j][4] * tau * T + 60 * coeffs[j][5] * tau2 * T * T;
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
    rect_vertices = {
        pos,
        {0.1, 0.1, 0},
        {0.5, 0.1, 0},
        {0.5, 0.3, 0},
        {0.1, 0.5, 0},
        {0.1, 0.1, 0}
    };

    // Initialize current target
    current_target = rect_vertices[current_vertex];

    // Write header to the file
    outfile << "Time;X;Y;Z;Vx;Vy;Vz;Ax;Ay;Az;Jx;Jy;Jz\n";

    while (iterNum < endRun && !ESC) {
        moveRect();
        ++iterNum;
    }

    outfile.close();
    return 0;
}
