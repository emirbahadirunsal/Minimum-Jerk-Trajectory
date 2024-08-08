#ifndef MIN_JERK_TRAJECTORY_H
#define MIN_JERK_TRAJECTORY_H

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

    void sendMessageToServer(const char* message);
};

// Global variables for actual position and velocity
extern const int endRun; // Define appropriately
extern double fullvelData[][6]; // Cartesian velocity
extern double fulljointPos[][6]; // Joint position
extern double fulltcpPos[][6]; // TCP (tool) position, Cartesian position

extern double time_step; // 8 milliseconds
extern int iterNum;
extern std::vector<double> pos; // Initial position
extern std::vector<double> vel; // Initial velocity
extern double tolerance; // Tolerance for movement

extern bool ESC; // Escape flag for terminating the loop

// File for writing results
extern std::ofstream outfile;

// Function to send a message to the robot
void sendMessageToRobot(double Vx_ref, double Vy_ref, double Vz_ref);

// Global variables for the rectangle vertices and current vertex
extern std::vector<std::vector<double>> rect_vertices;
extern size_t current_vertex;
extern std::vector<double> current_target;

// Function to calculate polynomial coefficients for minimum jerk trajectory
std::vector<double> calculateCoefficients(double x_i, double v_i, double a_i, double x_f, double v_f, double a_f, double T);

// Function to calculate distance between two points
double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2);

// Function to calculate segment time based on distance
double calculateSegmentTime(const std::vector<double>& start, const std::vector<double>& end, double base_speed);

// Optimized function to calculate and apply minimum jerk trajectory incrementally
void moveRect();

// Main function
int main();

#endif // MIN_JERK_TRAJECTORY_H
