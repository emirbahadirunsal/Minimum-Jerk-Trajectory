#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>

// Global variables for current position and velocity
std::vector<double> pos = {0.0, 0.0, 0.0}; // Initial position [x, y, z]
std::vector<double> vel = {0.0, 0.0, 0.0}; // Initial velocity [vx, vy, vz]

// Function to calculate minimum jerk trajectory considering initial velocity
void minJerk(const std::vector<double>& start_pos, const std::vector<double>& end_pos, 
             const std::vector<double>& start_vel, double time_step, double segment_time,
             std::vector<std::vector<double>>& new_pos, std::vector<std::vector<double>>& new_vel,
             std::vector<std::vector<double>>& new_acc, std::vector<std::vector<double>>& new_jerk) {
    double T = segment_time; // Duration of the segment
    std::vector<double> tau((int)(T / time_step) + 1);
    for (size_t i = 0; i < tau.size(); ++i) {
        tau[i] = (i * time_step) / T;
    }

    std::vector<double> pos_diff = {end_pos[0] - start_pos[0], end_pos[1] - start_pos[1], end_pos[2] - start_pos[2]};

    new_pos.resize(tau.size(), std::vector<double>(3));
    new_vel.resize(tau.size(), std::vector<double>(3));
    new_acc.resize(tau.size(), std::vector<double>(3));
    new_jerk.resize(tau.size(), std::vector<double>(3));

    for (size_t i = 0; i < tau.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            new_pos[i][j] = start_pos[j] + start_vel[j] * tau[i] * T + (pos_diff[j] - start_vel[j] * T) * (10*std::pow(tau[i], 3) - 15*std::pow(tau[i], 4) + 6*std::pow(tau[i], 5));
            new_vel[i][j] = start_vel[j] + (pos_diff[j] - start_vel[j] * T) * (30*std::pow(tau[i], 2) - 60*std::pow(tau[i], 3) + 30*std::pow(tau[i], 4)) / T;
            new_acc[i][j] = (pos_diff[j] - start_vel[j] * T) * (60*tau[i] - 180*std::pow(tau[i], 2) + 120*std::pow(tau[i], 3)) / (T * T);
            new_jerk[i][j] = (pos_diff[j] - start_vel[j] * T) * (60 - 360*tau[i] + 360*std::pow(tau[i], 2)) / (T * T * T);
        }
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
    double total_time = 10.0; // Total time for the simulation in seconds
    int num_steps = static_cast<int>(total_time / time_step); // Number of steps

    // Preallocate arrays for storing position, velocity, acceleration, and jerk
    std::vector<std::vector<double>> positions(num_steps, std::vector<double>(3));
    std::vector<std::vector<double>> velocities(num_steps, std::vector<double>(3));
    std::vector<std::vector<double>> accelerations(num_steps, std::vector<double>(3));
    std::vector<std::vector<double>> jerks(num_steps, std::vector<double>(3));

    int step_index = 0; // Index to keep track of the step

    for (size_t i = 0; i < rect_vertices.size() - 1; ++i) {
        // Calculate the current vertex and the next vertex
        std::vector<double> current_vertex = rect_vertices[i];
        std::vector<double> next_vertex = rect_vertices[i + 1];

        // Calculate the minimum jerk trajectory for the current segment
        std::vector<std::vector<double>> segment_pos, segment_vel, segment_acc, segment_jerk;
        minJerk(current_vertex, next_vertex, vel, time_step, total_time / (rect_vertices.size() - 1),
                segment_pos, segment_vel, segment_acc, segment_jerk);

        // Store the results for the current segment
        for (size_t j = 0; j < segment_pos.size(); ++j) {
            if (step_index < num_steps) {
                positions[step_index] = segment_pos[j];
                velocities[step_index] = segment_vel[j];
                accelerations[step_index] = segment_acc[j];
                jerks[step_index] = segment_jerk[j];
                ++step_index;
            }
        }

        // Update the global position and velocity to the end of the current segment
        pos = next_vertex;
        vel = segment_vel.back();
    }

    // Write results to file
    std::ofstream outfile("minJerkTrajectory.txt");
    outfile << std::fixed << std::setprecision(6);
    outfile << "Time;X;Y;Z;Vx;Vy;Vz;Ax;Ay;Az;Jx;Jy;Jz\n";
    for (int i = 0; i < step_index; ++i) {
        double time = i * time_step;
        outfile << time << ";";
        for (int j = 0; j < 3; ++j) outfile << positions[i][j] << ";";
        for (int j = 0; j < 3; ++j) outfile << velocities[i][j] << ";";
        for (int j = 0; j < 3; ++j) outfile << accelerations[i][j] << ";";
        for (int j = 0; j < 3; ++j) outfile << jerks[i][j] << (j < 2 ? ";" : "\n");
    }

    outfile.close();
    return 0;
}
