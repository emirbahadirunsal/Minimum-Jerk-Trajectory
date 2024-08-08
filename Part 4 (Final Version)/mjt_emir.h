#ifndef MJT_EMIR_H
#define MJT_EMIR_H

#include <vector>

// Global variables
extern const int endRun;
extern double fullvelData[endRun][6];
extern double fulljointPos[endRun][6];
extern double fulltcpPos[endRun][6];
extern double time_step;
extern int iterNum;
extern std::vector<double> vel;
extern double tolerance;
extern bool ESC;
extern std::vector<std::vector<double>> rect_vertices;
extern size_t current_vertex;
extern std::vector<double> current_target;

// Function declarations
void sendMessageToRobot(double Vx_ref, double Vy_ref, double Vz_ref);
std::vector<double> calculateCoefficients(double x_i, double v_i, double a_i, double x_f, double v_f, double a_f, double T);
double calculateDistance(const std::vector<double>& point1, const std::vector<double>& point2);
double calculateSegmentTime(const std::vector<double>& start, const std::vector<double>& end, double base_speed);
void moveRect();

#endif // MJT_EMIR_H
