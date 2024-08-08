#pragma once

#include <SDKDDKVer.h>  // To remove preprocessor warning
#define _WIN32_WINNT 0x0A00

#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/math/special_functions/gamma.hpp>
#include <boost/lambda/lambda.hpp>
#include <fstream>
#include <dos.h>
#include <process.h>
#include <time.h>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <conio.h>
#include <ctime>
#include <math.h>
#include "decoupledACforUR5.h"
#include "atisensors.h" // ATI sensor file
#include "clientSocket.h"
#include <iterator>
#include <algorithm>
#include <GL/glew.h>
#include <GL/freeglut.h>

// Mode definitions
#define AdmittanceControlMode 0
#define RobotInitializingMode 9
#define UserInitializingMode 10

using namespace std;

#undef UNICODE
#define WIN32_LEAN_AND_MEAN

// Define macros
#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array[0])))
#define endRun 600000

#define accInSpeed 15.0 // Acceleration upper limit for speed

// Global variables
int a;
float timeToStayInSpeed = 0.1;
int status = RobotInitializingMode;
bool isControllerInitialized = false;
DWORD threadID1;

// Storage variables for read data
double fullvelData[endRun][6]; // Cartesian velocity
double fullactualSpeedVector[endRun][6]; // Not used
double fulljointPos[endRun][6]; // Joint position
double fulltcpPos[endRun][6]; // TCP (tool) position, Cartesian position
double UR5Time;
double temp_tcpAcc[3];
double temp_targetJointTorques[6];
double temp_targetJointCurrents[6];
double temp_actualJointCurrents[6];
double temp_generalizedTCPforces[6];
double temp_targetJointAccs[6];
double temp_targetJointVels[6];
double temp_targetJointPoss[6];
double temp_actualJointVels[6];
double tcpAcc[endRun][3];
double targetJointTorques[endRun][6];
double targetJointCurrents[endRun][6];
double actualJointCurrents[endRun][6];
double generalizedTCPforces[endRun][6];
double targetJointAccs[endRun][6];
double targetJointVels[endRun][6];
double targetJointPoss[endRun][6];
double actualJointVels[endRun][6];
double actualSpeedVector[6];
double jointPos[6];
double tcpPos[6];
double averageSensorForcesR[endRun][6];
double averageSensorForcesH[endRun][6];
double averageSensorForcesRU[endRun][6]; // SARA
double averageSensorForcesHU[endRun][6]; // SARA
float TimeReal[endRun];
double BytesTranferred[endRun];
double dt = 0.008;
int iterationNum = 0;
double loopTimes[endRun];
double controllerTime[endRun];
double controllerRealTime[endRun];
double robotModeArray[endRun];
double tempjointMode[6];
double tempUnused[15];
double loopInstant[endRun];
int sleepArray[endRun];
double loopTimesOld[endRun];
double delayUStime[endRun];
float FRX[endRun], FRY[endRun], FRZ[endRun], FHX[endRun], FHY[endRun], FHZ[endRun], TRX[endRun], TRY[endRun], TRZ[endRun], THX[endRun], THY[endRun], THZ[endRun];

LARGE_INTEGER startTick, startLoopTick, endLoopTick, sysFreq, readThreadTick, GlobalStartTick;
int timeSec;
float elapsedTime;
int exitThread = 0;
int endProcessThread = 0;
int oldRunTime = 0;
static int totRuns = 0;
int thisTotRun = 0;
int currentTotRun = 0;

char message[100];
double JointAngleError;
double InstantJointVelocity;

double GAIN = 1.0; // Adaptive Gain

float FTR[6]; // Resultant force/torque vector for robot sensor (interaction force sensor)
float FTH[6]; // Resultant force/torque vector for human sensor (human force sensor)
float FTRU[6]; // Unbiased force - robot sensor (SARA)
float FTHU[6]; // Unbiased force - human sensor (SARA)

decoupledIOAC dcAC; // Decoupled admittance controller

CRITICAL_SECTION GUI_cs; // Critical section for GUI

// Struct to handle temporary force/torque sensor data
struct tempFT {
    std::array<double, 6> tempFT_interaction; // Interaction force/torque
    std::array<double, 6> tempFT_human; // Human force/torque
    std::array<double, 6> tempFT_interactionU; // Unbiased interaction force/torque (SARA)
    std::array<double, 6> tempFT_humanU; // Unbiased human force/torque (SARA)
    std::array<float, 6> tempV_interaction; // Interaction sensor voltages
    std::array<float, 6> tempV_human; // Human sensor voltages
    std::array<double, 12> FT_array; // General force/torque array
    std::array<double, 12> FT_arrayU; // General unbiased force/torque array (SARA)
    std::array<double, 12> V_array; // General voltage array
    int voltage_saturation_flag; // Voltage saturation flag
    int force_saturation_flag; // Force saturation flag

    tempFT() {
        setZero();
    }

    void makeGeneralArray() {
        for (int i = 0; i < 6; i++) {
            FT_array[i] = tempFT_interaction[i];
            FT_array[i + 6] = tempFT_human[i];
            FT_arrayU[i] = tempFT_interactionU[i];
            FT_arrayU[i + 6] = tempFT_humanU[i];
            V_array[i] = tempV_interaction[i];
            V_array[i + 6] = tempV_human[i];
        }
    }

    void read_FTint(array<double, 6> FTint) {
        tempFT_interaction = FTint;
    }

    void read_FThuman(array<double, 6> FThuman) {
        tempFT_human = FThuman;
    }

    void read_FTintU(array<double, 6> FTintU) {
        tempFT_interactionU = FTintU;
    }

    void read_FThumanU(array<double, 6> FThumanU) {
        tempFT_humanU = FThumanU;
    }

    void read_VoltInt(float Vint[6]) {
        memcpy(tempV_interaction.data(), Vint, 6 * sizeof(float));
    }

    void read_VoltHuman(float Vhuman[6]) {
        memcpy(tempV_human.data(), Vhuman, 6 * sizeof(float));
    }

    void read(atisensors atiObj, float robot_force[6], float human_force[6], float robot_forceU[6], float human_forceU[6]) {
        read_FTint(atiObj.averageLastForce2);
        read_FThuman(atiObj.averageLastForce1);
        read_FTintU(atiObj.averageLastForce2U);
        read_FThumanU(atiObj.averageLastForce1U);
        read_VoltInt(atiObj.lastVoltages2);
        read_VoltHuman(atiObj.lastVoltages1);
        voltage_saturation_flag = atiObj.saturationVoltageCheck;
        force_saturation_flag = atiObj.force_saturation_check;

        if (force_saturation_flag == 2) {
            cout << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            cout << "force saturation warning!!!!!!!!!!!!!!!" << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" << endl;
        }

        for (int i = 0; i < 6; i++) {
            robot_force[i] = tempFT_interaction[i];
            human_force[i] = tempFT_human[i];
            robot_forceU[i] = tempFT_interactionU[i];
            human_forceU[i] = tempFT_humanU[i];
        }
    }

    void setZero() {
        tempFT_human = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        tempFT_interaction = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        tempFT_humanU = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        tempFT_interactionU = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        tempV_interaction = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        tempV_human = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        voltage_saturation_flag = 0;
        force_saturation_flag = 0;
    }
};

// Timing functions
void waitFor(double secs) {
    double retTime = time(0) + secs;
    while (time(0) < retTime);
}

void MYuSleep(int waitTime) {
    LARGE_INTEGER time1, time2, freq;
    QueryPerformanceCounter(&time1);
    QueryPerformanceFrequency(&freq);

    do {
        QueryPerformanceCounter(&time2);
    } while (((time2.QuadPart - time1.QuadPart) * 1000000 / sysFreq.QuadPart) < waitTime);
    
    delayUStime[iterationNum] = (time2.QuadPart - time1.QuadPart) * 1000000 / sysFreq.QuadPart;
}

// Parse joint angles from a move command message
double* parse_joint_angles(char* movel_message) {
    stringstream movel_stream(movel_message);
    string temp_string;

    getline(movel_stream, temp_string, '[');
    getline(movel_stream, temp_string, '[');
    stringstream movel_stream_2(temp_string);
    getline(movel_stream_2, temp_string, ']');
    stringstream movel_stream_mid(temp_string);
    
    double* joint_angles = new double[6];
    for (int i = 0; i < 6; i++) {
        getline(movel_stream_mid, temp_string, ',');
        joint_angles[i] = atof(temp_string.c_str());
    }
    return joint_angles;
}

// Calculate joint angle error
double calculate_joint_angle_error(char* target_q_char, double* actual_q) {
    double* target_q = parse_joint_angles(target_q_char);
    JointAngleError = sqrt(pow((target_q[0] - actual_q[0]), 2) + pow((target_q[1] - actual_q[1]), 2) + pow((target_q[2] - actual_q[2]), 2) +
                           pow((target_q[3] - actual_q[3]), 2) + pow((target_q[4] - actual_q[4]), 2) + pow((target_q[5] - actual_q[5]), 2));
    cout << "\nJoint Error:\t" << JointAngleError << endl;
    return JointAngleError;
}

// Calculate instantaneous speed
double calculate_instantaneous_speed(double* actual_qdot) {
    InstantJointVelocity = sqrt(pow(actual_qdot[0], 2) + pow(actual_qdot[1], 2) + pow(actual_qdot[2], 2));
    cout << "Instant Joint Velocity\t" << InstantJointVelocity << endl;
    return InstantJointVelocity;
}
