#include "BaseUR5Code.h"
#include "mjt_emir.h"

// Global variable declarations for boost asio to listen robot continuously
boost::asio::io_service io_service;
boost::asio::ip::tcp::resolver resolver(io_service);
boost::asio::ip::tcp::socket sock(io_service);
boost::array<char, 812> buffer;

// Initial configuration command for the robot
char* initConf = "movej([1.57079, -1.57079, 2.35619, -0.78539, 1.57079, 0.0000],a=3.0,v=0.15,t=0.0,r=0.0)\n"; 

// File Writing
FILE* fJointData; // File pointer to write sensor data
errno_t err;

// Sensor Handling
atisensors ftsensors;
tempFT _tempFT;

// TCP Client Establishing
clientSocket toUR5; // Object for socket programming

// Data Recording in real-time
std::array<double, 132> iteration_based_logging;
std::vector<std::array<double, 132>> data_logs;
CONDITION_VARIABLE AlarmforDataRecording;
CRITICAL_SECTION DataRecordingCS;
bool consumeLeftovers = false;
unsigned long LastIndexforDataRecording = 0;
int BUFFERSIZE = 20;

// Define constants and variables from mjt_emir
const int endRun = 3000;
double fullvelData[endRun][6]; // Cartesian velocity
double fulljointPos[endRun][6]; // Joint position
double fulltcpPos[endRun][6]; // TCP (tool) position, Cartesian position
double time_step = 0.008; // 8 milliseconds
int iterNum = 0;
std::vector<double> vel = {0.1, 0.0, 0.0}; // Initial velocity
double tolerance = 0.001; // Tolerance for movement
bool ESC = false; // Escape flag for terminating the loop
std::vector<std::vector<double>> rect_vertices;
size_t current_vertex = 0;
std::vector<double> current_target;

// Function to send a message to the robot
void sendMessageToRobot(double Vx_ref, double Vy_ref, double Vz_ref) {
    char message[256];
    sprintf(message, "speedl([%1.5f,%1.5f,%1.5f,0.0,0.0,0.0],15.0,0.1)\n", Vx_ref, Vy_ref, Vz_ref);
    toUR5.sendMessageToServer(message);
}

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
    static std::vector<double> start_pos = {fulltcpPos[iterNum][0], fulltcpPos[iterNum][1], fulltcpPos[iterNum][2]};
    static std::vector<double> start_vel = vel;
    static double T = 2;
    static std::vector<double> pos_diff(3);
    static size_t steps = static_cast<int>(T / time_step) + 1;

    static std::vector<std::vector<double>> coeffs(3, std::vector<double>(6));

    // Initialize variables for a new segment if necessary
    if (step == 0) {
        if (current_vertex < rect_vertices.size()) {
            current_target = rect_vertices[current_vertex];
            if (std::fabs(start_pos[0] - current_target[0]) <= tolerance && std::fabs(start_pos[1] - current_target[1]) <= tolerance && std::fabs(start_pos[2] - current_target[2]) <= tolerance) {
                ++current_vertex;
                if (current_vertex < rect_vertices.size()) {
                    current_target = rect_vertices[current_vertex];
                }
            }
        }

        start_pos = {fulltcpPos[iterNum][0], fulltcpPos[iterNum][1], fulltcpPos[iterNum][2]};
        start_vel = vel;
        pos_diff = {current_target[0] - start_pos[0], current_target[1] - start_pos[1], current_target[2] - start_pos[2]};

        // Calculate new segment time
        T = calculateSegmentTime(start_pos, current_target, 0.2); // 0.2 is the base speed (adjust as necessary)
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

        // Increment the step and total_time
        ++step;
        total_time += time_step;
    } else {
        // Reset step for the next segment
        step = 0;
    }
}

// Data recording thread function
void DataRecording(void* pparams) {
    LastIndexforDataRecording = 0;
    std::vector<std::array<double, 132>> temp_data_logs;
    int recording_counter = 0;
    unsigned long long temp_iteration_num;
    
    while (true) {
        recording_counter = 0;
        SleepConditionVariableCS(&AlarmforDataRecording, &DataRecordingCS, INFINITE);
        EnterCriticalSection(&DataRecordingCS);
        temp_iteration_num = iterationNum;
        
        if (!consumeLeftovers) {
            temp_data_logs = { data_logs.begin() + LastIndexforDataRecording, data_logs.begin() + LastIndexforDataRecording + BUFFERSIZE };
        } else {
            temp_data_logs = { data_logs.begin() + LastIndexforDataRecording, data_logs.end() };
        }
        LeaveCriticalSection(&DataRecordingCS);

        for (const auto& log : temp_data_logs) {
            for (const double& datum : log) {
                fprintf(fJointData, "%f;", datum);
            }
            fprintf(fJointData, "\n");
            recording_counter++;
        }

        if (consumeLeftovers) {
            std::cout << "IterationNum: " << temp_iteration_num << "\nLastIndexforDataRecording:" << LastIndexforDataRecording << "\nRecorded: " << recording_counter << std::endl;
            std::cout << "Please press CTRL+C to exit..." << std::endl;
        }
        LastIndexforDataRecording += recording_counter;
    }
}

// Biasing the sensor and read values, save in arrays
void readFTvalues_Once() {
    if (ftsensors.biasIsCompleted == 1) {
        EnterCriticalSection(&ftsensors.cs1);
        _tempFT.read(ftsensors, FTR, FTH, FTRU, FTHU);
        LeaveCriticalSection(&ftsensors.cs1);
        
        if (iterationNum == 0) {
            QueryPerformanceCounter(&startTick);
        }

        oldRunTime = thisTotRun;
    } else {
        _tempFT.setZero();
    }
}

// Open sensor file for writing
void openJointDataFile(char* appendix = "") {
    std::string filename = "fJointData";
    filename.append(appendix);
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "_%d_%b_%y_%H_%M_%S.txt");
    filename.append(oss.str());

    if ((err = fopen_s(&fJointData, filename.c_str(), "w")) != 0) {
        char sys_msg[64];
        strerror_s(sys_msg, sizeof sys_msg, err);
        fprintf(stderr, "cannot open file '%s': %s\n", filename.c_str(), sys_msg);
    } else {
        fprintf(fJointData, "V0_int;V1_int;V2_int;V3_int;V4_int;V5_int;");
        fprintf(fJointData, "V0_hum;V1_hum;V2_hum;V3_hum;V4_hum;V5_hum;");
        fprintf(fJointData, "Fx_b_int;Fy_b_int;Fz_b_int;Tx_b_int;Ty_b_int;Tz_b_int;");
        fprintf(fJointData, "Fx_b_hum;Fy_b_hum;Fz_b_hum;Tx_b_hum;Ty_b_hum;Tz_b_hum;");
        fprintf(fJointData, "Fx_ub_int;Fy_ub_int;Fz_ub_int;Tx_ub_int;Ty_ub_int;Tz_ub_int;");
        fprintf(fJointData, "Fx_ub_hum;Fy_ub_hum;Fz_ub_hum;Tx_ub_hum;Ty_ub_hum;Tz_ub_hum;");
        fprintf(fJointData, "q1_t;q2_t;q3_t;q4_t;q5_t;q6_t;");
        fprintf(fJointData, "q1_a;q2_a;q3_a;q4_a;q5_a;q6_a;");
        fprintf(fJointData, "qd1_t;qd2_t;qd3_t;qd4_t;qd5_t;qd6_t;");
        fprintf(fJointData, "qd1_a;qd2_a;qd3_a;qd4_a;qd5_a;qd6_a;");
        fprintf(fJointData, "qdd1_t;qdd2_t;qdd3_t;qdd4_t;qdd5_t;qdd6_t;");
        fprintf(fJointData, "p1;p2;p3;p4;p5;p6;");
        fprintf(fJointData, "pd1;pd2;pd3;pd4;pd5;pd6;");
        fprintf(fJointData, "t1_t;t2_t;t3_t;t4_t;t5_t;t6_t;");
        fprintf(fJointData, "i1_t;i2_t;i3_t;i4_t;i5_t;i6_t;");
        fprintf(fJointData, "i1_a;i2_a;i3_a;i4_a;i5_a;i6_a;");
        fprintf(fJointData, "f1;f2;f3;f4;f5;f6;");
        fprintf(fJointData, "pdd1;pdd2;pdd3;");
        fprintf(fJointData, "time_1;time_2;time_3;bytes;iterationNum;RobotMode;");
        fprintf(fJointData, "j1_info;j2_info;j3_info;j4_info;j5_info;j6_info;");
        fprintf(fJointData, "unused1;unused2;unused3;unused4;unused5;unused6;unused7;unused8;unused9;unused10;unused11;unused12;unused13;unused14;unused15;\n");

        printf("> Creating the %s file...Done!\n", filename.c_str());
    }
}

// Read all encoder & sensor bits and convert to F/T data
void parseJointData() {
    char TarJointTorq[8], TarJointCur[8], ActJointCur[8], GenTCPforce[8];
    char TarJointAcc[8], TarJointVel[8], TarJointPos[8], ActJointVel[8];
    char myTempDoubleTCPAcc[8], myTempDoubleSpeed[8], myTempDoubleJointPos[8], myTempDoubleTCPpos[8];
    char myTempDoubleTime[8], myTempDoubleControllerTime[8], robotMode[8], jointMode[8], unused[8];

    for (int j = 0; j < 15; j++) {
        for (int i = 0; i < 8; i++) {
            unused[7 - i] = buffer[420 + i + 8 * j];
        }
        memcpy(&temp_targetJointTorques[j], unused, sizeof(double)); // Target Joint Torques
    }

    for (int j = 0; j < 6; j++) {
        for (int i = 0; i < 8; i++) {
            TarJointPos[7 - i] = buffer[12 + i + 8 * j];
            TarJointVel[7 - i] = buffer[60 + i + 8 * j];
            TarJointAcc[7 - i] = buffer[108 + i + 8 * j];
            TarJointCur[7 - i] = buffer[156 + i + 8 * j];
            TarJointTorq[7 - i] = buffer[204 + i + 8 * j];
            myTempDoubleJointPos[7 - i] = buffer[252 + i + 8 * j];
            ActJointVel[7 - i] = buffer[300 + i + 8 * j];
            ActJointCur[7 - i] = buffer[348 + i + 8 * j];
            GenTCPforce[7 - i] = buffer[540 + i + 8 * j];
            myTempDoubleTCPpos[7 - i] = buffer[588 + i + 8 * j];
            myTempDoubleSpeed[7 - i] = buffer[636 + i + 8 * j];
            jointMode[7 - i] = buffer[764 + i + 8 * j];
        }
        memcpy(&temp_targetJointTorques[j], TarJointTorq, sizeof(double));
        memcpy(&temp_targetJointCurrents[j], TarJointCur, sizeof(double));
        memcpy(&temp_actualJointCurrents[j], ActJointCur, sizeof(double));
        memcpy(&temp_generalizedTCPforces[j], GenTCPforce, sizeof(double));
        memcpy(&temp_targetJointAccs[j], TarJointAcc, sizeof(double));
        memcpy(&temp_targetJointVels[j], TarJointVel, sizeof(double));
        memcpy(&temp_actualJointVels[j], ActJointVel, sizeof(double));
        memcpy(&temp_targetJointPoss[j], TarJointPos, sizeof(double));
        memcpy(&jointPos[j], myTempDoubleJointPos, sizeof(double));
        memcpy(&actualSpeedVector[j], myTempDoubleSpeed, sizeof(double));
        memcpy(&tcpPos[j], myTempDoubleTCPpos, sizeof(double));
        memcpy(&tempjointMode[j], jointMode, sizeof(double));

        fullvelData[iterationNum][j] = actualSpeedVector[j];
        fulljointPos[iterationNum][j] = jointPos[j];
        fulltcpPos[iterationNum][j] = tcpPos[j];
        targetJointTorques[iterationNum][j] = temp_targetJointTorques[j];
        targetJointCurrents[iterationNum][j] = temp_targetJointCurrents[j];
        actualJointCurrents[iterationNum][j] = temp_actualJointCurrents[j];
        generalizedTCPforces[iterationNum][j] = temp_generalizedTCPforces[j];
        targetJointAccs[iterationNum][j] = temp_targetJointAccs[j];
        targetJointVels[iterationNum][j] = temp_targetJointVels[j];
        targetJointPoss[iterationNum][j] = temp_targetJointPoss[j];
        actualJointVels[iterationNum][j] = temp_actualJointVels[j];
    }

    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 8; i++) {
            myTempDoubleTCPAcc[7 - i] = buffer[396 + i + 8 * j];
        }
        memcpy(&temp_tcpAcc[j], myTempDoubleTCPAcc, sizeof(double));
        tcpAcc[iterationNum][j] = temp_tcpAcc[j];
    }

    for (int i = 0; i < 8; i++) {
        myTempDoubleTime[7 - i] = buffer[4 + i];
        myTempDoubleControllerTime[7 - i] = buffer[740 + i];
        robotMode[7 - i] = buffer[756 + i];
    }
    memcpy(&controllerTime[iterationNum], myTempDoubleTime, sizeof(double));
    memcpy(&controllerRealTime[iterationNum], myTempDoubleControllerTime, sizeof(double));
    memcpy(&robotModeArray[iterationNum], robotMode, sizeof(double));
}

// Clean up and end operations
void endingOperations() {
    io_service.stop();
    io_service.reset();
    exitThread = 1;

    printf("Program terminates...\n");
    toUR5.closeConnectionToServer();
    waitFor(3);
}

void fill_iteration_based_logging() {
    /*
    Data Recording: Features =
        Gauge Voltages for interaction and human sensor [12] 0 - 11
        Biased Force value for interaction and human sensor [12] 12- 23
        Unbiased Force value for interaction and human sensor [12] 24-35
        Target Joint Positions [6] 36
        Actual Joint Positions [6] 42
        Target Joint Velocities [6] 48
        Actual Joint Velocities [6] 54
        Target Joint Acceleration [6] 60
        TCP Positions [6] 66
        TCP Velocities [6] 72
        Target Joint Torques [6] 78 
        Target Joint Currents [6] 84
        Actual Joint Currents [6] 90
        TCP Forces [6] 96 
        TCP Accelerations [3] 99 
        Controller Time [1] 100
        Controller RealTime [1] 101
        Time Real [1] 102
        Transferred Bytes [1] 103
    */
    EnterCriticalSection(&DataRecordingCS);
    for (int j = 0; j < 6; j++) {
        iteration_based_logging[j] = _tempFT.tempV_interaction[j]; // Gauge Voltages of Interaction Sensor
        iteration_based_logging[6 + j] = _tempFT.tempV_human[j]; // Gauge Voltages of Human Sensor
        iteration_based_logging[12 + j] = _tempFT.tempFT_interaction[j]; // Biased InteractionForce
        iteration_based_logging[18 + j] = _tempFT.tempFT_human[j]; // Biased Human Force
        iteration_based_logging[24 + j] = _tempFT.tempFT_interactionU[j]; // Unbiased Interaction Force
        iteration_based_logging[30 + j] = _tempFT.tempFT_humanU[j]; // Unbiased Human Force
        iteration_based_logging[36 + j] = temp_targetJointPoss[j];
        iteration_based_logging[42 + j] = jointPos[j];
        iteration_based_logging[48 + j] = temp_targetJointVels[j];
        iteration_based_logging[54 + j] = temp_actualJointVels[j];
        iteration_based_logging[60 + j] = temp_targetJointAccs[j];
        iteration_based_logging[66 + j] = tcpPos[j];
        iteration_based_logging[72 + j] = actualSpeedVector[j];
        iteration_based_logging[78 + j] = temp_targetJointTorques[j];
        iteration_based_logging[84 + j] = temp_targetJointCurrents[j];
        iteration_based_logging[90 + j] = temp_actualJointCurrents[j];
        iteration_based_logging[96 + j] = temp_generalizedTCPforces[j];
        iteration_based_logging[111 + j] = tempjointMode[j];
    }
    for (int j = 0; j < 3; j++) {
        iteration_based_logging[102 + j] = temp_tcpAcc[j];
    }
    iteration_based_logging[105] = controllerTime[iterationNum];
    iteration_based_logging[106] = controllerRealTime[iterationNum];
    iteration_based_logging[107] = TimeReal[iterationNum];
    iteration_based_logging[108] = BytesTranferred[iterationNum];
    iteration_based_logging[109] = iterationNum;
    iteration_based_logging[110] = robotModeArray[iterationNum];
    for (int j = 0; j < 15; j++) {
        iteration_based_logging[117 + j] = tempUnused[j];
    }

    data_logs.push_back(iteration_based_logging);
    LeaveCriticalSection(&DataRecordingCS);
}

void fill_ftvalues_for_logging() {
    for (int i = 0; i < 6; i++) {
        averageSensorForcesR[iterationNum][i] = _tempFT.tempFT_interaction[i]; // Biased InteractionForce
        averageSensorForcesH[iterationNum][i] = _tempFT.tempFT_human[i]; // Biased Human Force
        averageSensorForcesRU[iterationNum][i] = _tempFT.tempFT_interactionU[i]; // Unbiased Interaction Force
        averageSensorForcesHU[iterationNum][i] = _tempFT.tempFT_humanU[i]; // Unbiased Human Force
    }
}

void admittance_controller() {
    if (!isControllerInitialized) {
        // Initialize Admittance Controller
        dcAC.InitializeParameters({ 50,50,60,0.50,0.50,0.50 }, { 90, 90, 125 ,5 ,5 ,5 }, 0.008, {1,1,1,1,1,1}, fulljointPos[iterationNum], ZOH_IOAC, { 1.0,1.0,1.0,1.0,1.0,1.0 }, 4.0, 200000UL);
        dcAC.doforcesNeedFilter = true;
        dcAC.filterorder = 1;
        isControllerInitialized = true;
    } else if (iterationNum > 1 && isControllerInitialized) {
        dcAC.ForceConditioner(_tempFT.tempFT_interaction);
        fill_ftvalues_for_logging();
        dcAC.RunController(fulljointPos[iterationNum]);
        sprintf(message, "speedl([%1.5f,%1.5f,%1.5f,%1.5f,%1.5f,%1.5f],%2.2f,%1.3f)\n", dcAC.spatial_velocity(0), dcAC.spatial_velocity(1), dcAC.spatial_velocity(2), dcAC.spatial_velocity(3), dcAC.spatial_velocity(4), dcAC.spatial_velocity(5), accInSpeed, timeToStayInSpeed);
        toUR5.sendMessageToServer(message);
    }
}

// Move robot to home position
void home_positioning() {
    if (calculate_joint_angle_error(initConf, fulljointPos[iterationNum]) > 0.0001 || calculate_instantaneous_speed(actualJointVels[iterationNum]) > 0.01) {
        printf(".");
    } else {
        printf("Done!\n");
        printf("> The Robot is initializing...");
        sprintf(message, "speedl([%1.5f,%1.5f,%1.5f,%1.5f,%1.5f,%1.5f],1.0,0.1)\n", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        toUR5.sendMessageToServer(message);
        printf("Done!\n");
        printf("> The Bias Voltages are cleaning...");
        ftsensors.biasIsCompleted = 0;
        printf("Done!\n");

        status = UserInitializingMode;
    }
}

// Initialize sensors
void sensor_initializing() {
    printf("> Sensors Data Acquisition Class is initializing....\n");
    ftsensors.initialization("Calibration/FT39161.cal", "Calibration/FT39162.cal");
    ftsensors.deviceNameAndPins = "Dev1/ai16:21,Dev1/ai0:5";
    ftsensors.force_sampling_frequency = 1000.0;
    ftsensors.run();
    printf("Done!\n");
}

// Read sensor and do admittance control
void read_handler(const boost::system::error_code& ec, std::size_t bytes_transferred) {
    if (!ec) {
        QueryPerformanceCounter(&startLoopTick);
        elapsedTime = (float)(startLoopTick.QuadPart - GlobalStartTick.QuadPart) * 1000.0 / sysFreq.QuadPart;
        TimeReal[iterationNum] = elapsedTime;
        BytesTranferred[iterationNum] = bytes_transferred;

        parseJointData();
        readFTvalues_Once();

        if (status == RobotInitializingMode) {
            home_positioning();
        } else if (status == UserInitializingMode) {
            openJointDataFile();
            int statusofDR = _beginthread(DataRecording, 0, NULL);
            std::cout << "> Data Recording Thread's State: " << statusofDR << std::endl;
            std::cout << std::strerror(errno) << '\n';
            status = AdmittanceControlMode;
            printf("\n\n*** YOUR TURN ! *** ..\n\n");
        } else if (status == AdmittanceControlMode) {
            admittance_controller();
        }

        fill_iteration_based_logging();
        
        if (iterationNum % BUFFERSIZE + 1 == BUFFERSIZE) {
            WakeConditionVariable(&AlarmforDataRecording);
        }

        QueryPerformanceCounter(&endLoopTick);
        loopTimes[iterationNum] = (endLoopTick.QuadPart - startLoopTick.QuadPart) * 1000000 / sysFreq.QuadPart;
        MYuSleep(7500 - (int)loopTimes[iterationNum]);

        if (iterationNum++ > endRun || (GetAsyncKeyState(VK_ESCAPE))) {
            exitThread = 1;
            endProcessThread = 1;
            QueryPerformanceCounter(&endLoopTick);
            double finalElapsedTime = (endLoopTick.QuadPart - startTick.QuadPart) * 1000000.0 / sysFreq.QuadPart;
            endingOperations();
            consumeLeftovers = true;
            WakeConditionVariable(&AlarmforDataRecording);
            exit(EXIT_FAILURE);
        } else {
            sock.async_read_some(boost::asio::buffer(buffer, 812), read_handler);
        }
    } else {
        printf("reading Failed...");
        endingOperations();
    }
}

// Call read_handler
void connect_handler(const boost::system::error_code& ec) {
    if (!ec) {
        sock.async_read_some(boost::asio::buffer(buffer), read_handler);
    }
}

// Call connect_handler
void resolve_handler(const boost::system::error_code& ec, boost::asio::ip::tcp::resolver::iterator it) {
    if (!ec) {
        sock.async_connect(*it, connect_handler);
    }
}

// Initialize global variables
void initializeGlobalVariables(char* IP = "192.168.1.3", char* port = "30002") {
    toUR5.connectionSettings(IP, port); // Robot IP and port number

    FTR[2] = 0; // Initialize force readings
    FTH[2] = 0;
    FHX[0] = FHY[0] = FHZ[0] = 0.0;
    THX[0] = THY[0] = THZ[0] = 0.0;

    FTRU[2] = 0; // Initialize unbiased force readings
    FTHU[2] = 0;

    consumeLeftovers = false;
    InitializeConditionVariable(&AlarmforDataRecording);
    InitializeCriticalSection(&DataRecordingCS);
}

int main(int argc, char* argv[]) {
    char* IP = "192.168.1.3";
    char* sending_port = "30002";
    char* receiving_port = "30003";

    initializeGlobalVariables(IP, sending_port);
    waitFor(3);
    toUR5.connectToServer();
    waitFor(3);
    sensor_initializing();
    waitFor(3);

    status = RobotInitializingMode;

    // Initialize rectangle vertices in the XY plane
    rect_vertices = {
        {fulltcpPos[0][0], fulltcpPos[0][1], fulltcpPos[0][2]},
        {0.1, 0.1, 0},
        {0.5, 0.1, 0},
        {0.5, 0.3, 0},
        {0.1, 0.5, 0},
        {0.1, 0.1, 0}
    };

    // Initialize current target
    current_target = rect_vertices[current_vertex];

    boost::asio::ip::tcp::resolver::query query(IP, receiving_port);
    resolver.async_resolve(query, resolve_handler);

    QueryPerformanceFrequency(&sysFreq);
    QueryPerformanceCounter(&startTick);
    QueryPerformanceCounter(&GlobalStartTick);
    printf("> Going to the co-manipulation position...");
    toUR5.sendMessageToServer(initConf);
    io_service.run();

    while (iterNum < endRun && !ESC) {
        moveRect();
        iterNum++;
    }
    
    return 0;
}
