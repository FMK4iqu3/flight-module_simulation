// sim.cpp
// Author: Kaique Fernandes
// Version: 0.1
// Build: g++ -std=c++17 -O2 -pthread sim.cpp -o sim
// Usage: ./sim <scenario>
// scenario: 0 = nominal, 1 = IMUs drop out sequentially, 2 = GNSS dropout 500ms
//
// Writes CSV to sim_<scenario>_<start_ts>.csv

#include <iostream>
#include <thread>
#include <random>
#include <chrono>
#include <mutex>
#include <vector>
#include <queue>
#include <cmath>
#include <condition_variable>
#include <optional>
#include <fstream>
#include <atomic>

using namespace std::chrono; 
// using steady = std::chrono::steady_clock;
// using TimePoint = steady::time_point;

// ------------- Thread-safe queue --------------
template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& value){
        std::lock_guard<std::mutex> lock(mtx_);
        q_.push(value);
        cv_.notify_one();
    }

    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mtx_);
        if (q_.empty())
            return std::nullopt;
        T val = q_.front();
        q_.pop();
        return val;
    } 

private:
    std::queue<T> q_;
    std::mutex mtx_;
    std::condition_variable cv_;
};

// ------------- Data Structures --------------
struct IMUData {
    // TimePoint time;
    double time;
    double rate_x, rate_y, rate_z;
};

struct GNSSData {
    // TimePoint time;
    double time;
    double pos_x, pos_y, pos_z;
};

// ------------- Queues --------------
ThreadSafeQueue<IMUData> imuQueue;
ThreadSafeQueue<GNSSData> gnssQueue;
std::atomic<bool> running{true};

// ------------- Sensors definition --------------
void SensorIMU(double& time) {
    // random noise generator
    std::mt19937 rdn {42};
    std::normal_distribution<double> noise(0.0, 0.1);

    while (running) {   
        IMUData data;
        data.time = time;

        data.rate_x = 0.01 * std::sin(2 * M_PI * 0.2 * time) + noise(rdn);
        data.rate_y = 0.01 * std::cos(2 * M_PI * 0.15 * time) + noise(rdn);
        data.rate_z = 0.01 * std::sin(2 * M_PI * 0.1 * time) + noise(rdn);
        imuQueue.push(data);

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
    };
};

void SensorGNSS(double& time) {
    // random noise generator
    std::mt19937 rdn {42};
    std::normal_distribution<double> noise(0.0, 0.1);

    while (running) {
        GNSSData data;
        data.time = time;
        // generate slowly changing position (e.g., linear + small sinusoid) -> review values
        data.pos_x = 7000.0 + 0.1 * time + 2.0 * std::sin(2.0 * M_PI * 0.01 * time) + noise(rdn);
        data.pos_y = -1200.0 + 0.05 * time + 1.2 * std::cos(2.0 * M_PI * 0.012 * time) + noise(rdn);
        data.pos_z = 10.0 + 0.01 * std::sin(2.0 * M_PI * 0.02 * time) + noise(rdn);
        gnssQueue.push(data);

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
    };
};
// ------------- Processing --------------
struct ProcessingOutput {
    double time;
    // bool attitude_valid;
    double att_x, att_y, att_z; // averaged rates
    // bool position_valid;
    double pos_x, pos_y, pos_z;
};

class Processing {
public:
    Processing(): log_stream_(std::cout) {};
    
    void run(){
        while (running) {
            auto imuData = imuQueue.pop();
            auto gnssData = gnssQueue.pop();

            // prepare outputs with defaults
            double att_x = 0.0, att_y = 0.0, att_z = 0.0;
            double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;

            // simple placeholder processing; ignore empty optionals
            if (imuData) {
                att_x = imuData->rate_x;
                att_y = imuData->rate_y;
                att_z = imuData->rate_z;
            }

            if (gnssData) {
                pos_x = gnssData->pos_x;
                pos_y = gnssData->pos_y;
                pos_z = gnssData->pos_z;
            }
            
            double t = duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
            ProcessingOutput out_line{t, att_x, att_y, att_z, pos_x, pos_y, pos_z};

            log_line(out_line);

            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz

        };
    };

private:
    void log_line(const ProcessingOutput &o) {
        // Format txt line:
        // sim_time,att_x,att_y,att_z,pos_x,pos_y,pos_z
        std::ostringstream ss;
        ss.setf(std::ios::fixed); ss.precision(6);
        double t;
        ss << o.time << ',';
        ss << o.att_x << ',' << o.att_y << ',' << o.att_z << ',';
        ss << o.pos_x << ',' << o.pos_y << ',' << o.pos_z;
        log_stream_ << ss.str() << '\n';
    }

    std::ostream &log_stream_;

};
// ------------- FDIR --------------

// ------------- Run Simulation --------------

int main(int argc, char* argv[]) {
    int scenario = 0;
    if (argc > 1) scenario = std::atoi(argv[1]);

    const double sim_duration = 10.0;   // seconds
    //const double dt_imu = 0.01;         // 100 Hz
    //const double dt_gnss = 0.05;        // 20 Hz
    //const double dt_proc = 0.02;        // 50 Hz
    //bool running = true; 
    //int it_number = 0;

    // data
    std::vector<IMUData> imu_data;
    std::vector<GNSSData> gnss_data;

    // run processing loop
    std::ofstream log("simple_log.txt");
    log << "time,att_x,att_y,att_z,pos_x,pos_y,pos_z\n";

    // processing loop
    Processing proc;
    std::thread proc_thread(&Processing::run, &proc);

    double t = 0.0;
    auto start_time = std::chrono::steady_clock::now();

    double sim_time_imu = 0.0;
    double sim_time_gnss = 0.0;
    std::thread imu_thread(SensorIMU, std::ref(sim_time_imu));
    std::thread gnss_thread(SensorGNSS, std::ref(sim_time_gnss));

    // run simulation for the requested duration
    std::this_thread::sleep_for(std::chrono::duration<double>(sim_duration));
    running = false;

    // join threads
    if (imu_thread.joinable()) imu_thread.join();
    if (gnss_thread.joinable()) gnss_thread.join();
    if (proc_thread.joinable()) proc_thread.join();

    return 0;

};