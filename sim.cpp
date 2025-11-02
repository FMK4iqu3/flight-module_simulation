// sim.cpp
// Author: Kaique Fernandes
// Version: 0.1
// Build: g++ -std=c++17 -O2 -pthread sim.cpp -o sim
// Usage: ./sim <scenario>
// scenario: 0 = nominal, 1 = IMUs drop out sequentially, 2 = GNSS dropout 500ms
//
// Writes TXT to sim_<scenario>_<start_ts>.txt

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
#include <limits>

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
        //cv_.wait(lock, [this]{ return !q_.empty() || !running; });
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
ThreadSafeQueue<IMUData> imuQueue0, imuQueue1, imuQueue2;
ThreadSafeQueue<GNSSData> gnssQueue0, gnssQueue1;
std::atomic<bool> running{true};

// ------------- Sensors definition --------------
void SensorIMU(double& time, ThreadSafeQueue<IMUData>& queue) {
    auto start = std::chrono::steady_clock::now();
    // random noise generator
    std::mt19937 rdn {42};
    std::normal_distribution<double> noise(0.0, 0.1);

    double dt_imu = 0.01;

    while (running) {   
        auto now = std::chrono::steady_clock::now();
        time = std::chrono::duration<double>(now - start).count();

        IMUData data;
        data.time = time;

        data.rate_x = 0.01 * std::sin(2 * M_PI * 0.2 * time) + noise(rdn);
        data.rate_y = 0.01 * std::cos(2 * M_PI * 0.15 * time) + noise(rdn);
        data.rate_z = 0.01 * std::sin(2 * M_PI * 0.1 * time) + noise(rdn);
        queue.push(data);

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
        // time += dt_imu;
    };
};

void SensorGNSS(double& time, ThreadSafeQueue<GNSSData>& queue) {
    auto start = std::chrono::steady_clock::now();
    // random noise generator
    std::mt19937 rdn {42};
    std::normal_distribution<double> noise(0.0, 0.1);

    double dt_gnss = 0.05;

    while (running) {
        auto now = std::chrono::steady_clock::now();
        time = std::chrono::duration<double>(now - start).count();

        GNSSData data;
        data.time = time;
        // generate slowly changing position (e.g., linear + small sinusoid) -> review values
        data.pos_x = 7000.0 + 0.1 * time + 2.0 * std::sin(2.0 * M_PI * 0.01 * time) + noise(rdn);
        data.pos_y = -1200.0 + 0.05 * time + 1.2 * std::cos(2.0 * M_PI * 0.012 * time) + noise(rdn);
        data.pos_z = 10.0 + 0.01 * std::sin(2.0 * M_PI * 0.02 * time) + noise(rdn);
        queue.push(data);

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
        // time += dt_gnss;
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
    Processing(std::ofstream& os) : log_stream_(os) {}
    
    void run(){
        while (running) {
            // Pop data from all queues
            auto imu0 = imuQueue0.pop();
            auto imu1 = imuQueue1.pop();
            auto imu2 = imuQueue2.pop();
            auto gnss0 = gnssQueue0.pop();
            auto gnss1 = gnssQueue1.pop();
            
            // Prepare accumulators
            double sum_att_x = 0.0, sum_att_y = 0.0, sum_att_z = 0.0;
            double sum_pos_x = 0.0, sum_pos_y = 0.0, sum_pos_z = 0.0;
            int imu_count = 0, gnss_count = 0;
            double t = 0.0;

            // module processing
            // outputting log files per module
            log_imu_data(imu0, "0");
            log_imu_data(imu1, "1");
            log_imu_data(imu2, "2");
            log_gnss_data(gnss0, "0");
            log_gnss_data(gnss1, "1");


            // --- IMU averaging ---
            imu_average(imu0, sum_att_x, sum_att_y, sum_att_z, imu_count, t);
            imu_average(imu1, sum_att_x, sum_att_y, sum_att_z, imu_count, t);
            imu_average(imu2, sum_att_x, sum_att_y, sum_att_z, imu_count, t);

            // --- GNSS averaging ---
            gnss_average(gnss0, sum_pos_x, sum_pos_y, sum_pos_z, gnss_count, t);
            gnss_average(gnss1, sum_pos_x, sum_pos_y, sum_pos_z, gnss_count, t);
            
            // Compute averages or set NaN
            double att_x = imu_count ? sum_att_x / imu_count : std::nan("");
            double att_y = imu_count ? sum_att_y / imu_count : std::nan("");
            double att_z = imu_count ? sum_att_z / imu_count : std::nan("");

            double pos_x = gnss_count ? sum_pos_x / gnss_count : std::nan("");
            double pos_y = gnss_count ? sum_pos_y / gnss_count : std::nan("");
            double pos_z = gnss_count ? sum_pos_z / gnss_count : std::nan("");

            ProcessingOutput out_line{t, att_x, att_y, att_z, pos_x, pos_y, pos_z};

            log_line(out_line);

            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz
        };
    };

private:
    // log helper functions
    void log_line(const ProcessingOutput &o) {
        log_stream_.setf(std::ios::fixed);
        log_stream_.precision(6);
        log_stream_ << "=============== AVERAGE ===============\n";
        log_stream_ << o.time << ','
                    << o.att_x << ',' << o.att_y << ',' << o.att_z << ','
                    << o.pos_x << ',' << o.pos_y << ',' << o.pos_z << '\n';
        log_stream_ << "=======================================\n";
        log_stream_.flush(); // make sure it's written immediately
    }

    std::ofstream &log_stream_;

    void log_gnss_data(const std::optional<GNSSData>& gnss, const std::string& sensor_id) {
        if (gnss) {
            log_stream_.setf(std::ios::fixed);
            log_stream_.precision(6);
            log_stream_ << "GNSS_RAW_" << sensor_id << ","
                        << gnss->time << ","
                        << gnss->pos_x << ","
                        << gnss->pos_y << ","
                        << gnss->pos_z << "\n";
        }
    }

    void log_imu_data(const std::optional<IMUData>& imu, const std::string& sensor_id) {
        if (imu) {
            log_stream_.setf(std::ios::fixed);
            log_stream_.precision(6);
            log_stream_ << "IMU_RAW_" << sensor_id << ","
                        << imu->time << ","
                        << imu->rate_x << ","
                        << imu->rate_y << ","
                        << imu->rate_z << "\n";
        }
    }

    // --- Helper functions ---
    void imu_average(const std::optional<IMUData>& imu,
                     double& sum_x, double& sum_y, double& sum_z,
                     int& count, double& t) {
        if (imu) {
            sum_x += imu->rate_x;
            sum_y += imu->rate_y;
            sum_z += imu->rate_z;
            count++;
            t = imu->time; // take last valid time
        }
    }

    void gnss_average(const std::optional<GNSSData>& gnss,
                      double& sum_x, double& sum_y, double& sum_z,
                      int& count, double& t) {
        if (gnss) {
            sum_x += gnss->pos_x;
            sum_y += gnss->pos_y;
            sum_z += gnss->pos_z;
            count++;
            if (t == 0.0) t = gnss->time;
        }
    }


};
// ------------- FDIR --------------

// ------------- Run Simulation --------------

int main(int argc, char* argv[]) {
    int scenario = 0;
    if (argc > 1) scenario = std::atoi(argv[1]);

    const double sim_duration = 10.0;   // seconds

    // data
    std::vector<IMUData> imu0_data;
    std::vector<IMUData> imu1_data;
    std::vector<IMUData> imu2_data;
    std::vector<GNSSData> gnss0_data;
    std::vector<GNSSData> gnss1_data;

    // run processing loop
    std::ofstream log("simple_log.txt");
    log << "time,att_x,att_y,att_z,pos_x,pos_y,pos_z\n";

    // processing loop
    Processing proc(log);
    std::thread proc_thread(&Processing::run, &proc);

    // do I need this?
    double t = 0.0;
    auto start_time = std::chrono::steady_clock::now();

    double sim_time_imu0 = 0.0, sim_time_imu1 = 0.0, sim_time_imu2 = 0.0;
    double sim_time_gnss0 = 0.0, sim_time_gnss1 = 0.0;

    std::thread imu0_thread(SensorIMU, std::ref(sim_time_imu0), std::ref(imuQueue0));
    std::thread imu1_thread(SensorIMU, std::ref(sim_time_imu1), std::ref(imuQueue1));
    std::thread imu2_thread(SensorIMU, std::ref(sim_time_imu2), std::ref(imuQueue2));
    std::thread gnss0_thread(SensorGNSS, std::ref(sim_time_gnss0), std::ref(gnssQueue0));
    std::thread gnss1_thread(SensorGNSS, std::ref(sim_time_gnss1), std::ref(gnssQueue1));

    // run simulation for the requested duration
    std::this_thread::sleep_for(std::chrono::duration<double>(sim_duration));
    running = false;

    // join threads
    if (imu0_thread.joinable()) imu0_thread.join();
    if (imu1_thread.joinable()) imu1_thread.join();
    if (imu2_thread.joinable()) imu2_thread.join();
    if (gnss0_thread.joinable()) gnss0_thread.join();
    if (gnss1_thread.joinable()) gnss1_thread.join();
    if (proc_thread.joinable()) proc_thread.join();

    log.close();
    std::cout << "Simulation done. Wrote simple_log.txt\n"; 
};