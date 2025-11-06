// sim.cpp
// Author: Kaique Fernandes
// Version: 0.1
// Build: g++ -std=c++17 sim.cpp -o sim
// Usage: ./sim 
// scenario: 1 = nominal, 2 = IMUs drop out sequentially, 3 = GNSS dropout 500ms
//
// Writes data to sim_<scenario-name>.txt
// Writes warning and faults to warn_<scenario-name>.txt

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
#include <filesystem>

using namespace std::chrono; 
namespace fs = std::filesystem;

std::atomic<bool> running{true};

// ------------- Thread-safe queue --------------
template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& value){
        std::lock_guard<std::mutex> lock(mtx_);
        q_.push(value);
        cv_.notify_one();
    }

    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (q_.empty()) return std::nullopt;
        T val = q_.front(); q_.pop();
        return val;
    }

    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]{ return !q_.empty() || !running.load(); });
        if (q_.empty())
            return std::nullopt;
        T val = q_.front();
        q_.pop();
        return val;
    } 

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        std::queue<T> empty;
        std::swap(q_, empty);
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
// ------------- FDIR --------------
class FDIR {
public:
    FDIR(std::ofstream& warn_file)
        : warn_stream_(warn_file) {}

    // Called for IMU and GNSS data (valid or not)
    void notify_imu(size_t id, double time) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (id >= last_imu_time_.size()) last_imu_time_.resize(id + 1, -1.0);
        last_imu_time_[id] = time;
    }

    void notify_gnss(size_t id, double time) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (id >= last_gnss_time_.size()) last_gnss_time_.resize(id + 1, -1.0);
        last_gnss_time_[id] = time;
    }

    void check(double current_time, bool imu_valid, bool gnss_valid) {
        std::lock_guard<std::mutex> lock(mtx_);

        // constants for requirement -> 3x no sensor signal
        const double IMU_TIMEOUT = 0.03;   // 3 × 10ms (100 Hz) 
        const double GNSS_TIMEOUT = 0.15;  // 3 × 50ms (20 Hz)

        // --- Check no valid input ---
        if (!imu_valid || !gnss_valid) {
            warn_stream_ << "[FDIR] Missing valid data at t=" << current_time << "\n";
        }

        // --- Check stale GNSS ---
        for (size_t i = 0; i < last_gnss_time_.size(); ++i) {
            double age = current_time - last_gnss_time_[i];
            if (last_gnss_time_[i] >= 0 && age > GNSS_TIMEOUT) {
                warn_stream_ << "[FDIR] GNSS[" << i << "] no output for "
                             << age << "s at t=" << current_time << "\n";
            }
            if (last_gnss_time_[i] >= 0 && age > 1.0) {
                warn_stream_ << "[FDIR] GNSS[" << i << "] stale (age="
                             << age << "s) at t=" << current_time << "\n";
            }
        }

        // --- Check stale IMU ---
        for (size_t i = 0; i < last_imu_time_.size(); ++i) {
            double age = current_time - last_imu_time_[i];
            if (last_imu_time_[i] >= 0 && age > IMU_TIMEOUT) {
                warn_stream_ << "[FDIR] IMU[" << i << "] no output for "
                             << age << "s at t=" << current_time << "\n";
            }
            if (last_imu_time_[i] >= 0 && age > 1.0) {
                warn_stream_ << "[FDIR] IMU[" << i << "] stale (age="
                             << age << "s) at t=" << current_time << "\n";
            }
        }

        warn_stream_.flush();
    }

private:
    std::vector<double> last_imu_time_;
    std::vector<double> last_gnss_time_;
    std::ofstream& warn_stream_;
    std::mutex mtx_;  // makes it safe if called from multiple threads
};
// ------------- Sensors definition --------------
class SensorIMU {
public:
    SensorIMU(ThreadSafeQueue<IMUData>& queue, 
        FDIR& fdir, 
        size_t sensor_id, 
        const std::chrono::steady_clock::time_point start_time,
        double rate_hz = 100.0)
        : queue_(queue), fdir_(fdir), sensor_id_(sensor_id), start_(start_time), 
        dt_(1.0 / rate_hz), enabled_(true)  {};

    void disable() { 
        enabled_ = false; 
        queue_.clear();  // flush queue so DataProcessor sees no data
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    void set_enabled(bool e) { enabled_ = e; }

    void run() {
        // random noise generator
        std::mt19937 rdn {42};
        std::normal_distribution<double> noise(0.0, 0.1);

        while (running) {  

            if (!enabled_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            auto now = std::chrono::steady_clock::now();
            double time = std::chrono::duration<double>(now - start_).count();

            IMUData data;
            data.time = time;

            data.rate_x = 0.01 * std::sin(2 * M_PI * 0.2 * time) + noise(rdn);
            data.rate_y = 0.01 * std::cos(2 * M_PI * 0.15 * time) + noise(rdn);
            data.rate_z = 0.01 * std::sin(2 * M_PI * 0.1 * time) + noise(rdn);
            queue_.push(data);

            fdir_.notify_imu(sensor_id_, data.time);

            std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
        };
    };

private:
    std::atomic<bool> enabled_;
    FDIR& fdir_;
    size_t sensor_id_;
    ThreadSafeQueue<IMUData>&  queue_;
    double dt_;
    const std::chrono::steady_clock::time_point start_;

};

class SensorGNSS {
public:
    SensorGNSS(ThreadSafeQueue<GNSSData>& queue, 
        FDIR& fdir, 
        size_t sensor_id, 
        const std::chrono::steady_clock::time_point start_time,
        double rate_hz = 20.0) 
            : queue_(queue), fdir_(fdir), sensor_id_(sensor_id), start_(start_time), 
            dt_(1.0 / rate_hz), enabled_(true) {}

    void disable() { 
        enabled_ = false; 
        queue_.clear();  // flush queue so DataProcessor sees no data
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    void set_enabled(bool e) { enabled_ = e; }

    void run() {
        // random noise generator
        std::mt19937 rdn {37};
        std::normal_distribution<double> noise(0.0, 0.1);
               
        while (running) {

            if (!enabled_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            auto now = std::chrono::steady_clock::now();
            double time = std::chrono::duration<double>(now - start_).count();

            GNSSData data;
            data.time = time;
            // generate slowly changing position (e.g., linear + small sinusoid) -> review values
            data.pos_x = 7000.0 + 0.1 * time + 2.0 * std::sin(2.0 * M_PI * 0.01 * time) + noise(rdn);
            data.pos_y = -1200.0 + 0.05 * time + 1.2 * std::cos(2.0 * M_PI * 0.012 * time) + noise(rdn);
            data.pos_z = 10.0 + 0.01 * std::sin(2.0 * M_PI * 0.02 * time) + noise(rdn);
            queue_.push(data);
            
            fdir_.notify_gnss(sensor_id_, data.time);

            std::this_thread::sleep_for(std::chrono::duration<double>(dt_)); // 20 Hz
        };
    };

private:
    std::atomic<bool> enabled_;
    FDIR& fdir_;
    size_t sensor_id_;
    ThreadSafeQueue<GNSSData>& queue_;
    const std::chrono::steady_clock::time_point start_; 
    double dt_;
};
// ------------- Processing --------------
struct ProcessorOutput {
    double time;
    // bool attitude_valid;
    double att_x, att_y, att_z; // averaged rates
    // bool position_valid;
    double pos_x, pos_y, pos_z;
};

class DataProcessor {
public:
    DataProcessor(std::ofstream& data_os, 
            std::ofstream& warn_os,
            FDIR& fdir,
            const std::chrono::steady_clock::time_point& start_time) 
        : log_stream_(data_os), warn_stream_(warn_os),  fdir_(fdir), global_start_(start_time) {}

    // Last-known valid samples
    std::optional<IMUData> last_imu[3];
    std::optional<GNSSData> last_gnss[2];
    double last_gnss_time_[2] = {0.0, 0.0};
    bool imu_input_validity[3] = {false, false, false};
    bool gnss_input_validity[2] = {false, false};

    void run(){

        while (running) {

            double t_now = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - global_start_).count();

            // Pop data from all queues and check freshness and data INPUT validity
            auto imu0_new = imuQueue0.try_pop(); 
            if (imu0_new) last_imu[0] = imu0_new; 
            imu_input_validity[0] = imu0_new ? true : false; 

            auto imu1_new = imuQueue1.try_pop(); 
            if (imu1_new) last_imu[1] = imu1_new; 
            imu_input_validity[1] = imu1_new ? true : false; 

            auto imu2_new = imuQueue2.try_pop(); 
            if (imu2_new) last_imu[2] = imu2_new; 
            imu_input_validity[2] = imu2_new ? true : false; 

            auto gnss0_new = gnssQueue0.try_pop(); 
            if (gnss0_new) last_gnss[0] = gnss0_new; 
            gnss_input_validity[0] = gnss0_new ? true : false; 

            auto gnss1_new = gnssQueue1.try_pop(); 
            if (gnss1_new) last_gnss[1] = gnss1_new; 
            gnss_input_validity[1] = gnss1_new ? true : false;
            
            // Validate general presence
            bool imu_valid = imu0_new.has_value() || imu1_new.has_value() || imu2_new.has_value();
            bool gnss_valid = gnss0_new.has_value() || gnss1_new.has_value();

            double t = t_now;

            valid_input_check(t_now, imu_valid, gnss_valid);

            std::optional<IMUData> imu0 = last_imu[0];
            std::optional<IMUData> imu1 = last_imu[1];
            std::optional<IMUData> imu2 = last_imu[2];
            std::optional<GNSSData> gnss0 = last_gnss[0];
            std::optional<GNSSData> gnss1 = last_gnss[1];

            // If simulation is stopping and there is no data anywhere, break
            if (!running.load() &&
                !imu0.has_value() && !imu1.has_value() && !imu2.has_value() &&
                !gnss0.has_value() && !gnss1.has_value())
            {
                break;
            }

            // Prepare Acumulattors
            double sum_att_x = 0.0, sum_att_y = 0.0, sum_att_z = 0.0;
            double sum_pos_x = 0.0, sum_pos_y = 0.0, sum_pos_z = 0.0;
            int imu_count = 0, gnss_count = 0;

            // module processing
            // --- IMU averaging ---
            imu_average(imu_input_validity[0], imu0, sum_att_x, sum_att_y, sum_att_z, imu_count, t);
            imu_average(imu_input_validity[1], imu1, sum_att_x, sum_att_y, sum_att_z, imu_count, t);
            imu_average(imu_input_validity[2], imu2, sum_att_x, sum_att_y, sum_att_z, imu_count, t);

            // --- GNSS averaging ---
            gnss_average(gnss_input_validity[0], gnss0, sum_pos_x, sum_pos_y, sum_pos_z, gnss_count, 0, t);
            gnss_average(gnss_input_validity[1], gnss1, sum_pos_x, sum_pos_y, sum_pos_z, gnss_count, 1, t);            
            
            // check data
            //imu_signal_check(t, imu0, "0");
            //imu_signal_check(t, imu1, "1");
            //imu_signal_check(t, imu2, "2");

            //gnss_signal_check(t, gnss0, "0", last_gnss_time_[0]);
            //gnss_signal_check(t, gnss1, "1", last_gnss_time_[1]);

            // outputting log files per module
            log_imu_data(imu_input_validity[0] ,imu0, "0", t);
            log_imu_data(imu_input_validity[1] ,imu1, "1", t);
            log_imu_data(imu_input_validity[2] ,imu2, "2", t);
            log_gnss_data(gnss_input_validity[0], gnss0, "0", t);
            log_gnss_data(gnss_input_validity[1], gnss1, "1", t);

            // Compute averages or set NaN
            double att_x = imu_count ? sum_att_x / imu_count : std::nan("");
            double att_y = imu_count ? sum_att_y / imu_count : std::nan("");
            double att_z = imu_count ? sum_att_z / imu_count : std::nan("");

            double pos_x = gnss_count ? sum_pos_x / gnss_count : std::nan("");
            double pos_y = gnss_count ? sum_pos_y / gnss_count : std::nan("");
            double pos_z = gnss_count ? sum_pos_z / gnss_count : std::nan("");

            bool imu_ok = (imu_count > 0);
            bool gnss_ok = (gnss_count > 0);

            fdir_.check(t, imu_ok, gnss_ok);

            // Log the averaged line 
            ProcessorOutput out_line{t, att_x, att_y, att_z, pos_x, pos_y, pos_z};
            log_line(out_line);

            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50 Hz
        };
    };

private:
    const double IMU_STALENESS_THRESHOLD_ = 0.1;
    const double GNSS_STALENESS_THRESHOLD_ = 1.0; // top margin
    const std::chrono::steady_clock::time_point& global_start_;
    FDIR& fdir_;
    std::ofstream &log_stream_;  
    std::ofstream &warn_stream_;

    // signal check
    void valid_input_check(double& t, bool& valid_imu_input, bool& valid_gnss_input) {
        if (!valid_imu_input && !valid_gnss_input) {
            warn_stream_.setf(std::ios::fixed);
            warn_stream_.precision(6);
            warn_stream_ << "[WARNING] No valid input";
            warn_stream_ << " at t=" << t << "\n";
            warn_stream_.flush();
        }
    }
    void gnss_signal_check(double t, const std::optional<GNSSData>& gnss, 
                            const std::string& sensor_id, 
                            double& last_gnss_time) {

        bool missing_gnss = !gnss;
        bool gnss_old = (t - last_gnss_time) > 1.0;

        if (missing_gnss) {
            warn_stream_.setf(std::ios::fixed);
            warn_stream_.precision(6);
            warn_stream_ << "[WARNING]";
            if (missing_gnss) warn_stream_ << "Missing GNSS_" << sensor_id;
            if (gnss_old) warn_stream_ << " GNSS_" << sensor_id << " older than 1 sec";
            warn_stream_ << " at t=" << t << "\n";
            warn_stream_.flush();
        }

    }
    void imu_signal_check(double t, const std::optional<IMUData>& imu, const std::string& sensor_id) {
        bool missing_imu = !imu;
        //if (!imu) missing_imu = true;

        if (missing_imu) {
            warn_stream_.setf(std::ios::fixed);
            warn_stream_.precision(6);
            warn_stream_ << "[WARNING]";
            warn_stream_ << "Missing IMU_" << sensor_id << " data for time " << t << "\n";
        }        
    }

    // log helper functions
    void log_line(const ProcessorOutput &o) {
        log_stream_.setf(std::ios::fixed);
        log_stream_.precision(6);
        log_stream_ << "=============== AVERAGE ===============\n";
        log_stream_ << "sim_time, rate_x, rate_y, rate_z, pos_x, pos_y, pos_z\n";
        log_stream_ << o.time << ','
                    << o.att_x << ',' << o.att_y << ',' << o.att_z << ','
                    << o.pos_x << ',' << o.pos_y << ',' << o.pos_z << '\n';
        log_stream_ << "=======================================\n";
        log_stream_.flush(); 
    }

    void log_gnss_data(bool& data_condition, const std::optional<GNSSData>& gnss, const std::string& sensor_id, double t) {
        if (data_condition) {
            log_stream_.setf(std::ios::fixed);
            log_stream_.precision(6);
            log_stream_ << "GNSS_RAW_" << sensor_id << ","
                        << t << ","
                        << gnss->pos_x << ","
                        << gnss->pos_y << ","
                        << gnss->pos_z << "\n";
        } else {
            log_stream_ << "GNSS_RAW_" << sensor_id << ","
                        << t << ","
                        << std::nan("") << "," << std::nan("") << "," << std::nan("") << "\n";
        }        
    }

    void log_imu_data(bool& data_condition, const std::optional<IMUData>& imu, const std::string& sensor_id, double t) {
        if (data_condition) {
            log_stream_.setf(std::ios::fixed);
            log_stream_.precision(6);
            log_stream_ << "IMU_RAW_" << sensor_id << ","
                        << t << ","
                        << imu->rate_x << ","
                        << imu->rate_y << ","
                        << imu->rate_z << "\n";
        } else {
        log_stream_ << "IMU_RAW_" << sensor_id << ","
                    << t << ","
                    << std::nan("") << "," << std::nan("") << "," << std::nan("") << "\n";
        }
    }

    // --- Helper functions ---
    // double last_gnss_time_[2] = {0.0, 0.0};
    void imu_average(bool& data_condition, const std::optional<IMUData>& imu,
                     double& sum_x, double& sum_y, double& sum_z,
                     int& count, double& t_now) {

        if (data_condition) {
            sum_x += imu->rate_x;
            sum_y += imu->rate_y;
            sum_z += imu->rate_z;
            count++;
            // t = imu->time; // take last valid time
        }
    }

    void gnss_average(bool& data_condition, const std::optional<GNSSData>& gnss,
                      double& sum_x, double& sum_y, double& sum_z,
                      int& count, int index, double& t_now) {
        if (data_condition) {
            sum_x += gnss->pos_x;
            sum_y += gnss->pos_y;
            sum_z += gnss->pos_z;
            count++;

            last_gnss_time_[index] = gnss->time;
            // if (t == 0.0) t = gnss->time;
        }
    }
};

// ------------- Scenario Set --------------
void run_scenario(const std::string& name,
                  std::function<void(double, 
                  SensorIMU&, SensorIMU&, SensorIMU&, SensorGNSS&, SensorGNSS&)> failure_injector) {

    std::cout << "Starting scenario: " << name << std::endl;

    // Create logs
    std::string scenario_name = name;
    fs::path output_dir = fs::path("output") / scenario_name;
    fs::create_directories(output_dir);

    std::ofstream data_log(output_dir / "data_log.txt");
    std::ofstream warn_log(output_dir / "warn_log.txt");

    auto global_start = std::chrono::steady_clock::now();
    double sim_duration = 10.0;   // seconds
    FDIR fdir(warn_log);

    // data
    SensorIMU imu0(imuQueue0, fdir, 0, global_start, 100.0);
    SensorIMU imu1(imuQueue1, fdir, 1, global_start, 100.0);
    SensorIMU imu2(imuQueue2, fdir, 2, global_start, 100.0);
    SensorGNSS gnss0(gnssQueue0, fdir, 0, global_start, 20.0);
    SensorGNSS gnss1(gnssQueue1, fdir, 1, global_start, 20.0);
    DataProcessor proc(data_log, warn_log, fdir, global_start);

    // threads
    running = true;
    std::thread proc_thread(&DataProcessor::run, &proc);

    std::thread imu0_thread(&SensorIMU::run, &imu0);
    std::thread imu1_thread(&SensorIMU::run, &imu1);
    std::thread imu2_thread(&SensorIMU::run, &imu2);
    std::thread gnss0_thread(&SensorGNSS::run, &gnss0);
    std::thread gnss1_thread(&SensorGNSS::run, &gnss1);

    // Fault injectoin
    // auto start = std::chrono::steady_clock::now();
    while (true) {
        double t_now = std::chrono::duration<double>(std::chrono::steady_clock::now() - global_start).count();
        if (t_now > sim_duration) break;

        failure_injector(t_now, imu0, imu1, imu2, gnss0, gnss1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // check every 10ms
    }

    // join threads
    running = false;
    if (imu0_thread.joinable()) imu0_thread.join();
    if (imu1_thread.joinable()) imu1_thread.join();
    if (imu2_thread.joinable()) imu2_thread.join();
    if (gnss0_thread.joinable()) gnss0_thread.join();
    if (gnss1_thread.joinable()) gnss1_thread.join();
    if (proc_thread.joinable()) proc_thread.join();

    data_log.close();
    warn_log.close();
    std::cout << "Simulation done. Wrote warning.txt and log.txt\n"; 
}

// ------------- Run Simulation --------------

int main() {
    int scenario = 0;

    std::cout << "Please insert a scenario from 1 to 3.\n"; 
    std::cout << "1 - Nominal for 10s;\n"
              << "2 - IMU Dropout (in t = 3, 5 and 7s);\n" 
              << "3 - GNSS dropout (between t=4.0 and t=4.5s).\n"; 
    std::cin >> scenario;
   
    // SCENARIO 1: Nominal
    if (scenario == 1) {
        run_scenario("nominal", [](double t, auto&, auto&, auto&, auto&, auto&) {
        });
    } else if (scenario == 2) {
        // SCENARIO 2: IMU dropouts
        run_scenario("imu_dropout", [](double t, auto& imu0, auto& imu1, auto& imu2, auto&, auto&) {
            if (t > 3.0) imu0.disable();
            if (t > 5.0) imu1.disable();
            if (t > 7.0) imu2.disable();
        });
    } else if (scenario == 3) {
        //SCENARIO 3: GNSS dropout
        run_scenario("gnss_dropout", [](double t, auto&, auto&, auto&, auto& gnss0, auto& gnss1) {
            bool gnss_fail = (t > 4.0 && t < 4.5);
            gnss0.set_enabled(!gnss_fail);
            gnss1.set_enabled(!gnss_fail);
        });
    }   else {

        std::cout << "Please insert a scenario from 1 to 3.\n";

    }

    return 0;
};
