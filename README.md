# Multi-Sensor Fault Detection, Isolation, and Recovery (FDIR) Simulation

**Author:** Kaique Fernandes  
**Version:** 0.1  
**Build:** `g++ -std=c++17 sim.cpp -o sim`  
**Tested on:** Ubuntu 22.04 (Linux, g++ 11.4)  
**Simulation duration:** 10 seconds per run  

---

## Overview

This simulation models a **multi-sensor navigation system** consisting of:
- 3 × **IMU sensors** (100 Hz)
- 2 × **GNSS sensors** (20 Hz)
- 1 × **Processing module**
- 1 × **FDIR module** (Fault Detection, Isolation, and Recovery)

Each sensor runs in a **dedicated thread**, producing synthetic noisy data.  
A central **Processing** thread consumes the data from all sensors, computes averages, detects missing or stale data, and logs results.  
The **FDIR** module monitors sensor health and logs alarms whenever signals are missing, delayed, or invalid.

---

## System Architecture

Below is an ASCII overview of the software structure (you can replace this section with your rendered diagram):


---

## Design Decisions

### 1. **Thread-Safe Queues**
Each sensor pushes its output into a `ThreadSafeQueue<T>`.  
The processing thread pops data at 50 Hz to simulate real-time asynchronous behavior.  
This approach ensures **safe concurrent access** without blocking the main loop.

### 2. **NaN Handling for Missing Data**
When a sensor provides no new data (e.g., after a dropout), its corresponding averaged output is explicitly set to `NaN`.  
This design avoids reusing stale values and makes fault visualization clearer in the output logs.

### 3. **FDIR Integration**
The FDIR class operates independently and receives:
- Time notifications (`notify_imu()` / `notify_gnss()`)
- Validity checks via `check()`

It writes warnings when:
- Data is missing
- Data age exceeds 1 second
- Both IMU and GNSS are invalid simultaneously

### 4. **Scenario-Based Simulation**
The code supports **three distinct scenarios**, implemented in `run_scenario()` through a failure injector function.

| Scenario | Description | Fault Injection |
|-----------|--------------|----------------|
| 1 | **Nominal** | All sensors active for 10 s |
| 2 | **IMU dropouts** | IMU₀ fails at 3 s, IMU₁ at 5 s, IMU₂ at 7 s |
| 3 | **GNSS dropout** | Both GNSS units disabled between 4.0–4.5 s |

### 5. **Output Organization**
All results are automatically stored in:
output/<scenario_name>/
├── data_log.txt # Filtered data with NaN where missing
└── warn_log.txt # Fault and recovery messages


g++ -std=c++17 sim.cpp -o sim

project_root/
├── sim.cpp           # Main simulation source
├── README.md         # This documentation
└── output/
    ├── nominal/
    ├── imu_dropout/
    └── gnss_dropout/

