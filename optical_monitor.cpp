// Optical Link Monitor - single file, terminal based, Linux
// Build : g++ -std=c++17 -Wall -Wextra -pthread optical_monitor.cpp -o monitor
// Run   : ./monitor --duration 10 --fault-rate 0.2
//         ./monitor --sensor sys --duration 5     (real CPU temp from /sys)
//         ./monitor --test                        (built-in self tests)

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>

// ======================= data types =======================

// One snapshot of the device. NaN = value not provided by this sensor.
struct Reading {
    double power_dbm = NAN;   // optical power
    double temp_c    = NAN;   // temperature
    double bias_ma   = NAN;   // laser bias current
    std::string source;       // "sim" or "sys"
};

struct Thresholds {
    double min_power_dbm = -25.0;
    double max_temp_c    = 75.0;
    double max_bias_ma   = 120.0;
};

// ======================= sensors (OOP) =======================

// Abstract interface: the monitor only knows Sensor, so real hardware
// (I2C/SPI driver) can be added later as another subclass.
class Sensor {
public:
    virtual ~Sensor() = default;
    virtual Reading read() = 0;
};

// Simulated optical module with random fault injection.
class SimSensor : public Sensor {
public:
    SimSensor(double fault_rate, unsigned seed)
        : fault_rate_(fault_rate), rng_(seed) {}

    Reading read() override {
        Reading r;
        r.source    = "sim";
        r.power_dbm = -10.0 + 0.3 * noise_(rng_);   // ~ -10 dBm
        r.temp_c    = 40.0  + 1.0 * noise_(rng_);   // ~ 40 C
        r.bias_ma   = 80.0  + 2.0 * noise_(rng_);   // ~ 80 mA

        if (chance_(rng_) < fault_rate_) {          // inject one fault
            switch (pick_(rng_)) {
                case 0: r.power_dbm = -30.0; break; // signal loss
                case 1: r.temp_c    = 85.0;  break; // overheating
                case 2: r.bias_ma   = 130.0; break; // laser aging
            }
        }
        return r;
    }

private:
    double fault_rate_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_{0.0, 1.0};
    std::uniform_real_distribution<double> chance_{0.0, 1.0};
    std::uniform_int_distribution<int> pick_{0, 2};
};

// Reads real CPU temperature that Linux exposes through /sys.
class SysSensor : public Sensor {
public:
    Reading read() override {
        Reading r;
        r.source = "sys";
        std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
        long milli = 0;
        if (f >> milli) r.temp_c = milli / 1000.0;  // millidegrees -> degrees
        return r;
    }
};

// ======================= thread-safe queue =======================

template <typename T>
class ThreadSafeQueue {
public:
    void push(T item) {
        {
            std::lock_guard<std::mutex> lock(m_);   // protect the queue
            q_.push(std::move(item));
        }
        cv_.notify_one();                            // wake the consumer
    }

    // Blocks until data arrives or the queue is closed.
    // Returns false only when closed AND empty.
    bool pop(T& out) {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [this] { return !q_.empty() || closed_; });
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop();
        return true;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(m_);
            closed_ = true;
        }
        cv_.notify_all();
    }

private:
    std::queue<T> q_;
    std::mutex m_;
    std::condition_variable cv_;
    bool closed_ = false;
};

// ======================= helper functions =======================

static std::atomic<bool> g_running{true};   // atomic: shared with signal handler

static void on_sigint(int) { g_running = false; }

static std::string now_str() {
    std::time_t t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof buf, "%H:%M:%S", std::localtime(&t));
    return buf;
}

// Pure function: which alarms does this reading trigger?
// NaN comparisons are always false, so missing values are skipped.
static std::string check_alarms(const Reading& r, const Thresholds& t,
                                std::string* value_out = nullptr) {
    std::ostringstream names, vals;
    if (r.power_dbm < t.min_power_dbm) { names << "LOW_POWER ";  vals << r.power_dbm << " "; }
    if (r.temp_c    > t.max_temp_c)    { names << "HIGH_TEMP ";  vals << r.temp_c    << " "; }
    if (r.bias_ma   > t.max_bias_ma)   { names << "HIGH_BIAS ";  vals << r.bias_ma   << " "; }
    if (value_out) *value_out = vals.str();
    return names.str();
}

// Producer thread: reads the sensor and pushes readings.
static void producer(Sensor& sensor, ThreadSafeQueue<Reading>& q) {
    while (g_running) {
        q.push(sensor.read());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    q.close();   // tell the consumer no more data is coming
}

struct Stats {
    int reads  = 0;
    int alarms = 0;
};

// Consumer thread: checks thresholds, prints alarms, writes the log.
static void monitor(ThreadSafeQueue<Reading>& q, const Thresholds& th,
                    const std::string& log_path, Stats& stats) {
    std::ofstream log(log_path);
    Reading r;
    while (q.pop(r)) {
        std::string ts = now_str();
        stats.reads++;
        log << "READ," << ts << "," << r.source << "," << r.power_dbm << ","
            << r.temp_c << "," << r.bias_ma << "\n";

        std::string vals;
        std::string names = check_alarms(r, th, &vals);
        if (!names.empty()) {
            stats.alarms++;
            names.pop_back();   // remove trailing space
            vals.pop_back();
            std::cout << "[ALARM] " << ts << " " << names << "(" << vals << ")\n";
            log << "ALARM," << ts << "," << names << "," << vals << "\n";
        }
    }
}

// Runs the whole pipeline for N seconds and returns the statistics.
static Stats run_monitor(Sensor& sensor, const Thresholds& th,
                         const std::string& log_path, int duration_s) {
    Stats stats;
    ThreadSafeQueue<Reading> queue;
    g_running = true;

    std::thread prod(producer, std::ref(sensor), std::ref(queue));
    std::thread mon(monitor, std::ref(queue), std::cref(th), log_path, std::ref(stats));

    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (duration_s > 0 &&
            std::chrono::steady_clock::now() - start >= std::chrono::seconds(duration_s))
            g_running = false;
    }
    prod.join();
    mon.join();
    return stats;
}

// ======================= built-in self tests =======================

static int g_failures = 0;

static void expect(bool ok, const std::string& name) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << "\n";
    if (!ok) g_failures++;
}

static int run_self_tests() {
    Thresholds th;

    // check_alarms unit tests
    Reading good; good.power_dbm = -10; good.temp_c = 40; good.bias_ma = 80;
    expect(check_alarms(good, th).empty(), "normal reading -> no alarm");

    Reading low = good; low.power_dbm = -30;
    expect(check_alarms(low, th) == "LOW_POWER ", "low power -> LOW_POWER");

    Reading hot = good; hot.temp_c = 85;
    expect(check_alarms(hot, th) == "HIGH_TEMP ", "high temp -> HIGH_TEMP");

    Reading nan_only;  // sys sensor style: only temp, rest NaN
    nan_only.temp_c = 50;
    expect(check_alarms(nan_only, th).empty(), "NaN fields are skipped");

    // full pipeline tests (2 seconds each)
    SimSensor clean(0.0, 1);
    Stats s1 = run_monitor(clean, th, "test_clean.log", 2);
    expect(s1.reads > 10 && s1.alarms == 0, "no faults -> reads logged, 0 alarms");

    SimSensor faulty(1.0, 1);
    Stats s2 = run_monitor(faulty, th, "test_fault.log", 2);
    expect(s2.alarms > 0 && s2.alarms == s2.reads, "all faults -> alarm for every reading");

    std::cout << (g_failures ? "\nSome tests FAILED\n" : "\nAll tests passed\n");
    return g_failures ? 1 : 0;
}

// ======================= main =======================

static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " [--sensor sim|sys] [--fault-rate x] [--duration s]"
                 " [--log file] [--seed n] [--test]\n";
}

int main(int argc, char** argv) {
    std::string sensor_type = "sim";
    std::string log_path    = "monitor.log";
    double fault_rate = 0.05;
    int duration = 0;          // 0 = run until Ctrl+C
    unsigned seed = 42;
    bool test_mode = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if      (a == "--test") test_mode = true;
        else if (a == "--sensor"     && i + 1 < argc) sensor_type = argv[++i];
        else if (a == "--log"        && i + 1 < argc) log_path    = argv[++i];
        else if (a == "--fault-rate" && i + 1 < argc) fault_rate  = std::atof(argv[++i]);
        else if (a == "--duration"   && i + 1 < argc) duration    = std::atoi(argv[++i]);
        else if (a == "--seed"       && i + 1 < argc) seed        = std::atoi(argv[++i]);
        else { print_usage(argv[0]); return 1; }
    }

    std::signal(SIGINT, on_sigint);   // Ctrl+C -> clean shutdown

    if (test_mode) return run_self_tests();

    // polymorphism: choose implementation at runtime, use via base class
    std::unique_ptr<Sensor> sensor;
    if (sensor_type == "sys") sensor = std::make_unique<SysSensor>();
    else                      sensor = std::make_unique<SimSensor>(fault_rate, seed);

    Thresholds th;
    std::cout << "Optical link monitor started (" << sensor_type
              << " sensor). Ctrl+C to stop.\n";

    Stats s = run_monitor(*sensor, th, log_path, duration);

    std::cout << "Stopped. " << s.reads << " readings, " << s.alarms
              << " alarms. Log: " << log_path << "\n";
    return 0;
}
