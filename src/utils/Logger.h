#pragma once
#include <chrono>
#include <format>
#include <fstream>
#include <iostream>
#include <string_view>
#include <unordered_map>

const std::string LOG_FILE_NAME = "simulation.log";

class Profiler {
    struct Entry {
        double totalTime = 0;
        int count = 0;
    };
    std::unordered_map<std::string, Entry> entries;
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> starts;
    std::chrono::high_resolution_clock::time_point reportStart =
        std::chrono::high_resolution_clock::now();
    std::ofstream perfFile;

    Profiler() { perfFile.open("performance.log", std::ios::out | std::ios::trunc); }

public:
    ~Profiler() {
        if (perfFile.is_open())
            perfFile.close();
    }

    static Profiler &instance() {
        static Profiler p;
        return p;
    }

    // Records start time for a block
    void start(const std::string &name) {
        starts[name] = std::chrono::high_resolution_clock::now();
    }

    // Records end time and accumulates duration
    void end(const std::string &name) {
        auto end = std::chrono::high_resolution_clock::now();
        auto it = starts.find(name);
        if (it != starts.end()) {
            std::chrono::duration<double, std::milli> diff = end - it->second;
            entries[name].totalTime += diff.count();
            entries[name].count++;
        }
    }

    // Periodically outputs performance stats
    void report() {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - reportStart;
        if (elapsed.count() > 2.0) {
            if (perfFile.is_open()) {
                perfFile << "\n--- PERF REPORT (avg ms/frame) ---\\n";
                for (auto &[name, entry] : entries) {
                    if (entry.count > 0)
                        perfFile << std::format("{}: {:.3f} ms\n", name,
                                                entry.totalTime / entry.count);
                    entry.totalTime = 0;
                    entry.count = 0;
                }
                perfFile << "----------------------------------\n";
                perfFile.flush();
            }
            reportStart = now;
        }
    }

    // Returns average time for a block
    double getAverageTime(const std::string &name) {
        auto it = entries.find(name);
        if (it != entries.end()) {
            const Entry &e = it->second;
            if (e.count == 0)
                return 0.0;
            return e.totalTime / e.count;
        }
        return 0.0;
    }
};

class Logger {
private:
    std::ofstream logFile;

    Logger() {
        logFile.open(LOG_FILE_NAME, std::ios::out | std::ios::app);
        if (!logFile.is_open()) {
            std::cerr << "Error: Could not open log file: " << LOG_FILE_NAME << std::endl;
        } else {
            logFile << "\n========================================\n";
            logFile << "New session started at " << getTimestamp() << "\n";
            logFile << "========================================\n";
            logFile.flush();
        }
    }

    ~Logger() {
        if (logFile.is_open()) {
            logFile << "\n========================================\n";
            logFile << "Session ended at " << getTimestamp() << "\n";
            logFile << "========================================\n\n";
            logFile.flush();
            logFile.close();
        }
    }

    // Generates formatted timestamp
    std::string getTimestamp() const {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        char buf[20];
        std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&time));
        return std::format("{}.{:03d}", buf, (int)ms.count());
    }

public:
    static Logger &instance() {
        static Logger logger;
        return logger;
    }

    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    // Log informational message
    template <typename... Args> void info(std::format_string<Args...> fmt, Args &&...args) {
        std::string message = std::format("[{}] INFO: ", getTimestamp()) +
                              std::format(fmt, std::forward<Args>(args)...);
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }

    // Log debug message
    template <typename... Args> void debug(std::format_string<Args...> fmt, Args &&...args) {
        std::string message = std::format("[{}] DEBUG: ", getTimestamp()) +
                              std::format(fmt, std::forward<Args>(args)...);
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }

    // Log error message
    template <typename... Args> void error(std::format_string<Args...> fmt, Args &&...args) {
        std::string message = std::format("[{}] ERROR: ", getTimestamp()) +
                              std::format(fmt, std::forward<Args>(args)...);
        std::cerr << message << "\n";
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }

    // Log warning message
    template <typename... Args> void warn(std::format_string<Args...> fmt, Args &&...args) {
        std::string message = std::format("[{}] WARN: ", getTimestamp()) +
                              std::format(fmt, std::forward<Args>(args)...);
        std::cerr << message << "\n";
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }
};

#define APP_LOG_INFO(...) Logger::instance().info(__VA_ARGS__)
#define APP_LOG_DEBUG(...) Logger::instance().debug(__VA_ARGS__)
#define APP_LOG_ERROR(...) Logger::instance().error(__VA_ARGS__)
#define APP_LOG_WARN(...) Logger::instance().warn(__VA_ARGS__)

#define PROFILE_START(name) Profiler::instance().start(name)
#define PROFILE_END(name) Profiler::instance().end(name)
#define PROFILE_REPORT() Profiler::instance().report()