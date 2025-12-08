#pragma once
#include <iostream>
#include <fstream>
#include <format>
#include <string_view>
#include <chrono>
#include <unordered_map>

const std::string LOG_FILE_NAME = "simulation.log";

class Profiler {
    struct Entry {
        double totalTime = 0;
        int count = 0;
    };
    std::unordered_map<std::string, Entry> entries;
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> starts;
    std::chrono::high_resolution_clock::time_point reportStart = std::chrono::high_resolution_clock::now();

public:
    static auto instance() -> Profiler& {
        static Profiler p;
        return p;
    }

    void start(const std::string& name) {
        starts[name] = std::chrono::high_resolution_clock::now();
    }

    void end(const std::string& name) {
        auto end = std::chrono::high_resolution_clock::now();
        auto start = starts[name];
        std::chrono::duration<double, std::milli> diff = end - start;
        entries[name].totalTime += diff.count();
        entries[name].count++;
    }

    void report() {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - reportStart;
        if (elapsed.count() > 2.0) { // Report every 2 seconds
            std::cout << "\n--- PERF REPORT (avg ms/frame) ---\n";
            for (auto& [name, entry] : entries) {
                if (entry.count > 0)
                    std::cout << std::format("{}: {:.3f} ms\n", name, entry.totalTime / entry.count);
                entry.totalTime = 0;
                entry.count = 0;
            }
            std::cout << "----------------------------------\n";
            reportStart = now;
        }
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
            // std::cout << "Logging to file: " << LOG_FILE_NAME << std::endl;
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

    auto getTimestamp() const -> std::string {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;

        char buf[20];
        std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&time));
        return std::format("{}.{:03d}", buf, ms.count());
    }

public:
    static auto instance() -> Logger& {
        static Logger logger;
        return logger;
    }

    Logger(const Logger&) = delete;
    auto operator=(const Logger&) -> Logger& = delete;

    template<typename... Args>
    auto info(std::format_string<Args...> fmt, Args&&... args) -> void {
        std::string message = std::format("[{}] INFO: ", getTimestamp()) + std::format(fmt, std::forward<Args>(args)...);
        // std::cout << message << "\n"; // Disabled console spam for performance
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }

    template<typename... Args>
    auto debug(std::format_string<Args...> fmt, Args&&... args) -> void {
        // Debug logs only to file
        std::string message = std::format("[{}] DEBUG: ", getTimestamp()) + std::format(fmt, std::forward<Args>(args)...);
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }

    template<typename... Args>
    auto error(std::format_string<Args...> fmt, Args&&... args) -> void {
        std::string message = std::format("[{}] ERROR: ", getTimestamp()) + std::format(fmt, std::forward<Args>(args)...);
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

#define PROFILE_START(name) Profiler::instance().start(name)
#define PROFILE_END(name) Profiler::instance().end(name)
#define PROFILE_REPORT() Profiler::instance().report()