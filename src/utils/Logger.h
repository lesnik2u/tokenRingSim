#pragma once
#include <iostream>
#include <fstream> // Added for file logging
#include <format>
#include <string_view>
#include <chrono>

// Define a constant for the log file name
const std::string LOG_FILE_NAME = "simulation.log";

class Logger {
private:
    std::ofstream logFile; // Member for file logging

    Logger() {
        logFile.open(LOG_FILE_NAME, std::ios::out | std::ios::app); // Open log file in append mode
        if (!logFile.is_open()) {
            std::cerr << "Error: Could not open log file: " << LOG_FILE_NAME << std::endl;
        } else {
            // Write session separator and startup message
            logFile << "\n========================================\n";
            logFile << "New session started at " << getTimestamp() << "\n";
            logFile << "========================================\n";
            logFile.flush();
            std::cout << "Logging to file: " << LOG_FILE_NAME << std::endl;
        }
    }

    // Destructor to close the log file
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
        std::cout << message << "\n";
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush(); // Ensure message is written immediately
        }
    }

    template<typename... Args>
    auto debug(std::format_string<Args...> fmt, Args&&... args) -> void {
        std::string message = std::format("[{}] DEBUG: ", getTimestamp()) + std::format(fmt, std::forward<Args>(args)...);
        std::cout << message << "\n";
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }

    template<typename... Args>
    auto error(std::format_string<Args...> fmt, Args&&... args) -> void {
        std::string message = std::format("[{}] ERROR: ", getTimestamp()) + std::format(fmt, std::forward<Args>(args)...);
        std::cerr << message << "\n"; // Errors still go to cerr
        if (logFile.is_open()) {
            logFile << message << "\n";
            logFile.flush();
        }
    }
};

#define APP_LOG_INFO(...) Logger::instance().info(__VA_ARGS__)
#define APP_LOG_DEBUG(...) Logger::instance().debug(__VA_ARGS__)
#define APP_LOG_ERROR(...) Logger::instance().error(__VA_ARGS__)
