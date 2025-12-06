#pragma once
#include <iostream>
#include <format>
#include <string_view>
#include <chrono>

class Logger {
private:
    Logger() = default;

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
        std::cout << std::format("[{}] INFO: ", getTimestamp())
                  << std::format(fmt, std::forward<Args>(args)...) << "\n";
    }

    template<typename... Args>
    auto debug(std::format_string<Args...> fmt, Args&&... args) -> void {
        std::cout << std::format("[{}] DEBUG: ", getTimestamp())
                  << std::format(fmt, std::forward<Args>(args)...) << "\n";
    }

    template<typename... Args>
    auto error(std::format_string<Args...> fmt, Args&&... args) -> void {
        std::cerr << std::format("[{}] ERROR: ", getTimestamp())
                  << std::format(fmt, std::forward<Args>(args)...) << "\n";
    }
};

#define LOG_INFO(...) Logger::instance().info(__VA_ARGS__)
#define LOG_DEBUG(...) Logger::instance().debug(__VA_ARGS__)
#define LOG_ERROR(...) Logger::instance().error(__VA_ARGS__)
