// Copyright 2026
// Author: Wei Wang -> shaxikai@outlook.com

#ifndef NAV2_COMMON__SP_TIMER_H_
#define NAV2_COMMON__SP_TIMER_H_

#include <chrono>
#include <functional>
#include <string>

#define CONCAT_IMPL(a, b) a##b
#define CONCAT(a, b) CONCAT_IMPL(a, b)
#define FUNC_TIMER SPTimer CONCAT(timer_, __LINE__)(__func__);

class SPTimer {
public:
    using LogFn = std::function<void(const std::string& name, double ms)>;

    SPTimer(const std::string& name = "", LogFn fn = nullptr)
        : name_(name), fn_(fn), t0_(std::chrono::high_resolution_clock::now()) {}

    ~SPTimer() {
        if (fn_) {
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - t0_).count();
            fn_(name_, ms);
        }
    }

private:
    std::string name_;
    LogFn fn_;
    std::chrono::high_resolution_clock::time_point t0_;
};

#endif  // NAV2_COMMON__SP_TIMER_H_