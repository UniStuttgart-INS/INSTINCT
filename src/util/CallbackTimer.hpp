/**
 * @file CallbackTimer.hpp
 * @brief Starts a Periodic Timer
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-07-15
 */

#pragma once

#include <functional>
#include <thread>
#include <atomic>

class CallbackTimer
{
  public:
    CallbackTimer() = default;                               ///< Default Constructor
    CallbackTimer(const CallbackTimer&) = delete;            ///< Copy constructor
    CallbackTimer(CallbackTimer&&) = delete;                 ///< Move constructor
    CallbackTimer& operator=(const CallbackTimer&) = delete; ///< Copy assignment operator
    CallbackTimer& operator=(CallbackTimer&&) = delete;      ///< Move assignment operator

    ~CallbackTimer();

    void stop();

    void start(int interval, const std::function<void(void*)>& func, void* userData);

    [[nodiscard]] bool is_running() const noexcept;

  private:
    std::atomic<bool> _execute{ false };
    std::thread _thd;
};