/// @file CallbackTimer.hpp
/// @brief Starts a Periodic Timer
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-07-15

#pragma once

#include <functional>
#include <thread>
#include <atomic>

class CallbackTimer
{
  public:
    /// @brief Default constructor
    CallbackTimer() = default;
    /// @brief Copy constructor
    CallbackTimer(const CallbackTimer&) = delete;
    /// @brief Move constructor
    CallbackTimer(CallbackTimer&&) = delete;
    /// @brief Copy assignment operator
    CallbackTimer& operator=(const CallbackTimer&) = delete;
    /// @brief Move assignment operator
    CallbackTimer& operator=(CallbackTimer&&) = delete;
    /// @brief Destructor
    ~CallbackTimer();

    /// @brief Stops the Timer
    void stop();

    /// @brief Starts the timer
    /// @param[in] interval Interval in [ms] when to trigger the callback
    /// @param[in] func Function to call
    /// @param[in, out] userData User Data which will be passed to the callback function
    void start(int interval, const std::function<void(void*)>& func, void* userData);

    [[nodiscard]] bool is_running() const noexcept;

  private:
    std::atomic<bool> _execute{ false };
    std::thread _thd;
};