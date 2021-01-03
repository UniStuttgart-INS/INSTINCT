/// @file FlowExecutor.hpp
/// @brief Flow Executor Thread
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-02

#pragma once

#include <thread>
#include <atomic>

namespace NAV
{
class FlowExecutor
{
  public:
    /// @brief Default constructor
    FlowExecutor() = default;
    /// @brief Destructor
    ~FlowExecutor();
    /// @brief Copy constructor
    FlowExecutor(const FlowExecutor&) = delete;
    /// @brief Move constructor
    FlowExecutor(FlowExecutor&&) = delete;
    /// @brief Copy assignment operator
    FlowExecutor& operator=(const FlowExecutor&) = delete;
    /// @brief Move assignment operator
    FlowExecutor& operator=(FlowExecutor&&) = delete;

    /// @brief Checks if the thread is running
    [[nodiscard]] bool isRunning() const noexcept;

    /// @brief Starts the Thread
    void start();

    /// @brief Stops the Thread
    void stop();

    /// @brief Waits for a thread to finish its execution
    void waitForFinish();

  private:
    /// @brief Initializes all Nodes if they are not initialized yet
    /// @return True if all nodes are initialized
    bool initialize();

    /// @brief Deinitialize all Nodes
    void deinitialize();

    /// @brief Main task of the thread
    void execute();

    std::atomic<bool> _execute{ false };
    std::thread _thd;
};

} // namespace NAV
