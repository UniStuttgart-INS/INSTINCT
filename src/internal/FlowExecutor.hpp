/// @file FlowExecutor.hpp
/// @brief Flow Executor Thread
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-02

#pragma once

namespace NAV::FlowExecutor
{
/// @brief Checks if the thread is running
[[nodiscard]] bool isRunning() noexcept;

/// @brief Starts the Thread
void start();

/// @brief Stops the Thread
void stop();

/// @brief Waits for a thread to finish its execution
void waitForFinish();

} // namespace NAV::FlowExecutor
