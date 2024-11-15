// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file FlowExecutor.hpp
/// @brief Flow Executor Thread
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-02

#pragma once

namespace NAV
{

class Node;

namespace FlowExecutor
{

/// @brief Checks if the thread is running
[[nodiscard]] bool isRunning() noexcept;

/// @brief Starts the Thread
void start();

/// @brief Stops the Thread
void stop();

/// @brief Waits for a thread to finish its execution
void waitForFinish();

/// @brief Called by nodes when they finished with sending data
/// @param[in] node The node to deregister
void deregisterNode(const Node* node);

} // namespace FlowExecutor

} // namespace NAV