// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CallbackTimer.hpp"

#include <chrono>

CallbackTimer::~CallbackTimer()
{
    if (_execute.load(std::memory_order_acquire))
    {
        stop();
    };
}

void CallbackTimer::stop()
{
    _execute.store(false, std::memory_order_release);
    if (_thd.joinable())
    {
        _thd.join();
    }
}

void CallbackTimer::start(int interval, const std::function<void(void*)>& func, void* userData)
{
    if (_execute.load(std::memory_order_acquire))
    {
        stop();
    };
    _interval.store(interval, std::memory_order_release);
    _execute.store(true, std::memory_order_release);
    _thd = std::thread([this, func, userData]() {
        while (_execute.load(std::memory_order_acquire))
        {
            auto start = std::chrono::steady_clock::now();
            func(userData);
            // std::this_thread::sleep_for(
            //     std::chrono::milliseconds(_interval));

            std::this_thread::sleep_until(start + std::chrono::milliseconds(_interval.load(std::memory_order_acquire)));
        }
    });
}

void CallbackTimer::setInterval(int interval)
{
    _interval.store(interval, std::memory_order_release);
}

bool CallbackTimer::is_running() const noexcept
{
    return (_execute.load(std::memory_order_acquire) && _thd.joinable());
}