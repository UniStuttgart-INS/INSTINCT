// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file dist_filter_sink.hpp
/// @brief Distribution sink with filter option. Stores a vector of sinks which get called when log is called
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-03-27

#pragma once

#include "spdlog/sinks/dist_sink.h"

namespace spdlog::sinks
{

/// Distribution sink (mux) with filter option
template<typename Mutex>
class dist_filter_sink : public spdlog::sinks::dist_sink<Mutex>
{
  public:
    /// @brief Default constructor
    /// @param[in] filter Filter string
    explicit dist_filter_sink(std::string filter)
        : filter_(std::move(filter)){};
    /// @brief Destructor
    ~dist_filter_sink() override = default;
    /// @brief Copy constructor
    dist_filter_sink(const dist_filter_sink&) = delete;
    /// @brief Move constructor
    dist_filter_sink(dist_filter_sink&&) noexcept = default;
    /// @brief Copy assignment operator
    dist_filter_sink& operator=(const dist_filter_sink&) = delete;
    /// @brief Move assignment operator
    dist_filter_sink& operator=(dist_filter_sink&&) noexcept = default;

  protected:
    /// String to filter messages for
    std::string filter_;

    /// @brief Function called to process the log message
    /// @param msg Log message struct
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        spdlog::memory_buf_t formatted_buf;
        spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted_buf);
        std::string formatted = fmt::to_string(formatted_buf);

        if (formatted.find(filter_) != std::string::npos)
        {
            spdlog::sinks::dist_sink<Mutex>::sink_it_(msg);
        }
    }
};

#ifndef DOXYGEN_IGNORE

    #include "spdlog/details/null_mutex.h"
    #include <mutex>
using dist_filter_sink_mt = dist_filter_sink<std::mutex>;
using dist_filter_sink_st = dist_filter_sink<spdlog::details::null_mutex>;

#endif

} // namespace spdlog::sinks