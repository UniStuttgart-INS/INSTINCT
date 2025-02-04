// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NodeData.hpp
/// @brief Abstract NodeData Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-04-16

#pragma once

#include <string>
#include <vector>
#include <optional>

#include "Navigation/Time/InsTime.hpp"
#include "util/Assert.h"
#include "util/Logger.hpp"

namespace NAV
{
/// @brief Parent class for all data transmitted over Flow pins
class NodeData
{
  public:
    /// @brief Default constructor
    NodeData() = default;
    /// @brief Destructor
    virtual ~NodeData() = default;
    /// @brief Copy constructor
    NodeData(const NodeData&) = default;
    /// @brief Move constructor
    NodeData(NodeData&&) = default;
    /// @brief Copy assignment operator
    NodeData& operator=(const NodeData&) = default;
    /// @brief Move assignment operator
    NodeData& operator=(NodeData&&) = default;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type() { return "NodeData"; }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] virtual std::string getType() const { return type(); };

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes() { return {}; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors() { return {}; }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 0; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] virtual std::vector<std::string> staticDataDescriptors() const { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] virtual size_t staticDescriptorCount() const { return GetStaticDescriptorCount(); }

    /// @brief Returns a vector of string events associated with this data
    [[nodiscard]] const std::vector<std::string>& events() const { return _events; }

    /// @brief Adds the event to the list
    /// @param[in] text Event text
    void addEvent(const std::string& text) { _events.push_back(text); }

    /// @brief Get the value at the index
    /// @return Value if in the observation
    [[nodiscard]] virtual std::optional<double> getValueAt(size_t /* idx */) const { return std::nullopt; }

    /// @brief Set the value at the index
    /// @return True if the value was updated
    [[nodiscard]] virtual bool setValueAt(size_t /* idx */, double /* value */)
    {
        LOG_CRITICAL("This function should never be called. Did you forget to override it?");
        return false;
    }

    /// @brief Get the value at the index or NaN if not in the observation
    /// @param idx Index corresponding to data descriptor order
    /// @return Value or NaN if not in the observation
    [[nodiscard]] double getValueAtOrNaN(size_t idx) const { return getValueAt(idx).value_or(std::nan("")); }

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] virtual std::vector<std::string> dynamicDataDescriptors() const { return {}; }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] virtual std::optional<double> getDynamicDataAt(const std::string& /* descriptor */) const { return std::nullopt; }

    /// @brief Set the value for the descriptor
    /// @return True if the value was updated
    [[nodiscard]] virtual bool setDynamicDataAt(const std::string& /* descriptor */, double /* value */)
    {
        LOG_CRITICAL("This function should never be called. Did you forget to override it?");
        return false;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] virtual std::vector<std::pair<std::string, double>> getDynamicData() const { return {}; }

    /// @brief Shows a GUI tooltip to look into details of the observation
    /// @param[in] detailView Flag to show the detailed view
    /// @param[in] firstOpen Flag whether the tooltip is opened once
    /// @param[in] displayName Data identifier, can be used in dynamic data to identify the correct data
    /// @param[in] id Unique identifier
    /// @param[in] rootWindow Pointer to the root window opening the tooltip
    virtual void guiTooltip([[maybe_unused]] bool detailView, [[maybe_unused]] bool firstOpen,
                            [[maybe_unused]] const char* displayName, [[maybe_unused]] const char* id,
                            [[maybe_unused]] int* rootWindow) const {}

    /// @brief Return whether this data has a tooltip
    [[nodiscard]] virtual bool hasTooltip() const { return false; }

    /// Time at which the message was received
    InsTime insTime;

  protected:
    /// @brief List of events
    std::vector<std::string> _events;
};

} // namespace NAV
