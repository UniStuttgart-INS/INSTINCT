// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LowPassFilter.hpp
/// @brief Filters incoming data
/// @author T. Hobiger (thomas.hobiger@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-12-20

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"

#include "util/Eigen.hpp"
#include <map>

namespace NAV
{
/// Filters incoming data
class LowPassFilter : public Node
{
  public:
    /// @brief Default constructor
    LowPassFilter();
    /// @brief Destructor
    ~LowPassFilter() override;
    /// @brief Copy constructor
    LowPassFilter(const LowPassFilter&) = delete;
    /// @brief Move constructor
    LowPassFilter(LowPassFilter&&) = delete;
    /// @brief Copy assignment operator
    LowPassFilter& operator=(const LowPassFilter&) = delete;
    /// @brief Move assignment operator
    LowPassFilter& operator=(LowPassFilter&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_FLOW = 0; ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_FLOW = 0;  ///< @brief Flow

    // ###########################################################################################################

    /// Types of available filters (to be extended)
    enum class FilterType : uint8_t
    {
        Linear, ///< Linear fit filter
        // Experimental,
        COUNT, ///< Amount of items in the enum
    };

    /// Filter description
    struct FilterItem
    {
        /// @brief Default Constructor
        FilterItem() = default;

        /// @brief Constructor
        /// @param[in] dataDescription Description of the data (dynamic data identifier)
        /// @param[in] dataIndex Index of the data (relevant for static data mostly)
        FilterItem(std::string dataDescription, size_t dataIndex)
            : dataDescription(std::move(dataDescription)), dataIndex(dataIndex) {}

        /// Description of the data
        std::string dataDescription;
        /// Index of the data
        size_t dataIndex = 0;
        /// Selected filter type in the GUI
        FilterType filterType = FilterType::Linear;
        /// Cutoff frequency [Hz], inverse of this parameter equals to fitting period
        double linear_filter_cutoff_frequency = 10.0;
        /// Map which stores all last data points which were used in the previous fit
        std::map<InsTime, double> dataToFilter;
        /// Flag to show indicator that it was modified
        bool modified = true;
    };

    /// @brief Selected item in the combo
    size_t _gui_availableItemsSelection = 0;

    /// Available items
    std::vector<std::string> _availableItems;
    /// Items to filter
    std::vector<FilterItem> _filterItems;

    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    static const char* to_string(FilterType value);

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

    /// @brief Called when a new link was established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Called when a link was deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterDeleteLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Callback when receiving data on a port
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Filter the provided data
    /// @param[in] item Filter item to fit
    /// @param[in] insTime Current Time
    /// @param[in] value Current value to filter
    [[nodiscard]] static std::optional<double> filterData(FilterItem& item, const InsTime& insTime, double value);

    friend void to_json(json& j, const FilterItem& data);
    friend void from_json(const json& j, FilterItem& data);
};

/// @brief Converts the provided link into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const NAV::LowPassFilter::FilterItem& data);
/// @brief Converts the provided json object into a link object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, NAV::LowPassFilter::FilterItem& data);

} // namespace NAV
