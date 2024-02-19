// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UbloxGnssObsConverter.hpp
/// @brief Convert UbloxObs into GnssObs
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-05

#pragma once

#include <unordered_set>

#include "internal/Node/Node.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

namespace NAV
{
/// Convert UbloxObs into GnssObs
class UbloxGnssObsConverter : public Node
{
  public:
    /// @brief Default constructor
    UbloxGnssObsConverter();
    /// @brief Destructor
    ~UbloxGnssObsConverter() override;
    /// @brief Copy constructor
    UbloxGnssObsConverter(const UbloxGnssObsConverter&) = delete;
    /// @brief Move constructor
    UbloxGnssObsConverter(UbloxGnssObsConverter&&) = delete;
    /// @brief Copy assignment operator
    UbloxGnssObsConverter& operator=(const UbloxGnssObsConverter&) = delete;
    /// @brief Move assignment operator
    UbloxGnssObsConverter& operator=(UbloxGnssObsConverter&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_GNSS_OBS = 0; ///< @brief Flow

    /// List of signals of the last epoch. To set the LLI flag
    std::unordered_set<SatSigId> _lastEpochObs;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Data receive function
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);
};

} // namespace NAV
