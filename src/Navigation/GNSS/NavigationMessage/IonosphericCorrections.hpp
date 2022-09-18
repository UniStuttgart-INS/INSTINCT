// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file IonosphericCorrections.hpp
/// @brief Ionospheric Correction data
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-07

#pragma once

#include <vector>
#include <optional>
#include <functional>

#include "util/Assert.h"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"

namespace NAV
{

/// @brief Ionospheric Corrections
class IonosphericCorrections
{
  public:
    /// @brief Alpha or beta values
    enum AlphaBeta
    {
        A, ///< Alpha
        B, ///< Beta
    };

    /// @brief Get the Ionospheric Correction values
    /// @param[in] satSys Satellite System to search the value for
    /// @param[in] alphaBeta Alpha or beta values (Galileo only alpha)
    /// @return List with the correction values (3 for GAL, otherwise 4)
    [[nodiscard]] std::optional<std::reference_wrapper<const std::vector<double>>> get(SatelliteSystem satSys, AlphaBeta alphaBeta) const
    {
        auto iter = std::find_if(_ionosphericCorrections.begin(), _ionosphericCorrections.end(), [satSys, alphaBeta](const Corrections& c) {
            return c.satSys == satSys && c.alphaBeta == alphaBeta;
        });
        if (iter != _ionosphericCorrections.end())
        {
            return std::cref(iter->data);
        }
        return std::nullopt;
    }

  private:
    /// @brief Inserts new data into the _ionosphericCorrections variable
    /// @param[in] satSys Satellite System to search the value for
    /// @param[in] alphaBeta Alpha or beta values (Galileo only alpha)
    /// @param[in] values Values to add
    void insert(SatelliteSystem satSys, AlphaBeta alphaBeta, const std::vector<double>& values)
    {
        auto iter = std::find_if(_ionosphericCorrections.begin(), _ionosphericCorrections.end(), [satSys, alphaBeta](const Corrections& c) {
            return c.satSys == satSys && c.alphaBeta == alphaBeta;
        });
        if (iter == _ionosphericCorrections.end())
        {
            _ionosphericCorrections.push_back({ satSys, alphaBeta, values });
        }
        else
        {
            iter->data = values;
        }
    }

    /// @brief Empties the data
    void clear()
    {
        _ionosphericCorrections.clear();
    }

    /// @brief Internal Data Storage
    struct Corrections
    {
        SatelliteSystem satSys;   ///< Satellite System (e.g. GPS, GAL, GLO, ...)
        AlphaBeta alphaBeta;      ///< Alpha or beta value
        std::vector<double> data; ///< Data storage (3 values for GAL, otherwise 4)
    };

    /// @brief Ionospheric correction values
    std::vector<Corrections> _ionosphericCorrections;

    friend class RinexNavFile;
};

} // namespace NAV
