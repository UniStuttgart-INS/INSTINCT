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

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"

namespace NAV
{

/// GNSS Navigation message information
class GnssNavInfo;

/// @brief Ionospheric Corrections
class IonosphericCorrections
{
  public:
    /// @brief Alpha or beta values
    enum AlphaBeta
    {
        Alpha, ///< Coefficients of a cubic equation representing the amplitude of the vertical delay
        Beta,  ///< Coefficients of a cubic equation representing the period of the model
    };

    /// @brief Ionospheric Corrections Data Storage
    struct Corrections
    {
        SatelliteSystem satSys;     ///< Satellite System (e.g. GPS, GAL, GLO, ...)
        AlphaBeta alphaBeta;        ///< Alpha or beta value
        std::array<double, 4> data; ///< Data storage (3 values for GAL, otherwise 4)
    };

    /// @brief Default constructor
    IonosphericCorrections() = default;

    /// @brief Constructor which collects the ionospheric parameters from the Navigation infos
    /// @param[in] gnssNavInfos List of GNSS navigation infos
    explicit IonosphericCorrections(const std::vector<const GnssNavInfo*>& gnssNavInfos);

    /// @brief Constructor from raw corrections
    /// @param corrections Corrections vector
    IonosphericCorrections(const std::vector<Corrections>& corrections); // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)

    /// @brief Get the Ionospheric Correction values
    /// @param[in] satSys Satellite System to search the value for
    /// @param[in] alphaBeta Alpha or beta values (Galileo only alpha)
    /// @return List with the correction values (3 for GAL, otherwise 4)
    [[nodiscard]] const std::array<double, 4>* get(SatelliteSystem satSys, AlphaBeta alphaBeta) const
    {
        auto iter = std::find_if(m_ionosphericCorrections.begin(), m_ionosphericCorrections.end(), [satSys, alphaBeta](const Corrections& c) {
            return c.satSys == satSys && c.alphaBeta == alphaBeta;
        });
        if (iter != m_ionosphericCorrections.end())
        {
            return &iter->data;
        }
        return nullptr;
    }

    /// @brief Checks whether the data is in the internal storage
    /// @param[in] satSys Satellite System to search the value for
    /// @param[in] alphaBeta Alpha or beta values (Galileo only alpha)
    [[nodiscard]] bool contains(SatelliteSystem satSys, AlphaBeta alphaBeta) const
    {
        auto iter = std::find_if(m_ionosphericCorrections.begin(), m_ionosphericCorrections.end(), [satSys, alphaBeta](const Corrections& c) {
            return c.satSys == satSys && c.alphaBeta == alphaBeta;
        });
        return iter != m_ionosphericCorrections.end();
    }

    /// @brief Returns the internal data storage
    [[nodiscard]] const std::vector<Corrections>& data() const
    {
        return m_ionosphericCorrections;
    }

    /// @brief Inserts new data into the m_ionosphericCorrections variable
    /// @param[in] satSys Satellite System to search the value for
    /// @param[in] alphaBeta Alpha or beta values (Galileo only alpha)
    /// @param[in] values Values to add
    void insert(SatelliteSystem satSys, AlphaBeta alphaBeta, const std::array<double, 4>& values)
    {
        auto iter = std::find_if(m_ionosphericCorrections.begin(), m_ionosphericCorrections.end(), [satSys, alphaBeta](const Corrections& c) {
            return c.satSys == satSys && c.alphaBeta == alphaBeta;
        });
        if (iter == m_ionosphericCorrections.end())
        {
            m_ionosphericCorrections.push_back({ satSys, alphaBeta, values });
        }
        else
        {
            iter->data = values;
        }
    }

    /// @brief Empties the data
    void clear()
    {
        m_ionosphericCorrections.clear();
    }

  private:
    /// @brief Ionospheric correction values
    std::vector<Corrections> m_ionosphericCorrections;
};

} // namespace NAV
