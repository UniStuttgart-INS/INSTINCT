// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppSolution.hpp
/// @brief SPP Algorithm output
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-28

#pragma once

#include <cstddef>
#include <vector>
#include <optional>
#include <algorithm>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"
#include "Navigation/GNSS/Positioning/SPP/Keys.hpp"

#include "NodeData/State/PosVel.hpp"

#include "util/Assert.h"
#include "util/Container/KeyedMatrix.hpp"
#include "util/Container/UncertainValue.hpp"

namespace NAV
{
/// SPP Algorithm output
class SppSolution : public PosVel
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "SppSolution";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = PosVel::parentTypes();
        parent.push_back(PosVel::type());
        return parent;
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        auto desc = PosVel::GetStaticDataDescriptors();
        desc.reserve(GetStaticDescriptorCount());
        desc.emplace_back("Number satellites");
        desc.emplace_back("Receiver clock bias GPS [s]");
        desc.emplace_back("Receiver clock bias StDev GPS [s]");
        desc.emplace_back("Receiver clock bias GAL [s]");
        desc.emplace_back("Receiver clock bias StDev GAL [s]");
        desc.emplace_back("Receiver clock bias GLO [s]");
        desc.emplace_back("Receiver clock bias StDev GLO [s]");
        desc.emplace_back("Receiver clock bias BDS [s]");
        desc.emplace_back("Receiver clock bias StDev BDS [s]");
        desc.emplace_back("Receiver clock bias QZSS [s]");
        desc.emplace_back("Receiver clock bias StDev QZSS [s]");
        desc.emplace_back("Receiver clock bias IRNSS [s]");
        desc.emplace_back("Receiver clock bias StDev IRNSS [s]");
        desc.emplace_back("Receiver clock bias SBAS [s]");
        desc.emplace_back("Receiver clock bias StDev SBAS [s]");
        desc.emplace_back("Receiver clock drift [s/s]");
        desc.emplace_back("Receiver clock drift StDev [s/s]");
        desc.emplace_back("HDOP");
        desc.emplace_back("VDOP");
        desc.emplace_back("PDOP");

        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return PosVel::GetStaticDescriptorCount() + 20; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        if (idx < PosVel::GetStaticDescriptorCount()) { return PosVel::getValueAt(idx); }
        switch (idx)
        {
        case PosVel::GetStaticDescriptorCount() + 0: // Number satellites
            return static_cast<double>(nSatellites);

        case PosVel::GetStaticDescriptorCount() + 1: // Receiver clock bias GPS [s]
            if (const double* v = recvClk.biasFor(GPS)) { return *v; }
            break;
        case PosVel::GetStaticDescriptorCount() + 2: // Receiver clock bias StDev GPS [s]
            if (const double* v = recvClk.biasStdDevFor(GPS)) { return *v; }
            break;

        case PosVel::GetStaticDescriptorCount() + 3: // Receiver clock bias GAL [s]
            if (const double* v = recvClk.biasFor(GAL)) { return *v; }
            break;
        case PosVel::GetStaticDescriptorCount() + 4: // Receiver clock bias StDev GAL [s]
            if (const double* v = recvClk.biasStdDevFor(GAL)) { return *v; }
            break;

        case PosVel::GetStaticDescriptorCount() + 5: // Receiver clock bias GLO [s]
            if (const double* v = recvClk.biasFor(GLO)) { return *v; }
            break;
        case PosVel::GetStaticDescriptorCount() + 6: // Receiver clock bias StDev GLO [s]
            if (const double* v = recvClk.biasStdDevFor(GLO)) { return *v; }
            break;

        case PosVel::GetStaticDescriptorCount() + 7: // Receiver clock bias BDS [s]
            if (const double* v = recvClk.biasFor(BDS)) { return *v; }
            break;
        case PosVel::GetStaticDescriptorCount() + 8: // Receiver clock bias StDev BDS [s]
            if (const double* v = recvClk.biasStdDevFor(BDS)) { return *v; }
            break;

        case PosVel::GetStaticDescriptorCount() + 9: // Receiver clock bias QZSS [s]
            if (const double* v = recvClk.biasFor(QZSS)) { return *v; }
            break;
        case PosVel::GetStaticDescriptorCount() + 10: // Receiver clock bias StDev QZSS [s]
            if (const double* v = recvClk.biasStdDevFor(QZSS)) { return *v; }
            break;

        case PosVel::GetStaticDescriptorCount() + 11: // Receiver clock bias IRNSS [s]
            if (const double* v = recvClk.biasFor(IRNSS)) { return *v; }
            break;
        case PosVel::GetStaticDescriptorCount() + 12: // Receiver clock bias StDev IRNSS [s]
            if (const double* v = recvClk.biasStdDevFor(IRNSS)) { return *v; }
            break;

        case PosVel::GetStaticDescriptorCount() + 13: // Receiver clock bias SBAS [s]
            if (const double* v = recvClk.biasFor(SBAS)) { return *v; }
            break;
        case PosVel::GetStaticDescriptorCount() + 14: // Receiver clock bias StDev SBAS [s]
            if (const double* v = recvClk.biasStdDevFor(SBAS)) { return *v; }
            break;

        case PosVel::GetStaticDescriptorCount() + 15: // Receiver clock drift [s/s]
            return recvClk.drift;
            break;
        case PosVel::GetStaticDescriptorCount() + 16: // Receiver clock drift StDev [s/s]
            return recvClk.driftStdDev;
            break;

        case PosVel::GetStaticDescriptorCount() + 17: // HDOP
            return HDOP;
        case PosVel::GetStaticDescriptorCount() + 18: // VDOP
            return VDOP;
        case PosVel::GetStaticDescriptorCount() + 19: // PDOP
            return PDOP;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors for the dynamic data
    [[nodiscard]] std::vector<std::string> dynamicDataDescriptors() const override
    {
        std::vector<std::string> descriptors;
        descriptors.reserve(interFrequencyBias.size() * 2 + satData.size() * 2);

        for (const auto& bias : interFrequencyBias)
        {
            descriptors.push_back(fmt::format("{} Inter-freq bias [s]", bias.first));
            descriptors.push_back(fmt::format("{} Inter-freq bias StDev [s]", bias.first));
        }
        for (const auto& [satId, satData] : satData)
        {
            descriptors.push_back(fmt::format("{} Elevation [deg]", satId));
            descriptors.push_back(fmt::format("{} Azimuth [deg]", satId));
            // descriptors.push_back(fmt::format("{} Satellite clock bias [s]", satData.first));
            // descriptors.push_back(fmt::format("{} Satellite clock drift [s/s]", satData.first));
            // descriptors.push_back(fmt::format("{} SatPos ECEF X [m]", satData.first));
            // descriptors.push_back(fmt::format("{} SatPos ECEF Y [m]", satData.first));
            // descriptors.push_back(fmt::format("{} SatPos ECEF Z [m]", satData.first));
            // descriptors.push_back(fmt::format("{} SatPos Latitude [deg]", satData.first));
            // descriptors.push_back(fmt::format("{} SatPos Longitude [deg]", satData.first));
            // descriptors.push_back(fmt::format("{} SatPos Altitude [m]", satData.first));
            // descriptors.push_back(fmt::format("{} SatVel ECEF X [m/s]", satData.first));
            // descriptors.push_back(fmt::format("{} SatVel ECEF Y [m/s]", satData.first));
            // descriptors.push_back(fmt::format("{} SatVel ECEF Z [m/s]", satData.first));
        }
        if (_e_sppCovarianceMatrix)
        {
            for (size_t r = 0; r < static_cast<size_t>(_e_sppCovarianceMatrix->rows()); r++)
            {
                for (size_t c = r; c < static_cast<size_t>(_e_sppCovarianceMatrix->cols()); c++)
                {
                    descriptors.push_back(fmt::format("Cov({},{})", _e_sppCovarianceMatrix->rowKeys().at(r), _e_sppCovarianceMatrix->colKeys().at(c)));
                }
            }
        }

        return descriptors;
    }

    /// @brief Get the value for the descriptor
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getDynamicDataAt(const std::string& descriptor) const override
    {
        for (const auto& bias : interFrequencyBias)
        {
            if (descriptor == fmt::format("{} Inter-freq bias [s]", bias.first)) { return bias.second.value; }
            if (descriptor == fmt::format("{} Inter-freq bias StDev [s]", bias.first)) { return bias.second.stdDev; }
        }
        for (const auto& [satId, satData] : satData)
        {
            if (descriptor == fmt::format("{} Elevation [deg]", satId)) { return rad2deg(satData.satElevation); }
            if (descriptor == fmt::format("{} Azimuth [deg]", satId)) { return rad2deg(satData.satAzimuth); }
            // if (descriptor == fmt::format("{} Satellite clock bias [s]", satData.first)) { return satData.second.satClock.bias; }
            // if (descriptor == fmt::format("{} Satellite clock drift [s/s]", satData.first)) { return satData.second.satClock.drift; }
            // if (descriptor == fmt::format("{} SatPos ECEF X [m]", satData.first)) { return satData.second.e_satPos.x(); }
            // if (descriptor == fmt::format("{} SatPos ECEF Y [m]", satData.first)) { return satData.second.e_satPos.y(); }
            // if (descriptor == fmt::format("{} SatPos ECEF Z [m]", satData.first)) { return satData.second.e_satPos.z(); }
            // if (descriptor == fmt::format("{} SatPos Latitude [deg]", satData.first)) { return rad2deg(satData.second.lla_satPos.x()); }
            // if (descriptor == fmt::format("{} SatPos Longitude [deg]", satData.first)) { return rad2deg(satData.second.lla_satPos.y()); }
            // if (descriptor == fmt::format("{} SatPos Altitude [m]", satData.first)) { return satData.second.lla_satPos.z(); }
            // if (descriptor == fmt::format("{} SatVel ECEF X [m/s]", satData.first)) { return satData.second.e_satVel.x(); }
            // if (descriptor == fmt::format("{} SatVel ECEF Y [m/s]", satData.first)) { return satData.second.e_satVel.y(); }
            // if (descriptor == fmt::format("{} SatVel ECEF Z [m/s]", satData.first)) { return satData.second.e_satVel.z(); }
        }
        if (_e_sppCovarianceMatrix)
        {
            for (size_t r = 0; r < static_cast<size_t>(_e_sppCovarianceMatrix->rows()); r++)
            {
                for (size_t c = r; c < static_cast<size_t>(_e_sppCovarianceMatrix->cols()); c++)
                {
                    if (descriptor == fmt::format("Cov({},{})", _e_sppCovarianceMatrix->rowKeys().at(r), _e_sppCovarianceMatrix->colKeys().at(c)))
                    {
                        return (*_e_sppCovarianceMatrix)(all, all)(static_cast<int>(r), static_cast<int>(c));
                    }
                }
            }
        }
        return std::nullopt;
    }

    /// @brief Returns a vector of data descriptors and values for the dynamic data
    [[nodiscard]] std::vector<std::pair<std::string, double>> getDynamicData() const override
    {
        std::vector<std::pair<std::string, double>> dynData;
        dynData.reserve(interFrequencyBias.size() * 2 + satData.size() * 2);

        for (const auto& bias : interFrequencyBias)
        {
            dynData.emplace_back(fmt::format("{} Inter-freq bias [s]", bias.first), bias.second.value);
            dynData.emplace_back(fmt::format("{} Inter-freq bias StDev [s]", bias.first), bias.second.stdDev);
        }
        for (const auto& [satId, satData] : satData)
        {
            dynData.emplace_back(fmt::format("{} Elevation [deg]", satId), rad2deg(satData.satElevation));
            dynData.emplace_back(fmt::format("{} Azimuth [deg]", satId), rad2deg(satData.satAzimuth));
            // dynData.emplace_back(fmt::format("{} Satellite clock bias [s]", satData.first), satData.second.satClock.bias);
            // dynData.emplace_back(fmt::format("{} Satellite clock drift [s/s]", satData.first), satData.second.satClock.drift);
            // dynData.emplace_back(fmt::format("{} SatPos ECEF X [m]", satData.first), satData.second.e_satPos.x());
            // dynData.emplace_back(fmt::format("{} SatPos ECEF Y [m]", satData.first), satData.second.e_satPos.y());
            // dynData.emplace_back(fmt::format("{} SatPos ECEF Z [m]", satData.first), satData.second.e_satPos.z());
            // dynData.emplace_back(fmt::format("{} SatPos Latitude [deg]", satData.first), rad2deg(satData.second.lla_satPos.x()));
            // dynData.emplace_back(fmt::format("{} SatPos Longitude [deg]", satData.first), rad2deg(satData.second.lla_satPos.y()));
            // dynData.emplace_back(fmt::format("{} SatPos Altitude [m]", satData.first), satData.second.lla_satPos.z());
            // dynData.emplace_back(fmt::format("{} SatVel ECEF X [m/s]", satData.first), satData.second.e_satVel.x());
            // dynData.emplace_back(fmt::format("{} SatVel ECEF Y [m/s]", satData.first), satData.second.e_satVel.y());
            // dynData.emplace_back(fmt::format("{} SatVel ECEF Z [m/s]", satData.first), satData.second.e_satVel.z());
        }
        if (_e_sppCovarianceMatrix)
        {
            for (size_t r = 0; r < static_cast<size_t>(_e_sppCovarianceMatrix->rows()); r++)
            {
                for (size_t c = r; c < static_cast<size_t>(_e_sppCovarianceMatrix->cols()); c++)
                {
                    dynData.emplace_back(fmt::format("Cov({},{})", _e_sppCovarianceMatrix->rowKeys().at(r), _e_sppCovarianceMatrix->colKeys().at(c)),
                                         (*_e_sppCovarianceMatrix)(all, all)(static_cast<int>(r), static_cast<int>(c)));
                }
            }
        }
        return dynData;
    }

    // --------------------------------------------------------- Public Members ------------------------------------------------------------

    /// Amount of satellites used for the calculation
    size_t nSatellites = 0;
    /// Amount of pseudorange measurements used to calculate the position solution
    size_t nMeasPsr = 0;
    /// Amount of doppler measurements used to calculate the velocity solution
    size_t nMeasDopp = 0;
    /// Amount of Parameters estimated in this epoch
    size_t nParam = 0;

    /// HDOP value
    double HDOP = std::nan("");
    /// VDOP value
    double VDOP = std::nan("");
    /// PDOP value
    double PDOP = std::nan("");

    /// Estimated receiver clock parameter
    ReceiverClock recvClk{ {} };

    /// Inter-frequency biases
    std::unordered_map<Frequency, UncertainValue<double>> interFrequencyBias;

    /// Satellite specific data
    struct SatData
    {
        double satElevation = 0.0; ///< Satellite Elevation [rad]
        double satAzimuth = 0.0;   ///< Satellite Azimuth [rad]
    };

    /// Extended data for each satellite
    std::vector<std::pair<SatId, SatData>> satData;

    /// Covariance matrix in ECEF coordinates
    std::optional<KeyedMatrixXd<SPP::States::StateKeyType, SPP::States::StateKeyType>> _e_sppCovarianceMatrix;
};

} // namespace NAV
