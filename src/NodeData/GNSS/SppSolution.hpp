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

#include <vector>
#include <optional>
#include <algorithm>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Positioning/ReceiverClock.hpp"
#include "Navigation/GNSS/Positioning/SPP/Keys.hpp"

#include "NodeData/State/PosVel.hpp"

#include "util/Assert.h"
#include "util/Container/KeyedMatrix.hpp"

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
        desc.emplace_back("Receiver clock bias [s]");
        desc.emplace_back("Receiver clock drift [s/s]");
        desc.emplace_back("Receiver clock bias StDev [s]");
        desc.emplace_back("Receiver clock drift StDev [s/s]");
        desc.emplace_back("System time reference system");
        desc.emplace_back("GPS system time difference [s]");
        desc.emplace_back("GAL system time difference [s]");
        desc.emplace_back("GLO system time difference [s]");
        desc.emplace_back("BDS system time difference [s]");
        desc.emplace_back("QZSS system time difference [s]");
        desc.emplace_back("IRNSS system time difference [s]");
        desc.emplace_back("SBAS system time difference [s]");
        desc.emplace_back("GPS system time drift difference [s/s]");
        desc.emplace_back("GAL system time drift difference [s/s]");
        desc.emplace_back("GLO system time drift difference [s/s]");
        desc.emplace_back("BDS system time drift difference [s/s]");
        desc.emplace_back("QZSS system time drift difference [s/s]");
        desc.emplace_back("IRNSS system time drift difference [s/s]");
        desc.emplace_back("SBAS system time drift difference [s/s]");
        desc.emplace_back("GPS system time difference StDev [s]");
        desc.emplace_back("GAL system time difference StDev [s]");
        desc.emplace_back("GLO system time difference StDev [s]");
        desc.emplace_back("BDS system time difference StDev [s]");
        desc.emplace_back("QZSS system time difference StDev [s]");
        desc.emplace_back("IRNSS system time difference StDev [s]");
        desc.emplace_back("SBAS system time difference StDev [s]");
        desc.emplace_back("GPS system time drift difference StDev [s/s]");
        desc.emplace_back("GAL system time drift difference StDev [s/s]");
        desc.emplace_back("GLO system time drift difference StDev [s/s]");
        desc.emplace_back("BDS system time drift difference StDev [s/s]");
        desc.emplace_back("QZSS system time drift difference StDev [s/s]");
        desc.emplace_back("IRNSS system time drift difference StDev [s/s]");
        desc.emplace_back("SBAS system time drift difference StDev [s/s]");

        return desc;
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 73; }

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
        switch (idx)
        {
        case 0:  // Latitude [deg]
        case 1:  // Longitude [deg]
        case 2:  // Altitude [m]
        case 3:  // North/South [m]
        case 4:  // East/West [m]
        case 5:  // X-ECEF [m]
        case 6:  // Y-ECEF [m]
        case 7:  // Z-ECEF [m]
        case 8:  // X-ECEF StDev [m]
        case 9:  // Y-ECEF StDev [m]
        case 10: // Z-ECEF StDev [m]
        case 11: // XY-ECEF StDev [m]
        case 12: // XZ-ECEF StDev [m]
        case 13: // YZ-ECEF StDev [m]
        case 14: // North StDev [m]
        case 15: // East StDev [m]
        case 16: // Down StDev [m]
        case 17: // NE StDev [m]
        case 18: // ND StDev [m]
        case 19: // ED StDev [m]
        case 20: // Velocity norm [m/s]
        case 21: // X velocity ECEF [m/s]
        case 22: // Y velocity ECEF [m/s]
        case 23: // Z velocity ECEF [m/s]
        case 24: // North velocity [m/s]
        case 25: // East velocity [m/s]
        case 26: // Down velocity [m/s]
        case 27: // X velocity ECEF StDev [m/s]
        case 28: // Y velocity ECEF StDev [m/s]
        case 29: // Z velocity ECEF StDev [m/s]
        case 30: // XY velocity StDev [m]
        case 31: // XZ velocity StDev [m]
        case 32: // YZ velocity StDev [m]
        case 33: // North velocity StDev [m/s]
        case 34: // East velocity StDev [m/s]
        case 35: // Down velocity StDev [m/s]
        case 36: // NE velocity StDev [m]
        case 37: // ND velocity StDev [m]
        case 38: // ED velocity StDev [m]
            return PosVel::getValueAt(idx);
        case 39: // Number satellites
            return static_cast<double>(nSatellites);
        case 40: // Receiver clock bias [s]
            return recvClk.bias.value;
        case 41: // Receiver clock drift [s/s]
            if (recvClk.drift.value != 0.0) { return recvClk.drift.value; }
            break;
        case 42: // Receiver clock bias StDev [s]
            return recvClk.bias.stdDev;
        case 43: // Receiver clock drift StDev [s/s]
            if (recvClk.drift.value != 0.0) { return recvClk.drift.stdDev; }
            break;
        case 44: // System time reference system
            return static_cast<double>(recvClk.referenceTimeSatelliteSystem.toEnumeration());
        case 45: // GPS system time difference [s]
            if (recvClk.sysTimeDiffBias.at(idx - 45).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 45).value; }
            break;
        case 46: // GAL system time difference [s]
            if (recvClk.sysTimeDiffBias.at(idx - 45).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 45).value; }
            break;
        case 47: // GLO system time difference [s]
            if (recvClk.sysTimeDiffBias.at(idx - 45).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 45).value; }
            break;
        case 48: // BDS system time difference [s]
            if (recvClk.sysTimeDiffBias.at(idx - 45).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 45).value; }
            break;
        case 49: // QZSS system time difference [s]
            if (recvClk.sysTimeDiffBias.at(idx - 45).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 45).value; }
            break;
        case 50: // IRNSS system time difference [s]
            if (recvClk.sysTimeDiffBias.at(idx - 45).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 45).value; }
            break;
        case 51: // SBAS system time difference [s]
            if (recvClk.sysTimeDiffBias.at(idx - 45).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 45).value; }
            break;
        case 52: // GPS system time drift difference [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 52).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 52).value; }
            break;
        case 53: // GAL system time drift difference [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 52).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 52).value; }
            break;
        case 54: // GLO system time drift difference [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 52).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 52).value; }
            break;
        case 55: // BDS system time drift difference [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 52).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 52).value; }
            break;
        case 56: // QZSS system time drift difference [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 52).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 52).value; }
            break;
        case 57: // IRNSS system time drift difference [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 52).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 52).value; }
            break;
        case 58: // SBAS system time drift difference [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 52).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 52).value; }
            break;
        case 59: // GPS system time difference StDev [s]
            if (recvClk.sysTimeDiffBias.at(idx - 59).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 59).stdDev; }
            break;
        case 60: // GAL system time difference StDev [s]
            if (recvClk.sysTimeDiffBias.at(idx - 59).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 59).stdDev; }
            break;
        case 61: // GLO system time difference StDev [s]
            if (recvClk.sysTimeDiffBias.at(idx - 59).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 59).stdDev; }
            break;
        case 62: // BDS system time difference StDev [s]
            if (recvClk.sysTimeDiffBias.at(idx - 59).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 59).stdDev; }
            break;
        case 63: // QZSS system time difference StDev [s]
            if (recvClk.sysTimeDiffBias.at(idx - 59).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 59).stdDev; }
            break;
        case 64: // IRNSS system time difference StDev [s]
            if (recvClk.sysTimeDiffBias.at(idx - 59).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 59).stdDev; }
            break;
        case 65: // SBAS system time difference StDev [s]
            if (recvClk.sysTimeDiffBias.at(idx - 59).value != 0.0) { return recvClk.sysTimeDiffBias.at(idx - 59).stdDev; }
            break;
        case 66: // GPS system time drift difference StDev [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 66).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 66).stdDev; }
            break;
        case 67: // GAL system time drift difference StDev [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 66).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 66).stdDev; }
            break;
        case 68: // GLO system time drift difference StDev [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 66).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 66).stdDev; }
            break;
        case 69: // BDS system time drift difference StDev [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 66).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 66).stdDev; }
            break;
        case 70: // QZSS system time drift difference StDev [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 66).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 66).stdDev; }
            break;
        case 71: // IRNSS system time drift difference StDev [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 66).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 66).stdDev; }
            break;
        case 72: // SBAS system time drift difference StDev [s/s]
            if (recvClk.sysTimeDiffDrift.at(idx - 66).value != 0.0) { return recvClk.sysTimeDiffDrift.at(idx - 66).stdDev; }
            break;
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

    /// Estimated receiver clock parameter
    ReceiverClock recvClk;

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

    /// @brief Adds an event to the event list
    /// @param event Event string
    void addEvent(const std::string& event) { _events.push_back(event); }

  private:
    /// Standard deviation of Position in ECEF coordinates [m]
    Eigen::Vector3d _e_positionStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Position in local navigation frame coordinates [m]
    Eigen::Vector3d _n_positionStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Standard deviation of Velocity in earth coordinates [m/s]
    Eigen::Vector3d _e_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");
    /// Standard deviation of Velocity in navigation coordinates [m/s]
    Eigen::Vector3d _n_velocityStdev = Eigen::Vector3d::Zero() * std::nan("");

    /// Covariance matrix in ECEF coordinates
    std::optional<KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>> _e_covarianceMatrix;

    /// Covariance matrix in local navigation coordinates
    std::optional<KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>> _n_covarianceMatrix;
};

} // namespace NAV
