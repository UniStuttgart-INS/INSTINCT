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
    [[nodiscard]] static std::vector<std::string> GetDataDescriptors()
    {
        auto desc = PosVel::GetDataDescriptors();
        desc.reserve(GetDescriptorCount());
        desc.emplace_back("Number satellites");
        desc.emplace_back("Receiver clock bias [s]");
        desc.emplace_back("Receiver clock drift [s/s]");
        desc.emplace_back("X-ECEF StDev [m]");
        desc.emplace_back("Y-ECEF StDev [m]");
        desc.emplace_back("Z-ECEF StDev [m]");
        desc.emplace_back("XY-ECEF StDev [m]");
        desc.emplace_back("XZ-ECEF StDev [m]");
        desc.emplace_back("YZ-ECEF StDev [m]");
        desc.emplace_back("North StDev [m]");
        desc.emplace_back("East StDev [m]");
        desc.emplace_back("Down StDev [m]");
        desc.emplace_back("NE StDev [m]");
        desc.emplace_back("ND StDev [m]");
        desc.emplace_back("ED StDev [m]");
        desc.emplace_back("X velocity ECEF StDev [m/s]");
        desc.emplace_back("Y velocity ECEF StDev [m/s]");
        desc.emplace_back("Z velocity ECEF StDev [m/s]");
        desc.emplace_back("XY velocity StDev [m]");
        desc.emplace_back("XZ velocity StDev [m]");
        desc.emplace_back("YZ velocity StDev [m]");
        desc.emplace_back("North velocity StDev [m/s]");
        desc.emplace_back("East velocity StDev [m/s]");
        desc.emplace_back("Down velocity StDev [m/s]");
        desc.emplace_back("NE velocity StDev [m]");
        desc.emplace_back("ND velocity StDev [m]");
        desc.emplace_back("ED velocity StDev [m]");
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
    [[nodiscard]] static constexpr size_t GetDescriptorCount() { return 73; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> dataDescriptors() const override { return GetDataDescriptors(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetDescriptorCount());
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
        case 8:  // Velocity norm [m/s]
        case 9:  // X velocity ECEF [m/s]
        case 10: // Y velocity ECEF [m/s]
        case 11: // Z velocity ECEF [m/s]
        case 12: // North velocity [m/s]
        case 13: // East velocity [m/s]
        case 14: // Down velocity [m/s]
            return PosVel::getValueAt(idx);
        case 15: // Number satellites
            return static_cast<double>(nSatellites);
        case 16: // Receiver clock bias [s]
            return recvClk.bias.value;
        case 17: // Receiver clock drift [s/s]
            if (recvClk.drift.value != 0.0) { return recvClk.drift.value; }
            break;
        case 18: // X-ECEF StDev [m]
            return e_positionStdev()(0);
        case 19: // Y-ECEF StDev [m]
            return e_positionStdev()(1);
        case 20: // Z-ECEF StDev [m]
            return e_positionStdev()(2);
        case 21: // XY-ECEF StDev [m]
            if (e_CovarianceMatrix().has_value()) { return (*e_CovarianceMatrix())(SPP::States::PosX, SPP::States::PosY); }
            break;
        case 22: // XZ-ECEF StDev [m]
            if (e_CovarianceMatrix().has_value()) { return (*e_CovarianceMatrix())(SPP::States::PosX, SPP::States::PosZ); }
            break;
        case 23: // YZ-ECEF StDev [m]
            if (e_CovarianceMatrix().has_value()) { return (*e_CovarianceMatrix())(SPP::States::PosY, SPP::States::PosZ); }
            break;
        case 24: // North StDev [m]
            return n_positionStdev()(0);
        case 25: // East StDev [m]
            return n_positionStdev()(1);
        case 26: // Down StDev [m]
            return n_positionStdev()(2);
        case 27: // NE StDev [m]
            if (n_CovarianceMatrix().has_value()) { return (*n_CovarianceMatrix())(SPP::States::PosX, SPP::States::PosY); }
            break;
        case 28: // ND StDev [m]
            if (n_CovarianceMatrix().has_value()) { return (*n_CovarianceMatrix())(SPP::States::PosX, SPP::States::PosZ); }
            break;
        case 29: // ED StDev [m]
            if (n_CovarianceMatrix().has_value()) { return (*n_CovarianceMatrix())(SPP::States::PosY, SPP::States::PosZ); }
            break;
        case 30: // X velocity ECEF StDev [m/s]
            return e_velocityStdev()(0);
        case 31: // Y velocity ECEF StDev [m/s]
            return e_velocityStdev()(1);
        case 32: // Z velocity ECEF StDev [m/s]
            return e_velocityStdev()(2);
        case 33: // XY velocity StDev [m]
            if (e_CovarianceMatrix().has_value() && (*e_CovarianceMatrix()).get().hasAnyCols(SPP::States::Vel)) { return (*e_CovarianceMatrix())(SPP::States::VelX, SPP::States::VelY); }
            break;
        case 34: // XZ velocity StDev [m]
            if (e_CovarianceMatrix().has_value() && (*e_CovarianceMatrix()).get().hasAnyCols(SPP::States::Vel)) { return (*e_CovarianceMatrix())(SPP::States::VelX, SPP::States::VelZ); }
            break;
        case 35: // YZ velocity StDev [m]
            if (e_CovarianceMatrix().has_value() && (*e_CovarianceMatrix()).get().hasAnyCols(SPP::States::Vel)) { return (*e_CovarianceMatrix())(SPP::States::VelY, SPP::States::VelZ); }
            break;
        case 36: // North velocity StDev [m/s]
            return n_velocityStdev()(0);
        case 37: // East velocity StDev [m/s]
            return n_velocityStdev()(1);
        case 38: // Down velocity StDev [m/s]
            return n_velocityStdev()(2);
        case 39: // NE velocity StDev [m]
            if (n_CovarianceMatrix().has_value() && (*n_CovarianceMatrix()).get().hasAnyCols(SPP::States::Vel)) { return (*n_CovarianceMatrix())(SPP::States::VelX, SPP::States::VelY); }
            break;
        case 40: // ND velocity StDev [m]
            if (n_CovarianceMatrix().has_value() && (*n_CovarianceMatrix()).get().hasAnyCols(SPP::States::Vel)) { return (*n_CovarianceMatrix())(SPP::States::VelX, SPP::States::VelZ); }
            break;
        case 41: // ED velocity StDev [m]
            if (n_CovarianceMatrix().has_value() && (*n_CovarianceMatrix()).get().hasAnyCols(SPP::States::Vel)) { return (*n_CovarianceMatrix())(SPP::States::VelY, SPP::States::VelZ); }
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

    /// Satellite specific data
    struct SatData
    {
        double satElevation = 0.0; ///< Satellite Elevation [rad]
        double satAzimuth = 0.0;   ///< Satellite Azimuth [rad]
    };

    /// Extended data for each satellite
    std::vector<std::pair<SatId, SatData>> satData;

    // ------------------------------------------------------------- Getter ----------------------------------------------------------------

    /// Returns the standard deviation of the position in ECEF frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& e_positionStdev() const { return _e_positionStdev; }

    /// Returns the standard deviation of the position in local navigation frame coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& n_positionStdev() const { return _n_positionStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Vector3d& e_velocityStdev() const { return _e_velocityStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& n_velocityStdev() const { return _n_velocityStdev; }

    /// Returns the Covariance matrix in ECEF frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>>> e_CovarianceMatrix() const { return _e_covarianceMatrix; }

    /// Returns the Covariance matrix in local navigation frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>>> n_CovarianceMatrix() const { return _n_covarianceMatrix; }

    // ------------------------------------------------------------- Setter ----------------------------------------------------------------

    /// @brief Set the Position in ECEF coordinates and its standard deviation
    /// @param[in] e_position New Position in ECEF coordinates [m]
    /// @param[in] e_PositionCovarianceMatrix Standard deviation of Position in ECEF coordinates [m]
    void setPositionAndStdDev_e(const Eigen::Vector3d& e_position, const Eigen::Matrix3d& e_PositionCovarianceMatrix)
    {
        setPosition_e(e_position);
        _e_positionStdev = e_PositionCovarianceMatrix.diagonal().cwiseSqrt();
        _n_positionStdev = (n_Quat_e() * e_PositionCovarianceMatrix * e_Quat_n()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Velocity in ECEF coordinates and its standard deviation
    /// @param[in] e_velocity New Velocity in ECEF coordinates [m/s]
    /// @param[in] e_velocityCovarianceMatrix Covariance matrix of Velocity in earth coordinates [m/s]
    void setVelocityAndStdDev_e(const Eigen::Vector3d& e_velocity, const Eigen::Matrix3d& e_velocityCovarianceMatrix)
    {
        setVelocity_e(e_velocity);
        _e_velocityStdev = e_velocityCovarianceMatrix.diagonal().cwiseSqrt();
        _n_velocityStdev = (n_Quat_e() * e_velocityCovarianceMatrix * e_Quat_n()).diagonal().cwiseSqrt();
    }

    /// @brief Set the Covariance matrix in ECEF coordinates
    /// @param[in] e_covarianceMatrix Kalman Filter or Least Squares error variance
    /// @attention Position has to be set before calling this
    void setCovarianceMatrix(const KeyedMatrixXd<SPP::States::StateKeyTypes, SPP::States::StateKeyTypes>& e_covarianceMatrix)
    {
        _e_covarianceMatrix = e_covarianceMatrix;
        _n_covarianceMatrix = _e_covarianceMatrix;

        Eigen::Vector3d lla_pos = lla_position();
        Eigen::Quaterniond n_Quat_e = trafo::n_Quat_e(lla_pos(0), lla_pos(1));
        Eigen::Quaterniond e_Quat_n = trafo::e_Quat_n(lla_pos(0), lla_pos(1));

        if (e_covarianceMatrix.hasCols(SPP::States::Vel))
        {
            (*_n_covarianceMatrix)(SPP::States::PosVel, all).setZero();
            (*_n_covarianceMatrix)(all, SPP::States::PosVel).setZero();
            (*_n_covarianceMatrix)(SPP::States::Vel, SPP::States::Vel) = n_Quat_e * (*_e_covarianceMatrix)(SPP::States::Vel, SPP::States::Vel) * e_Quat_n;
        }
        else
        {
            (*_n_covarianceMatrix)(SPP::States::Pos, all).setZero();
            (*_n_covarianceMatrix)(all, SPP::States::Pos).setZero();
        }
        (*_n_covarianceMatrix)(SPP::States::Pos, SPP::States::Pos) = n_Quat_e * (*_e_covarianceMatrix)(SPP::States::Pos, SPP::States::Pos) * e_Quat_n;
    }

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
