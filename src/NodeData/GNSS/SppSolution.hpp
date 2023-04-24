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
#include <algorithm>
#include "util/Assert.h"
#include "NodeData/State/PosVel.hpp"

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/ReceiverClock.hpp"

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

    // --------------------------------------------------------- Public Members ------------------------------------------------------------

    /// Amount of satellites used for the position calculation
    size_t nSatellitesPosition = 0;
    /// Amount of satellites used for the velocity calculation
    size_t nSatellitesVelocity = 0;

    /// Estimated receiver clock parameter
    ReceiverClock recvClk = { .bias = { std::nan(""), std::nan("") },
                              .drift = { std::nan(""), std::nan("") },
                              .sysTimeDiff = {},
                              .sysDriftDiff = {} };

    // ------------------------------------------------------------- Getter ----------------------------------------------------------------

    /// Returns the standard deviation of the position in ECEF frame coordinates in [m]
    [[nodiscard]] const Eigen::Matrix3d& e_positionStdev() const { return _e_positionStdev; }

    /// Returns the standard deviation of the position in local navigation frame coordinates in [m]
    [[nodiscard]] const Eigen::Matrix3d& n_positionStdev() const { return _n_positionStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Matrix3d& e_velocityStdev() const { return _e_velocityStdev; }

    /// Returns the standard deviation of the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Matrix3d& n_velocityStdev() const { return _n_velocityStdev; }

    // ------------------------------------------------------------- Setter ----------------------------------------------------------------

    /// @brief Set the Position in ECEF coordinates and its standard deviation
    /// @param[in] e_position New Position in ECEF coordinates [m]
    /// @param[in] e_positionStdev Standard deviation of Position in ECEF coordinates [m]
    void setPositionAndStdDev_e(const Eigen::Vector3d& e_position, const Eigen::Matrix3d& e_positionStdev)
    {
        setPosition_e(e_position);
        _e_positionStdev = e_positionStdev;
        _n_positionStdev = (n_Quat_e().toRotationMatrix() * _e_positionStdev * n_Quat_e().conjugate().toRotationMatrix()).cwiseAbs();
    }

    /// @brief Set the Velocity in ECEF coordinates and its standard deviation
    /// @param[in] e_velocity New Velocity in ECEF coordinates [m/s]
    /// @param[in] e_velocityStdev Standard deviation of Velocity in earth coordinates [m/s]
    void setVelocityAndStdDev_e(const Eigen::Vector3d& e_velocity, const Eigen::Matrix3d& e_velocityStdev)
    {
        setVelocity_e(e_velocity);
        _e_velocityStdev = e_velocityStdev;
        _n_velocityStdev = n_Quat_e().toRotationMatrix() * _e_velocityStdev * n_Quat_e().conjugate().toRotationMatrix();
    }

  private:
    /// Standard deviation of Position in ECEF coordinates [m]
    Eigen::Matrix3d _e_positionStdev = Eigen::Matrix3d::Zero() * std::nan("");
    /// Standard deviation of Position in local navigation frame coordinates [m]
    Eigen::Matrix3d _n_positionStdev = Eigen::Matrix3d::Zero() * std::nan("");
    /// Standard deviation of Velocity in earth coordinates [m/s]
    Eigen::Matrix3d _e_velocityStdev = Eigen::Matrix3d::Zero() * std::nan("");
    /// Standard deviation of Velocity in navigation coordinates [m/s]
    Eigen::Matrix3d _n_velocityStdev = Eigen::Matrix3d::Zero() * std::nan("");
};

#ifdef TESTING

/// SPP Algorithm extended output
class SppSolutionExtended : public SppSolution
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "SppSolutionExtended";
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        auto parent = SppSolution::parentTypes();
        parent.push_back(SppSolution::type());
        return parent;
    }

    /// Extended data structure
    struct ExtendedData
    {
        /// @brief Constructor
        /// @param[in] satSigId Satellite signal identifier (frequency and satellite number)
        /// @param[in] code Signal code
        ExtendedData(const SatSigId& satSigId, const Code code) : satSigId(satSigId), code(code) {}

        SatSigId satSigId = { Freq_None, 0 }; ///< Frequency and satellite number
        Code code;                            ///< GNSS Code
        InsTime transmitTime{};               ///< Time when the signal was transmitted
        Eigen::Vector3d e_satPos;             ///< Satellite position in ECEF frame coordinates [m]
        Eigen::Vector3d e_satVel;             ///< Satellite velocity in ECEF frame coordinates [m/s]
        double satClkBias{ 0.0 };             ///< Satellite clock bias [s]
        double satClkDrift{ 0.0 };            ///< Satellite clock drift [s/s]
        double satElevation{ std::nan("") };  ///< Elevation [rad]
        double satAzimuth{ std::nan("") };    ///< Azimuth [rad]
        bool skipped = false;                 ///< Bool to check whether the observation was skipped (signal unhealthy)
        bool elevationMaskTriggered = false;  ///< Bool to check whether the elevation mask was triggered

        double pseudorangeRate{ std::nan("") }; ///< Pseudorange rate [m/s]
        double dpsr_I{ std::nan("") };          ///< Estimated ionosphere propagation error [m]
        double dpsr_T{ std::nan("") };          ///< Estimated troposphere propagation error [m]
        double geometricDist{ std::nan("") };   ///< Geometric distance [m]
    };

    /// Extended data for testing
    std::vector<ExtendedData> extData;

    /// @brief Return the element with the identifier or a newly constructed one if it did not exist
    /// @param[in] freq Signal frequency (also identifies the satellite system)
    /// @param[in] satNum Number of the satellite
    /// @param[in] code Signal code
    /// @return The element found in the observations or a newly constructed one
    ExtendedData& operator()(const Frequency& freq, uint16_t satNum, Code code)
    {
        auto iter = std::find_if(extData.begin(), extData.end(), [freq, satNum, code](const ExtendedData& idData) {
            return idData.satSigId.freq == freq && idData.satSigId.satNum == satNum && idData.code == code;
        });
        if (iter != extData.end())
        {
            return *iter;
        }

        extData.emplace_back(SatSigId{ freq, satNum }, code);
        return extData.back();
    }

    /// @brief Return the element with the identifier
    /// @param[in] freq Signal frequency (also identifies the satellite system)
    /// @param[in] satNum Number of the satellite
    /// @param[in] code Signal code
    /// @return The element found in the observations
    const ExtendedData& operator()(const Frequency& freq, uint16_t satNum, Code code) const
    {
        auto iter = std::find_if(extData.begin(), extData.end(), [freq, satNum, code](const ExtendedData& idData) {
            return idData.satSigId.freq == freq && idData.satSigId.satNum == satNum && idData.code == code;
        });

        INS_ASSERT_USER_ERROR(iter != extData.end(), "You can not insert new elements in a const context.");
        return *iter;
    }
};

#endif

} // namespace NAV
