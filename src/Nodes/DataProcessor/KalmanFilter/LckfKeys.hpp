// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LckfKeys.hpp
/// @brief Keys for the Loosely Coupled Kalman Filter (LCKF)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2025-09-29

#pragma once

#include <cstdint>
#include <vector>
#include <iostream>
#include <fmt/core.h>
#include "util/Container/KeyedMatrix.hpp"

namespace NAV::LckfKeys
{

/// @brief State Keys of the Kalman filter
enum KFStates : uint8_t
{
    Roll = 0,            ///< Roll
    Pitch = 1,           ///< Pitch
    Yaw = 2,             ///< Yaw
    VelN = 3,            ///< Velocity North
    VelE = 4,            ///< Velocity East
    VelD = 5,            ///< Velocity Down
    PosLat = 6,          ///< Latitude
    PosLon = 7,          ///< Longitude
    PosAlt = 8,          ///< Altitude
    AccBiasX = 9,        ///< Accelerometer Bias X
    AccBiasY = 10,       ///< Accelerometer Bias Y
    AccBiasZ = 11,       ///< Accelerometer Bias Z
    GyrBiasX = 12,       ///< Gyroscope Bias X
    GyrBiasY = 13,       ///< Gyroscope Bias Y
    GyrBiasZ = 14,       ///< Gyroscope Bias Z
    HeightBias = 15,     ///< Baro Height Bias
    HeightScale = 16,    ///< Baro Height Scale
    KFStates_COUNT = 17, ///< Number of states

    Psi_eb_1 = Roll,  ///< Angle between Earth and Body frame around 1. axis
    Psi_eb_2 = Pitch, ///< Angle between Earth and Body frame around 2. axis
    Psi_eb_3 = Yaw,   ///< Angle between Earth and Body frame around 3. axis
    VelX = VelN,      ///< ECEF Velocity X
    VelY = VelE,      ///< ECEF Velocity Y
    VelZ = VelD,      ///< ECEF Velocity Z
    PosX = PosLat,    ///< ECEF Position X
    PosY = PosLon,    ///< ECEF Position Y
    PosZ = PosAlt,    ///< ECEF Position Z
};

/// @brief Measurement Keys of the Kalman filter
enum KFMeas : uint8_t
{
    dPosLat = 0, ///< Latitude difference
    dPosLon = 1, ///< Longitude difference
    dPosAlt = 2, ///< Altitude difference
    dVelN = 3,   ///< Velocity North difference
    dVelE = 4,   ///< Velocity East difference
    dVelD = 5,   ///< Velocity Down difference
    dHgt = 6,    ///< height difference

    dPosX = dPosLat, ///< ECEF Position X difference
    dPosY = dPosLon, ///< ECEF Position Y difference
    dPosZ = dPosAlt, ///< ECEF Position Z difference
    dVelX = dVelN,   ///< ECEF Velocity X difference
    dVelY = dVelE,   ///< ECEF Velocity Y difference
    dVelZ = dVelD,   ///< ECEF Velocity Z difference
};

/// @brief All position keys
inline static const std::vector<KFStates> KFPos = { KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt };
/// @brief All velocity keys
inline static const std::vector<KFStates> KFVel = { KFStates::VelN, KFStates::VelE, KFStates::VelD };
/// @brief All attitude keys
inline static const std::vector<KFStates> KFAtt = { KFStates::Roll, KFStates::Pitch, KFStates::Yaw };
/// @brief All acceleration bias keys
inline static const std::vector<KFStates> KFAccBias = { KFStates::AccBiasX, KFStates::AccBiasY, KFStates::AccBiasZ };
/// @brief All gyroscope bias keys
inline static const std::vector<KFStates> KFGyrBias = { KFStates::GyrBiasX, KFStates::GyrBiasY, KFStates::GyrBiasZ };

/// @brief All position and velocity keys
inline static const std::vector<KFStates> KFPosVel = { KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt,
                                                       KFStates::VelN, KFStates::VelE, KFStates::VelD };
/// @brief All position, velocity and attitude keys
inline static const std::vector<KFStates> KFPosVelAtt = { KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt,
                                                          KFStates::VelN, KFStates::VelE, KFStates::VelD,
                                                          KFStates::Roll, KFStates::Pitch, KFStates::Yaw };

/// @brief Vector with all state keys
inline static const std::vector<KFStates> States = { KFStates::Roll, KFStates::Pitch, KFStates::Yaw,
                                                     KFStates::VelN, KFStates::VelE, KFStates::VelD,
                                                     KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt,
                                                     KFStates::AccBiasX, KFStates::AccBiasY, KFStates::AccBiasZ,
                                                     KFStates::GyrBiasX, KFStates::GyrBiasY, KFStates::GyrBiasZ,
                                                     KFStates::HeightBias, KFStates::HeightScale };

/// @brief Vector with all state keys except the attitude keys
inline static const std::vector<KFStates> RestStates = { KFStates::VelN, KFStates::VelE, KFStates::VelD,
                                                         KFStates::PosLat, KFStates::PosLon, KFStates::PosAlt,
                                                         KFStates::AccBiasX, KFStates::AccBiasY, KFStates::AccBiasZ,
                                                         KFStates::GyrBiasX, KFStates::GyrBiasY, KFStates::GyrBiasZ,
                                                         KFStates::HeightBias, KFStates::HeightScale };

/// @brief Vector with PosVel measurement keys
inline static const std::vector<KFMeas> Meas = { KFMeas::dPosLat, KFMeas::dPosLon, KFMeas::dPosAlt, KFMeas::dVelN, KFMeas::dVelE, KFMeas::dVelD };
/// @brief All position difference keys
inline static const std::vector<KFMeas> dPos = { KFMeas::dPosLat, KFMeas::dPosLon, KFMeas::dPosAlt };
/// @brief All velocity difference keys
inline static const std::vector<KFMeas> dVel = { KFMeas::dVelN, KFMeas::dVelE, KFMeas::dVelD };

/// @brief State Keys of the Kalman filter with attitude quaternion (instead of RPY)
enum KFStatesQuat : uint8_t
{
    QuatW = 0,         ///< Attitude quaternion w component
    QuatX = 1,         ///< Attitude quaternion x component
    QuatY = 2,         ///< Attitude quaternion y component
    QuatZ = 3,         ///< Attitude quaternion z component
    QVelN = 4,         ///< Velocity North
    QVelE = 5,         ///< Velocity East
    QVelD = 6,         ///< Velocity Down
    QPosLat = 7,       ///< Latitude
    QPosLon = 8,       ///< Longitude
    QPosAlt = 9,       ///< Altitude
    QAccBiasX = 10,    ///< Accelerometer Bias X
    QAccBiasY = 11,    ///< Accelerometer Bias Y
    QAccBiasZ = 12,    ///< Accelerometer Bias Z
    QGyrBiasX = 13,    ///< Gyroscope Bias X
    QGyrBiasY = 14,    ///< Gyroscope Bias Y
    QGyrBiasZ = 15,    ///< Gyroscope Bias Z
    QHeightBias = 16,  ///< Baro Height Bias
    QHeightScale = 17, ///< Baro Height Scale
    KFQuat_COUNT = 18, ///< Number of state keys
};

/// @brief Vector with all state keys with attitude quaternion
inline static const std::vector<KFStatesQuat> StatesQuat = {
    KFStatesQuat::QuatW, KFStatesQuat::QuatX, KFStatesQuat::QuatY, KFStatesQuat::QuatZ,
    KFStatesQuat::QVelN, KFStatesQuat::QVelE, KFStatesQuat::QVelD,
    KFStatesQuat::QPosLat, KFStatesQuat::QPosLon, KFStatesQuat::QPosAlt,
    KFStatesQuat::QAccBiasX, KFStatesQuat::QAccBiasY, KFStatesQuat::QAccBiasZ,
    KFStatesQuat::QGyrBiasX, KFStatesQuat::QGyrBiasY, KFStatesQuat::QGyrBiasZ,
    KFStatesQuat::QHeightBias, KFStatesQuat::QHeightScale
};

using StateVector = KeyedVector<double, KFStates, KFStates_COUNT>;                                   ///< Keyed vector type for the RPY state
using StateVectorQuat = KeyedVector<double, KFStatesQuat, KFQuat_COUNT>;                             ///< Keyed vector type for the quaternion state
using StateMatrix = KeyedMatrix<double, KFStates, KFStates, KFStates_COUNT, KFStates_COUNT>;         ///< Keyed matrix type for the RPY state
using StateMatrixQuat = KeyedMatrix<double, KFStatesQuat, KFStatesQuat, KFQuat_COUNT, KFQuat_COUNT>; ///< Keyed matrix type for the quaternion state

/// @brief All position keys with attitude quaternion
inline static const std::vector<KFStatesQuat> KFPosQuat = { KFStatesQuat::QPosLat, KFStatesQuat::QPosLon, KFStatesQuat::QPosAlt };
/// @brief All velocity keys with attitude quaternion
inline static const std::vector<KFStatesQuat> KFVelQuat = { KFStatesQuat::QVelN, KFStatesQuat::QVelE, KFStatesQuat::QVelD };
/// @brief All attitude keys with attitude quaternion
inline static const std::vector<KFStatesQuat> KFAttQuat = { KFStatesQuat::QuatW, KFStatesQuat::QuatX, KFStatesQuat::QuatY, KFStatesQuat::QuatZ };
/// @brief All acceleration bias keys with attitude quaternion
inline static const std::vector<KFStatesQuat> KFAccBiasQuat = { KFStatesQuat::QAccBiasX, KFStatesQuat::QAccBiasY, KFStatesQuat::QAccBiasZ };
/// @brief All gyroscope bias keys with attitude quaternion
inline static const std::vector<KFStatesQuat> KFGyrBiasQuat = { KFStatesQuat::QGyrBiasX, KFStatesQuat::QGyrBiasY, KFStatesQuat::QGyrBiasZ };

/// @brief All position and velocity keys
inline static const std::vector<KFStatesQuat> KFPosVelQuat = { KFStatesQuat::QPosLat, KFStatesQuat::QPosLon, KFStatesQuat::QPosAlt,
                                                               KFStatesQuat::QVelN, KFStatesQuat::QVelE, KFStatesQuat::QVelD };
/// @brief All position, velocity and attitude keys
inline static const std::vector<KFStatesQuat> KFPosVelAttQuat = { KFStatesQuat::QPosLat, KFStatesQuat::QPosLon, KFStatesQuat::QPosAlt,
                                                                  KFStatesQuat::QVelN, KFStatesQuat::QVelE, KFStatesQuat::QVelD,
                                                                  KFStatesQuat::QuatW, KFStatesQuat::QuatX, KFStatesQuat::QuatY, KFStatesQuat::QuatZ };

/// @brief Vector with all state keys, except the attitude quaternion keys
inline static const std::vector<KFStatesQuat> KFRestQuat = {
    KFStatesQuat::QVelN, KFStatesQuat::QVelE, KFStatesQuat::QVelD,
    KFStatesQuat::QPosLat, KFStatesQuat::QPosLon, KFStatesQuat::QPosAlt,
    KFStatesQuat::QAccBiasX, KFStatesQuat::QAccBiasY, KFStatesQuat::QAccBiasZ,
    KFStatesQuat::QGyrBiasX, KFStatesQuat::QGyrBiasY, KFStatesQuat::QGyrBiasZ,
    KFStatesQuat::QHeightBias, KFStatesQuat::QHeightScale
};

} // namespace NAV::LckfKeys

#ifndef DOXYGEN_IGNORE

template<>
struct fmt::formatter<NAV::LckfKeys::KFStates> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] st Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::LckfKeys::KFStates& st, FormatContext& ctx) const
    {
        switch (st)
        {
        case NAV::LckfKeys::KFStates::Roll:
            return fmt::formatter<const char*>::format("Roll/Psi_eb_1", ctx);
        case NAV::LckfKeys::KFStates::Pitch:
            return fmt::formatter<const char*>::format("Pitch/Psi_eb_2", ctx);
        case NAV::LckfKeys::KFStates::Yaw:
            return fmt::formatter<const char*>::format("Yaw/Psi_eb_3", ctx);
        case NAV::LckfKeys::KFStates::VelN:
            return fmt::formatter<const char*>::format("VelN/VelX", ctx);
        case NAV::LckfKeys::KFStates::VelE:
            return fmt::formatter<const char*>::format("VelE/VelY", ctx);
        case NAV::LckfKeys::KFStates::VelD:
            return fmt::formatter<const char*>::format("VelD/VelZ", ctx);
        case NAV::LckfKeys::KFStates::PosLat:
            return fmt::formatter<const char*>::format("PosLat/PosX", ctx);
        case NAV::LckfKeys::KFStates::PosLon:
            return fmt::formatter<const char*>::format("PosLon/PosY", ctx);
        case NAV::LckfKeys::KFStates::PosAlt:
            return fmt::formatter<const char*>::format("PosAlt/PosZ", ctx);
        case NAV::LckfKeys::KFStates::AccBiasX:
            return fmt::formatter<const char*>::format("AccBiasX", ctx);
        case NAV::LckfKeys::KFStates::AccBiasY:
            return fmt::formatter<const char*>::format("AccBiasY", ctx);
        case NAV::LckfKeys::KFStates::AccBiasZ:
            return fmt::formatter<const char*>::format("AccBiasZ", ctx);
        case NAV::LckfKeys::KFStates::GyrBiasX:
            return fmt::formatter<const char*>::format("GyrBiasX", ctx);
        case NAV::LckfKeys::KFStates::GyrBiasY:
            return fmt::formatter<const char*>::format("GyrBiasY", ctx);
        case NAV::LckfKeys::KFStates::GyrBiasZ:
            return fmt::formatter<const char*>::format("GyrBiasZ", ctx);
        case NAV::LckfKeys::KFStates::HeightBias:
            return fmt::formatter<const char*>::format("HeightBias", ctx);
        case NAV::LckfKeys::KFStates::HeightScale:
            return fmt::formatter<const char*>::format("HeightScale", ctx);
        case NAV::LckfKeys::KFStates::KFStates_COUNT:
            return fmt::formatter<const char*>::format("COUNT", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};
template<>
struct fmt::formatter<NAV::LckfKeys::KFMeas> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] st Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::LckfKeys::KFMeas& st, FormatContext& ctx) const
    {
        switch (st)
        {
        case NAV::LckfKeys::KFMeas::dPosLat:
            return fmt::formatter<const char*>::format("dPosLat/dPosX", ctx);
        case NAV::LckfKeys::KFMeas::dPosLon:
            return fmt::formatter<const char*>::format("dPosLon/dPosY", ctx);
        case NAV::LckfKeys::KFMeas::dPosAlt:
            return fmt::formatter<const char*>::format("dPosAlt/dPosZ", ctx);
        case NAV::LckfKeys::KFMeas::dVelN:
            return fmt::formatter<const char*>::format("dVelN/dVelX", ctx);
        case NAV::LckfKeys::KFMeas::dVelE:
            return fmt::formatter<const char*>::format("dVelE/dVelY", ctx);
        case NAV::LckfKeys::KFMeas::dVelD:
            return fmt::formatter<const char*>::format("dVelD/dVelZ", ctx);
        case NAV::LckfKeys::KFMeas::dHgt:
            return fmt::formatter<const char*>::format("dHgt", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

#endif

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::LckfKeys::KFStates& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::LckfKeys::KFMeas& obj);