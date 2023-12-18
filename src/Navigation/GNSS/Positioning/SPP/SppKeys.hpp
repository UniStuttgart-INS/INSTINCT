// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppKeys.hpp
/// @brief Keys for SPP with keyed matrices
/// @author P. Peitschat (HiWi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-26

#pragma once

#include <variant>

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"

namespace NAV::GNSS::Positioning::SPP
{
namespace States
{
/// @brief State Keys of the SPP
enum SppStates
{
    PosX,            ///< Position ECEF_X [m]
    PosY,            ///< Position ECEF_Y [m]
    PosZ,            ///< Position ECEF_Z [m]
    VelX,            ///< Velocity ECEF_X [m/s]
    VelY,            ///< Velocity ECEF_Y [m/s]
    VelZ,            ///< Velocity ECEF_Z [m/s]
    RecvClkErr,      ///< Receiver clock error [m]
    RecvClkDrift,    ///< Receiver clock drift [m/s]
    SppStates_COUNT, ///< Count
};
/// @brief Inter-system clock errors (one for additional satellite system)
struct InterSysErr
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const InterSysErr& rhs) const { return satSys == rhs.satSys; }
    /// @brief Satellite system
    SatelliteSystem satSys;
};
/// @brief Inter-system clock drifts (one for additional satellite system)
struct InterSysDrift
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const InterSysDrift& rhs) const { return satSys == rhs.satSys; }
    /// @brief Satellite system
    SatelliteSystem satSys;
};

/// Alias for the state key type
using StateKeyTypes = std::variant<SppStates, InterSysErr, InterSysDrift>;
/// @brief All position keys
inline static const std::vector<StateKeyTypes> Pos = { SppStates::PosX, SppStates::PosY, SppStates::PosZ };
/// @brief All velocity keys
inline static const std::vector<StateKeyTypes> Vel = { SppStates::VelX, SppStates::VelY, SppStates::VelZ };
/// @brief All receiver clock keys
inline static const std::vector<StateKeyTypes> RecvClk = { SppStates::RecvClkErr, SppStates::RecvClkDrift };
/// @brief Vector with all position and velocity state keys
inline static const std::vector<StateKeyTypes> PosVel = { SppStates::PosX, SppStates::PosY, SppStates::PosZ,
                                                          SppStates::VelX, SppStates::VelY, SppStates::VelZ };
/// @brief Vector with all position, velocity and receiver clock state keys
inline static const std::vector<StateKeyTypes> PosVelRecvClk = { SppStates::PosX, SppStates::PosY, SppStates::PosZ,
                                                                 SppStates::VelX, SppStates::VelY, SppStates::VelZ,
                                                                 SppStates::RecvClkErr, SppStates::RecvClkDrift };

/// @brief Vector with all position keys and receiver clock error key (for LSE from pseudoranges)
inline static const std::vector<StateKeyTypes> PosRecvClkErr = { SppStates::PosX, SppStates::PosY, SppStates::PosZ,
                                                                 SppStates::RecvClkErr };
/// @brief Vector with all velocity keys and receiver clock drift key (for LSE from pseudorange-rates)
inline static const std::vector<StateKeyTypes> VelRecvClkDrift = { SppStates::VelX, SppStates::VelY, SppStates::VelZ,
                                                                   SppStates::RecvClkDrift };

} // namespace States

namespace Meas
{

/// @brief Pseudorange measurement [m]
struct Psr
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const Psr& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};
/// @brief Range-rate (doppler) measurement [m/s]
struct Doppler
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const Doppler& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the measurement key type
using MeasKeyTypes = std::variant<Psr, Doppler>;

} // namespace Meas
} // namespace NAV::GNSS::Positioning::SPP

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::States::InterSysErr>
{
    /// @brief Hash function
    /// @param[in] interSysErr Inter-system clock errors
    size_t operator()(const NAV::GNSS::Positioning::SPP::States::InterSysErr& interSysErr) const
    {
        return NAV::GNSS::Positioning::SPP::States::SppStates_COUNT + std::hash<NAV::SatelliteSystem>()(interSysErr.satSys);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::States::InterSysDrift>
{
    /// @brief Hash function
    /// @param[in] interSysDrift Inter-system clock drifts
    size_t operator()(const NAV::GNSS::Positioning::SPP::States::InterSysDrift& interSysDrift) const
    {
        return NAV::GNSS::Positioning::SPP::States::SppStates_COUNT + std::hash<NAV::SatelliteSystem>()(interSysDrift.satSys);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::Meas::Psr>
{
    /// @brief Hash function
    /// @param[in] psr Pseudorange observation
    size_t operator()(const NAV::GNSS::Positioning::SPP::Meas::Psr& psr) const
    {
        return std::hash<NAV::SatSigId>()(psr.satSigId);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::GNSS::Positioning::SPP::Meas::Doppler>
{
    /// @brief Hash function
    /// @param[in] doppler Doppler observation
    size_t operator()(const NAV::GNSS::Positioning::SPP::Meas::Doppler& doppler) const
    {
        return std::hash<NAV::SatSigId>()(doppler.satSigId) << 12;
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::SppStates> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::States::SppStates& state, FormatContext& ctx)
    {
        using namespace NAV::GNSS::Positioning::SPP::States; // NOLINT(google-build-using-namespace)

        switch (state)
        {
        case PosX:
            return fmt::formatter<const char*>::format("PosX", ctx);
        case PosY:
            return fmt::formatter<const char*>::format("PosY", ctx);
        case PosZ:
            return fmt::formatter<const char*>::format("PosZ", ctx);
        case VelX:
            return fmt::formatter<const char*>::format("VelX", ctx);
        case VelY:
            return fmt::formatter<const char*>::format("VelY", ctx);
        case VelZ:
            return fmt::formatter<const char*>::format("VelZ", ctx);
        case RecvClkErr:
            return fmt::formatter<const char*>::format("RecvClkErr", ctx);
        case RecvClkDrift:
            return fmt::formatter<const char*>::format("RecvClkDrift", ctx);
        case SppStates_COUNT:
            return fmt::formatter<const char*>::format("SppStates_COUNT", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::InterSysErr> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] interSysErr Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::States::InterSysErr& interSysErr, FormatContext& ctx)
    {
        return fmt::formatter<std::string>::format(fmt::format("InterSysErr({})", interSysErr.satSys), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::InterSysDrift> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] interSysDrift Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::States::InterSysDrift& interSysDrift, FormatContext& ctx)
    {
        return fmt::formatter<std::string>::format(fmt::format("InterSysDrift({})", interSysDrift.satSys), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::Meas::Psr> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] psr Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::Meas::Psr& psr, FormatContext& ctx)
    {
        return fmt::formatter<std::string>::format(fmt::format("psr({})", psr.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::Meas::Doppler> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] doppler Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::Meas::Doppler& doppler, FormatContext& ctx)
    {
        return fmt::formatter<std::string>::format(fmt::format("dop({})", doppler.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::StateKeyTypes> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::States::StateKeyTypes& state, FormatContext& ctx)
    {
        using namespace NAV::GNSS::Positioning::SPP::States; // NOLINT(google-build-using-namespace)

        if (const auto* s = std::get_if<NAV::GNSS::Positioning::SPP::States::SppStates>(&state))
        {
            switch (*s)
            {
            case PosX:
                return fmt::formatter<std::string>::format("PosX", ctx);
            case PosY:
                return fmt::formatter<std::string>::format("PosY", ctx);
            case PosZ:
                return fmt::formatter<std::string>::format("PosZ", ctx);
            case VelX:
                return fmt::formatter<std::string>::format("VelX", ctx);
            case VelY:
                return fmt::formatter<std::string>::format("VelY", ctx);
            case VelZ:
                return fmt::formatter<std::string>::format("VelZ", ctx);
            case RecvClkErr:
                return fmt::formatter<std::string>::format("RecvClkErr", ctx);
            case RecvClkDrift:
                return fmt::formatter<std::string>::format("RecvClkDrift", ctx);
            case SppStates_COUNT:
                return fmt::formatter<std::string>::format("SppStates_COUNT", ctx);
            }
        }
        if (const auto* interSysErr = std::get_if<NAV::GNSS::Positioning::SPP::States::InterSysErr>(&state))
        {
            return fmt::formatter<std::string>::format(fmt::format("InterSysErr({}))", interSysErr->satSys), ctx);
        }
        if (const auto* interSysDrift = std::get_if<NAV::GNSS::Positioning::SPP::States::InterSysDrift>(&state))
        {
            return fmt::formatter<std::string>::format(fmt::format("InterSysDrift({}))", interSysDrift->satSys), ctx);
        }

        return fmt::formatter<std::string>::format("ERROR", ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::Meas::MeasKeyTypes> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] meas Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::Meas::MeasKeyTypes& meas, FormatContext& ctx)
    {
        if (const auto* psr = std::get_if<NAV::GNSS::Positioning::SPP::Meas::Psr>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("psr({})", psr->satSigId), ctx);
        }
        if (const auto* doppler = std::get_if<NAV::GNSS::Positioning::SPP::Meas::Doppler>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("doppler({})", doppler->satSigId), ctx);
        }

        return fmt::formatter<std::string>::format("ERROR", ctx);
    }
};

#endif