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
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::SppStates>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

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
            return fmt::format_to(ctx.out(), "PosX");
        case PosY:
            return fmt::format_to(ctx.out(), "PosY");
        case PosZ:
            return fmt::format_to(ctx.out(), "PosZ");
        case VelX:
            return fmt::format_to(ctx.out(), "VelX");
        case VelY:
            return fmt::format_to(ctx.out(), "VelY");
        case VelZ:
            return fmt::format_to(ctx.out(), "VelZ");
        case RecvClkErr:
            return fmt::format_to(ctx.out(), "RecvClkErr");
        case RecvClkDrift:
            return fmt::format_to(ctx.out(), "RecvClkDrift");
        case SppStates_COUNT:
            return fmt::format_to(ctx.out(), "SppStates_COUNT");
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::InterSysErr>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] interSysErr Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::States::InterSysErr& interSysErr, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "InterSysErr({})", interSysErr.satSys);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::InterSysDrift>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] interSysDrift Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::States::InterSysDrift& interSysDrift, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "InterSysDrift({})", interSysDrift.satSys);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::Meas::Psr>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] psr Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::Meas::Psr& psr, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "psr({})", psr.satSigId);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::Meas::Doppler>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] doppler Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::Meas::Doppler& doppler, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "dop({})", doppler.satSigId);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::States::StateKeyTypes>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

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
                return fmt::format_to(ctx.out(), "PosX");
            case PosY:
                return fmt::format_to(ctx.out(), "PosY");
            case PosZ:
                return fmt::format_to(ctx.out(), "PosZ");
            case VelX:
                return fmt::format_to(ctx.out(), "VelX");
            case VelY:
                return fmt::format_to(ctx.out(), "VelY");
            case VelZ:
                return fmt::format_to(ctx.out(), "VelZ");
            case RecvClkErr:
                return fmt::format_to(ctx.out(), "RecvClkErr");
            case RecvClkDrift:
                return fmt::format_to(ctx.out(), "RecvClkDrift");
            case SppStates_COUNT:
                return fmt::format_to(ctx.out(), "SppStates_COUNT");
            }
        }
        if (const auto* interSysErr = std::get_if<NAV::GNSS::Positioning::SPP::States::InterSysErr>(&state))
        {
            return fmt::format_to(ctx.out(), "InterSysErr({})", interSysErr->satSys);
        }
        if (const auto* interSysDrift = std::get_if<NAV::GNSS::Positioning::SPP::States::InterSysDrift>(&state))
        {
            return fmt::format_to(ctx.out(), "InterSysDrift({})", interSysDrift->satSys);
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::GNSS::Positioning::SPP::Meas::MeasKeyTypes>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format structs
    /// @param[in] meas Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::GNSS::Positioning::SPP::Meas::MeasKeyTypes& meas, FormatContext& ctx)
    {
        if (const auto* psr = std::get_if<NAV::GNSS::Positioning::SPP::Meas::Psr>(&meas))
        {
            return fmt::format_to(ctx.out(), "psr({})", psr->satSigId);
        }
        if (const auto* doppler = std::get_if<NAV::GNSS::Positioning::SPP::Meas::Doppler>(&meas))
        {
            return fmt::format_to(ctx.out(), "doppler({})", doppler->satSigId);
        }

        return fmt::format_to(ctx.out(), "ERROR");
    }
};

#endif