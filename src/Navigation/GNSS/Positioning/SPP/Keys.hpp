// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Keys.hpp
/// @brief Keys for the SPP algorithm for use inside the KeyedMatrices
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-22

#pragma once

#include <vector>
#include <variant>
#include <fmt/format.h>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/SystemModel/MotionModel.hpp"
#include "Navigation/GNSS/SystemModel/InterFrequencyBiasModel.hpp"
#include "Navigation/GNSS/SystemModel/ReceiverClockModel.hpp"

namespace NAV::SPP
{

namespace States
{

constexpr size_t POS_STATE_COUNT = 4; ///< Amount of states to estimate for the position
constexpr size_t VEL_STATE_COUNT = 4; ///< Amount of states to estimate for the velocity

constexpr size_t POS_VEL_STATE_COUNT = 6; ///< Amount of states

/// Alias for the state key type
using StateKeyType = std::variant<Keys::MotionModelKey, Keys::RecvClkBias, Keys::RecvClkDrift, Keys::InterFreqBias>;

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

} // namespace NAV::SPP

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Meas::Psr& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Meas::Doppler& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SPP::States::StateKeyType& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Meas::MeasKeyTypes& obj);

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::SPP::Meas::Psr>
{
    /// @brief Hash function
    /// @param[in] psr Pseudorange observation
    size_t operator()(const NAV::SPP::Meas::Psr& psr) const
    {
        return std::hash<NAV::SatSigId>()(psr.satSigId);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::SPP::Meas::Doppler>
{
    /// @brief Hash function
    /// @param[in] doppler Doppler observation
    size_t operator()(const NAV::SPP::Meas::Doppler& doppler) const
    {
        return std::hash<NAV::SatSigId>()(doppler.satSigId) << 12;
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::SPP::Meas::Psr> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] psr Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SPP::Meas::Psr& psr, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("psr({})", psr.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::SPP::Meas::Doppler> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] doppler Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SPP::Meas::Doppler& doppler, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("dop({})", doppler.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::SPP::States::StateKeyType> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SPP::States::StateKeyType& state, FormatContext& ctx) const
    {
        using namespace NAV::Keys; // NOLINT(google-build-using-namespace)

        if (const auto* s = std::get_if<MotionModelKey>(&state))
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
            case MotionModelKey_COUNT:
                return fmt::formatter<std::string>::format("MotionModelKey_COUNT", ctx);
            }
        }
        if (const auto* recvClkErr = std::get_if<RecvClkBias>(&state))
        {
            return fmt::formatter<std::string>::format(fmt::format("RecvClkBias({}))", recvClkErr->satSys), ctx);
        }
        if (const auto* recvClkDrift = std::get_if<RecvClkDrift>(&state))
        {
            return fmt::formatter<std::string>::format(fmt::format("RecvClkDrift({}))", recvClkDrift->satSys), ctx);
        }
        if (const auto* interFreqBias = std::get_if<NAV::Keys::InterFreqBias>(&state))
        {
            return fmt::formatter<std::string>::format(fmt::format("InterFreqBias({}))", interFreqBias->freq), ctx);
        }

        return fmt::formatter<std::string>::format("ERROR", ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::SPP::Meas::MeasKeyTypes> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] meas Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SPP::Meas::MeasKeyTypes& meas, FormatContext& ctx) const
    {
        if (const auto* psr = std::get_if<NAV::SPP::Meas::Psr>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("psr({})", psr->satSigId), ctx);
        }
        if (const auto* doppler = std::get_if<NAV::SPP::Meas::Doppler>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("doppler({})", doppler->satSigId), ctx);
        }

        return fmt::formatter<std::string>::format("ERROR", ctx);
    }
};

#endif