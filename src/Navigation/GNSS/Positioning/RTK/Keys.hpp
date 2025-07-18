// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Keys.hpp
/// @brief Keys for the RTK algorithm for use inside the KeyedMatrices
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-21

#pragma once

#include <cstdint>
#include <vector>
#include <variant>
#include <fmt/format.h>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV::RTK
{

namespace States
{

/// @brief State Keys of the Kalman filter
enum KFStates : uint8_t
{
    PosX,           ///< Position ECEF_X [m]
    PosY,           ///< Position ECEF_Y [m]
    PosZ,           ///< Position ECEF_Z [m]
    VelX,           ///< Velocity ECEF_X [m/s]
    VelY,           ///< Velocity ECEF_Y [m/s]
    VelZ,           ///< Velocity ECEF_Z [m/s]
    KFStates_COUNT, ///< Count
};
/// @brief Double differenced N_br^1s = N_br^s - N_br^1 ambiguity [cycles] (one for each satellite signal, except for the pivot satellites)
struct AmbiguityDD
{
    /// @brief Constructor
    /// @param[in] satSigId Satellite Signal Id
    explicit AmbiguityDD(const SatSigId& satSigId) : satSigId(satSigId) {}
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const AmbiguityDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the state key type
using StateKeyType = std::variant<KFStates, AmbiguityDD>;
/// @brief Vector with all position and velocity state keys
inline static const std::vector<StateKeyType> PosVel = { KFStates::PosX, KFStates::PosY, KFStates::PosZ,
                                                         KFStates::VelX, KFStates::VelY, KFStates::VelZ };
/// @brief All position keys
inline static const std::vector<StateKeyType> Pos = { KFStates::PosX, KFStates::PosY, KFStates::PosZ };
/// @brief All velocity keys
inline static const std::vector<StateKeyType> Vel = { KFStates::VelX, KFStates::VelY, KFStates::VelZ };

} // namespace States

namespace Meas
{

/// @brief Double differenced pseudorange measurement psr_br^1s [m] (one for each satellite signal, referenced to the pivot satellite)
struct PsrDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const PsrDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};
/// @brief Double differenced carrier-phase measurement phi_br^1s [m] (one for each satellite signal, referenced to the pivot satellite)
struct CarrierDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const CarrierDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};
/// @brief Double differenced range-rate (doppler) measurement d_br^1s [m/s] (one for each satellite signal, referenced to the pivot satellite)
struct DopplerDD
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const DopplerDD& rhs) const { return satSigId == rhs.satSigId; }
    /// @brief Satellite Signal Id
    SatSigId satSigId;
};

/// Alias for the measurement key type
using MeasKeyTypes = std::variant<PsrDD, CarrierDD, DopplerDD, States::AmbiguityDD>;

/// @brief Single Observation key
template<typename ReceiverType>
struct SingleObs
{
    /// @brief Constructor
    /// @param[in] satSigId Signal id
    /// @param[in] recvType Receiver Type
    /// @param[in] obsType Observation Type
    SingleObs(const SatSigId& satSigId, ReceiverType recvType, GnssObs::ObservationType obsType)
        : satSigId(satSigId), recvType(recvType), obsType(obsType) {}

    SatSigId satSigId;                ///< Signal id
    ReceiverType recvType;            ///< Receiver Type
    GnssObs::ObservationType obsType; ///< Observation Type

    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const SingleObs<ReceiverType>& rhs) const
    {
        return satSigId == rhs.satSigId
               && recvType == rhs.recvType
               && obsType == rhs.obsType;
    }
};

/// @brief Ambiguity Observation key
template<typename ReceiverType>
struct AmbObs
{
    /// @brief Constructor
    /// @param[in] satSigId Signal id
    /// @param[in] recvType Receiver Type
    AmbObs(const SatSigId& satSigId, ReceiverType recvType)
        : satSigId(satSigId), recvType(recvType) {}

    SatSigId satSigId;     ///< Signal id
    ReceiverType recvType; ///< Receiver Type

    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const AmbObs<ReceiverType>& rhs) const
    {
        return satSigId == rhs.satSigId && recvType == rhs.recvType;
    }
};

} // namespace Meas

} // namespace NAV::RTK

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RTK::States::AmbiguityDD>
{
    /// @brief Hash function
    /// @param[in] ambDD Double differenced ambiguity
    size_t operator()(const NAV::RTK::States::AmbiguityDD& ambDD) const
    {
        return NAV::RTK::States::KFStates_COUNT + std::hash<NAV::SatSigId>()(ambDD.satSigId);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RTK::Meas::PsrDD>
{
    /// @brief Hash function
    /// @param[in] psrDD Double differenced pseudorange
    size_t operator()(const NAV::RTK::Meas::PsrDD& psrDD) const
    {
        return std::hash<NAV::SatSigId>()(psrDD.satSigId);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RTK::Meas::CarrierDD>
{
    /// @brief Hash function
    /// @param[in] cpDD Double differenced carrier-phase
    size_t operator()(const NAV::RTK::Meas::CarrierDD& cpDD) const
    {
        return std::hash<NAV::SatSigId>()(cpDD.satSigId) << 12;
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::RTK::Meas::DopplerDD>
{
    /// @brief Hash function
    /// @param[in] dDD Double differenced doppler
    size_t operator()(const NAV::RTK::Meas::DopplerDD& dDD) const
    {
        return std::hash<NAV::SatSigId>()(dDD.satSigId) << 24;
    }
};
/// @brief Hash function (needed for unordered_map)
template<typename ReceiverType>
struct hash<NAV::RTK::Meas::SingleObs<ReceiverType>>
{
    /// @brief Hash function
    /// @param[in] obs Single Observation
    size_t operator()(const NAV::RTK::Meas::SingleObs<ReceiverType>& obs) const
    {
        auto hash1 = std::hash<NAV::SatSigId>()(obs.satSigId);
        auto hash2 = static_cast<size_t>(obs.obsType);
        auto hash3 = static_cast<size_t>(obs.recvType);

        return (hash1 << 4) | (hash2 << 2) | hash3;
    }
};
/// @brief Hash function (needed for unordered_map)
template<typename ReceiverType>
struct hash<NAV::RTK::Meas::AmbObs<ReceiverType>>
{
    /// @brief Hash function
    /// @param[in] obs Single Ambiguity Observation
    size_t operator()(const NAV::RTK::Meas::AmbObs<ReceiverType>& obs) const
    {
        auto hash1 = std::hash<NAV::SatSigId>()(obs.satSigId);
        auto hash2 = static_cast<size_t>(obs.recvType);

        return (hash1 << 2) | hash2;
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RTK::States::KFStates> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::States::KFStates& state, FormatContext& ctx) const
    {
        using namespace NAV::RTK::States; // NOLINT(google-build-using-namespace)

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
        case KFStates_COUNT:
            return fmt::formatter<const char*>::format("KFStates_COUNT", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RTK::States::AmbiguityDD> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] amb Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::States::AmbiguityDD& amb, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("Amb({})", amb.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RTK::Meas::PsrDD> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] psrDD Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::Meas::PsrDD& psrDD, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("psrDD({})", psrDD.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RTK::Meas::CarrierDD> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] phiDD Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::Meas::CarrierDD& phiDD, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("phiDD({})", phiDD.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RTK::Meas::DopplerDD> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] dDD Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::Meas::DopplerDD& dDD, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("dopDD({})", dDD.satSigId), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RTK::States::StateKeyType> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::States::StateKeyType& state, FormatContext& ctx) const
    {
        using namespace NAV::RTK::States; // NOLINT(google-build-using-namespace)

        if (const auto* s = std::get_if<NAV::RTK::States::KFStates>(&state))
        {
            return fmt::formatter<std::string>::format(fmt::format("{}", *s), ctx);
        }
        if (const auto* amb = std::get_if<NAV::RTK::States::AmbiguityDD>(&state))
        {
            return fmt::formatter<std::string>::format(fmt::format("{}", *amb), ctx);
        }

        return fmt::formatter<std::string>::format("ERROR", ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::RTK::Meas::MeasKeyTypes> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] meas Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::Meas::MeasKeyTypes& meas, FormatContext& ctx) const
    {
        if (const auto* psrDD = std::get_if<NAV::RTK::Meas::PsrDD>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("{}", *psrDD), ctx);
        }
        if (const auto* phiDD = std::get_if<NAV::RTK::Meas::CarrierDD>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("{}", *phiDD), ctx);
        }
        if (const auto* dDD = std::get_if<NAV::RTK::Meas::DopplerDD>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("{}", *dDD), ctx);
        }
        if (const auto* ambDD = std::get_if<NAV::RTK::States::AmbiguityDD>(&meas))
        {
            return fmt::formatter<std::string>::format(fmt::format("{}", *ambDD), ctx);
        }

        return fmt::formatter<std::string>::format("ERROR", ctx);
    }
};

/// @brief Formatter
template<typename ReceiverType>
struct fmt::formatter<NAV::RTK::Meas::SingleObs<ReceiverType>> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] obs Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::Meas::SingleObs<ReceiverType>& obs, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("obs({}_{:5}_{})", obs.obsType, obs.recvType, obs.satSigId), ctx);
    }
};

/// @brief Formatter
template<typename ReceiverType>
struct fmt::formatter<NAV::RTK::Meas::AmbObs<ReceiverType>> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] obs Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::RTK::Meas::AmbObs<ReceiverType>& obs, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("Amb({:5}_{})", obs.recvType, obs.satSigId), ctx);
    }
};

#endif