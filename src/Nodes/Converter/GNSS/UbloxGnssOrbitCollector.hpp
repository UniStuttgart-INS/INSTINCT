// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UbloxGnssOrbitCollector.hpp
/// @brief Collects UBX-RXM-SFRBX messages and provides the Orbit information
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-05

#pragma once

#include <mutex>
#include <bitset>
#include <memory>
#include <unordered_map>
#include <set>

#include "internal/Node/Node.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"

namespace ubx = NAV::vendor::ublox;

namespace NAV
{
/// Collects UBX-RXM-SFRBX messages and provides the Orbit information
class UbloxGnssOrbitCollector : public Node
{
  public:
    /// @brief Default constructor
    UbloxGnssOrbitCollector();
    /// @brief Destructor
    ~UbloxGnssOrbitCollector() override;
    /// @brief Copy constructor
    UbloxGnssOrbitCollector(const UbloxGnssOrbitCollector&) = delete;
    /// @brief Move constructor
    UbloxGnssOrbitCollector(UbloxGnssOrbitCollector&&) = delete;
    /// @brief Copy assignment operator
    UbloxGnssOrbitCollector& operator=(const UbloxGnssOrbitCollector&) = delete;
    /// @brief Move assignment operator
    UbloxGnssOrbitCollector& operator=(UbloxGnssOrbitCollector&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

  private:
    constexpr static size_t INPUT_PORT_INDEX_UBLOX_OBS = 0;      ///< @brief Flow (UbloxObs)
    constexpr static size_t OUTPUT_PORT_INDEX_GNSS_NAV_INFO = 0; ///< @brief Object

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Called when a link is to be deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void onDeleteLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Data object to share over the output pin
    GnssNavInfo _gnssNavInfo;

    /// Mutex to lock if the connected ublox obs provider is a file reader
    std::optional<std::unique_lock<std::mutex>> _postProcessingLock;

    /// @brief Ephemeris builder to store unfinished ephemeris data till all subframes are collected
    struct EphemerisBuilder
    {
        /// @brief Constructor
        /// @param[in] satId Satellite identifier
        /// @param[in] navData Nav data to store
        EphemerisBuilder(const SatId& satId, std::shared_ptr<SatNavData> navData)
            : satId(satId), navData(std::move(navData))
        {
            subframes.reset();
        }

        SatId satId;                         ///< Satellite Identifier
        std::shared_ptr<SatNavData> navData; ///< Navigation data pointer
        std::bitset<5> subframes;            ///< Flags for which subframes were received already. e.g. Subframes 1, 2, 3 for GPS
    };

    /// @brief Map of ephemeris build helper for each satellite
    std::vector<EphemerisBuilder> _ephemerisBuilder;

    /// @brief List of IOD for each satellite
    std::unordered_map<SatId, size_t> _lastAccessedBuilder;

    /// List of satellite systems to emit warnings because conversion is not implemented yet
    std::set<SatelliteSystem> _warningsNotImplemented;

    /// @brief Searches the ephemeris builder for the given satellite and time. If nothing found returns a new instance.
    /// @param[in] satId Satellite identifier
    /// @param[in] insTime Time to search for (either time of ephemeris or time of clock)
    /// @param[in] IOD Issue of Data (Ephemeris, Nav, ...)
    /// @return Reference to the ephemeris builder
    EphemerisBuilder& getEphemerisBuilder(const SatId& satId, const InsTime& insTime, size_t IOD = 0);

    /// @brief Searches the ephemeris builder for the given Issue of Data Ephemeris
    /// @param[in] satId Satellite Identifier
    /// @param[in] IOD Issue of Data (Ephemeris, Nav, ...)
    /// @return Reference to the ephemeris builder if it was found
    std::optional<std::reference_wrapper<EphemerisBuilder>> getEphemerisBuilder(const SatId& satId, size_t IOD);

    /// @brief Searches the most recent ephemeris builder for the given satellite
    /// @param[in] satId Satellite identifier
    /// @return Reference to the ephemeris builder if it was found
    std::optional<std::reference_wrapper<EphemerisBuilder>> getLastEphemerisBuilder(const SatId& satId);

    /// @brief Data receive function
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Decrypt the GPS SFRBX message
    /// @param[in] satId Satellite Identifier
    /// @param[in] sfrbx RAWX-SFRBX message
    /// @param[in] insTime Time of the message
    void decryptGPS(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime);

    /// @brief Decrypt the Galileo SFRBX message
    /// @param[in] satId Satellite Identifier
    /// @param[in] sfrbx RAWX-SFRBX message
    /// @param[in] insTime Time of the message
    void decryptGalileo(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime);

    /// @brief Decrypt the GLONASS SFRBX message
    /// @param[in] satId Satellite Identifier
    /// @param[in] sfrbx RAWX-SFRBX message
    /// @param[in] insTime Time of the message
    void decryptGLONASS(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime);

    /// @brief Decrypt the BeiDou SFRBX message
    /// @param[in] satId Satellite Identifier
    /// @param[in] sfrbx RAWX-SFRBX message
    /// @param[in] insTime Time of the message
    void decryptBeiDou(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime);

    /// @brief Decrypt the QZSS SFRBX message
    /// @param[in] satId Satellite Identifier
    /// @param[in] sfrbx RAWX-SFRBX message
    /// @param[in] insTime Time of the message
    void decryptQZSS(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime);

    /// @brief Decrypt the IRNSS SFRBX message
    /// @param[in] satId Satellite Identifier
    /// @param[in] sfrbx RAWX-SFRBX message
    /// @param[in] insTime Time of the message
    void decryptIRNSS(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime);

    /// @brief Decrypt the SBAS SFRBX message
    /// @param[in] satId Satellite Identifier
    /// @param[in] sfrbx RAWX-SFRBX message
    /// @param[in] insTime Time of the message
    void decryptSBAS(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime);
};

} // namespace NAV
