// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UbloxGnssOrbitCollector.hpp"

#include <chrono>

#include "util/Logger.hpp"
#include "util/Container/STL.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Transformations/Units.hpp"

NAV::UbloxGnssOrbitCollector::UbloxGnssOrbitCollector()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = false;

    nm::CreateInputPin(this, "UbloxObs", Pin::Type::Flow, { NAV::UbloxObs::type() }, &UbloxGnssOrbitCollector::receiveObs);

    nm::CreateOutputPin(this, GnssNavInfo::type().c_str(), Pin::Type::Object, { GnssNavInfo::type() }, &_gnssNavInfo);
}

NAV::UbloxGnssOrbitCollector::~UbloxGnssOrbitCollector()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UbloxGnssOrbitCollector::typeStatic()
{
    return "UbloxGnssOrbitCollector";
}

std::string NAV::UbloxGnssOrbitCollector::type() const
{
    return typeStatic();
}

std::string NAV::UbloxGnssOrbitCollector::category()
{
    return "Converter";
}

bool NAV::UbloxGnssOrbitCollector::initialize()
{
    LOG_TRACE("{}: called", nameId());
    if (!_postProcessingLock.has_value())
    {
        auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_GNSS_NAV_INFO);
        _gnssNavInfo.reset();
    }
    else
    {
        _gnssNavInfo.reset();
    }
    _ephemerisBuilder.clear();
    _lastAccessedBuilder.clear();
    _warningsNotImplemented.clear();

    if (inputPins.at(INPUT_PORT_INDEX_UBLOX_OBS).isPinLinked()
        && !inputPins.at(INPUT_PORT_INDEX_UBLOX_OBS).link.connectedNode->isOnlyRealtime()
        && !_postProcessingLock.has_value())
    {
        _postProcessingLock.emplace(outputPins.at(OUTPUT_PORT_INDEX_GNSS_NAV_INFO).dataAccessMutex);
    }

    return true;
}

void NAV::UbloxGnssOrbitCollector::onDeleteLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (_postProcessingLock.has_value())
    {
        _postProcessingLock.reset();
    }
}

NAV::UbloxGnssOrbitCollector::EphemerisBuilder& NAV::UbloxGnssOrbitCollector::getEphemerisBuilder(const SatId& satId, const InsTime& insTime, size_t IOD)
{
    LOG_DEBUG("{}: Searching for [{}] at [{}]", nameId(), satId, insTime.toYMDHMS(GPST));
    if (IOD != 0)
    {
        _lastAccessedBuilder[satId] = IOD;
    }

    auto iter = std::find_if(_ephemerisBuilder.begin(), _ephemerisBuilder.end(), [&](const auto& builder) {
        return builder.satId == satId && builder.navData->refTime == insTime;
    });
    if (iter == _ephemerisBuilder.end())
    {
        LOG_DEBUG("{}:   Constructing new builder", nameId());

        std::shared_ptr<SatNavData> satNavData = nullptr;
        switch (SatelliteSystem_(satId.satSys))
        {
        case GPS:
            satNavData = std::make_shared<GPSEphemeris>(insTime);
            break;
        case GAL:
            satNavData = std::make_shared<GalileoEphemeris>(insTime);
            break;
        case GLO:
            // satNavData = std::make_shared<GLONASSEphemeris>(insTime);
            LOG_CRITICAL("{}: GLONASS not implemented yet.", nameId()); // TODO: Not yet supported
            break;
        case BDS:
            // satNavData = std::make_shared<BDSEphemeris>(insTime);
            LOG_CRITICAL("{}: BeiDou not implemented yet.", nameId()); // TODO: Not yet supported
            break;
        case QZSS:
            LOG_CRITICAL("{}: QZSS not implemented yet.", nameId()); // TODO: Not yet supported
            break;
        case IRNSS:
            LOG_CRITICAL("{}: IRNSS not implemented yet.", nameId()); // TODO: Not yet supported
            break;
        case SBAS:
            LOG_CRITICAL("{}: SBAS not implemented yet.", nameId()); // TODO: Not yet supported
            break;
        case SatSys_None:
            LOG_CRITICAL("{}: Satellite system cannot be none.", nameId());
            break;
        }
        return _ephemerisBuilder.emplace_back(satId, satNavData);
    }

    LOG_DEBUG("{}:   Found builder", nameId());
    return *iter;
}

std::optional<std::reference_wrapper<NAV::UbloxGnssOrbitCollector::EphemerisBuilder>>
    NAV::UbloxGnssOrbitCollector::getEphemerisBuilder(const SatId& satId, size_t IOD)
{
    LOG_DEBUG("{}: Searching for [{}] at Issue of Data [{}]", nameId(), satId, IOD);
    _lastAccessedBuilder[satId] = IOD;

    auto iter = std::find_if(_ephemerisBuilder.begin(), _ephemerisBuilder.end(), [&](const auto& builder) {
        if (builder.satId == satId)
        {
            if (builder.navData->type == SatNavData::GPSEphemeris)
            {
                auto ephemeris = std::dynamic_pointer_cast<GPSEphemeris>(builder.navData);
                return ephemeris && ephemeris->IODE == IOD;
            }
            if (builder.navData->type == SatNavData::GalileoEphemeris)
            {
                auto ephemeris = std::dynamic_pointer_cast<GalileoEphemeris>(builder.navData);
                return ephemeris && ephemeris->IODnav == IOD;
            }
        }
        return false;
    });
    if (iter != _ephemerisBuilder.end())
    {
        LOG_DEBUG("{}:   Found builder", nameId());
        return *iter;
    }
    LOG_DEBUG("{}:   Could not find builder. Ignoring subframe.", nameId());
    return std::nullopt;
}

std::optional<std::reference_wrapper<NAV::UbloxGnssOrbitCollector::EphemerisBuilder>>
    NAV::UbloxGnssOrbitCollector::getLastEphemerisBuilder(const SatId& satId)
{
    LOG_DEBUG("{}: Searching the last builder for [{}]", nameId(), satId);
    if (_lastAccessedBuilder.contains(satId))
    {
        return getEphemerisBuilder(satId, _lastAccessedBuilder.at(satId));
    }
    LOG_DEBUG("{}:   Could not find last accessed builder. Ignoring subframe.", nameId());
    return std::nullopt;
}

void NAV::UbloxGnssOrbitCollector::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    [[maybe_unused]] auto ubloxObs = std::static_pointer_cast<const UbloxObs>(queue.extract_front());

    if (ubloxObs->msgClass == ubx::UBX_CLASS_RXM)
    {
        if (static_cast<ubx::UbxRxmMessages>(ubloxObs->msgId) == ubx::UbxRxmMessages::UBX_RXM_SFRBX)
        {
            const auto& sfrbx = std::get<ubx::UbxRxmSfrbx>(ubloxObs->data);

            SatelliteSystem satSys = ubx::getSatSys(sfrbx.gnssId);
            SatId satId(satSys, sfrbx.svId);
            LOG_DEBUG("{}: [{}] Converting message at [{}][{}]", nameId(), satId, ubloxObs->insTime.toYMDHMS(GPST), ubloxObs->insTime.toGPSweekTow(GPST));

            switch (SatelliteSystem_(satSys))
            {
            case GPS:
                decryptGPS(satId, sfrbx, ubloxObs->insTime);
                break;
            case GAL:
                decryptGalileo(satId, sfrbx, ubloxObs->insTime);
                break;
            case GLO:
                decryptGLONASS(satId, sfrbx, ubloxObs->insTime);
                break;
            case BDS:
                decryptBeiDou(satId, sfrbx, ubloxObs->insTime);
                break;
            case QZSS:
                decryptQZSS(satId, sfrbx, ubloxObs->insTime);
                break;
            case IRNSS:
                decryptIRNSS(satId, sfrbx, ubloxObs->insTime);
                break;
            case SBAS:
                decryptSBAS(satId, sfrbx, ubloxObs->insTime);
                break;
            case SatSys_None:
                LOG_CRITICAL("{}: Satellite system cannot be none.", nameId());
                break;
            }
        }
    }

    if (_postProcessingLock.has_value()
        && inputPins.at(INPUT_PORT_INDEX_UBLOX_OBS).isPinLinked()
        && inputPins.at(INPUT_PORT_INDEX_UBLOX_OBS).link.getConnectedPin()->noMoreDataAvailable)
    {
        _postProcessingLock.reset();
    }
}

void NAV::UbloxGnssOrbitCollector::decryptGPS(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime)
{
    // u-blox 8 / u-blox M8: Receiver description - Including protocol specification, ch. 10.2, p. 30f
    // > For GPS L1C/A signals, there is a fairly straightforward mapping between the reported subframe
    // > and the structure of subframe and words described in the GPS ICD. Each subframe comprises ten
    // > data words, which are reported in the same order they are received.
    // >
    // >          MSB                                           LSB
    // > 1 to 10: Pad (2 bits)    Data (24 bits)    Parity (6 bits)
    // >
    // > Note that as the GPS data words only comprise 30 bits, the 2 most significant bits in each word
    // > reported by UBX-RXM-SFRBX are padding and should be ignored.

    if (sfrbx.numWords != 10)
    {
        LOG_ERROR("{}: [{}] Received {} instead of 10 words", nameId(), satId, sfrbx.numWords);
        return;
    }

    size_t w = 0; // Word counter

    // Telemetry Word
    //   30 bits long, occurs every 6 seconds                   ┌ Integrity Status Flag
    //     Preamble                                             │  ┌ Reserved
    // 1 0 0 0 1 0 1 1  MSB       TLM Message              LSB  │  │       Parity
    // ┌──────┴──────┐ ┌───────────────┴──────────────────────┐ │  │  ┌──────┴────────┐
    // 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30
    LOG_DATA("{}: [{}]   tlm: {} {}", nameId(), satId, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
    constexpr uint8_t TLM_PREAMBLE = 0b10001011;
    if (static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111) != TLM_PREAMBLE) // Preamble found after the 2 padding bits
    {
        LOG_DEBUG("{}: [{}] Wrong telemetry word preamble. Ignoring SFRBX message.", nameId(), satId);
        return;
    }

    // Handover Word (HOW)
    //   30 bits long, occurs every 6 seconds        anti-spoof (A-S) flag
    //                                   alert flag  │
    //  MSB       TOW-Count Message         LSB   │  │ Subframe ID         Parity
    // ┌──────────────────┴────────────────────┐  │  │ ┌───┴──┐       ┌──────┴────────┐
    // 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30
    w++;
    LOG_DATA("{}: [{}]   how: {} {}", nameId(), satId, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
    auto subFrameId = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 8) & 0b111);
    LOG_DATA("{}: [{}]   subFrameId: {}", nameId(), satId, subFrameId);

    if (subFrameId == 0 || subFrameId > 5)
    {
        LOG_ERROR("{}: [{}]   The subFrameId has to be in the range [1..=5] but it is {}", nameId(), satId, subFrameId);
        return;
    }

    auto gpsWeekToW = insTime.toGPSweekTow(GPST);

    auto finishSubFrame = [&]<size_t N>(const std::shared_ptr<SatNavData>& ephemeris, size_t subFrameId, std::bitset<N>& subframesFound) {
        if (1 <= subFrameId && subFrameId <= 3)
        {
            subframesFound.set(subFrameId - 1);

            LOG_DATA("{}: [{}]     subframesFound: {}", nameId(), satId, subframesFound);
            if (subframesFound.count() == 3)
            {
                LOG_DATA("{}: [{}] [{}] All subframes found. Updating gnnsNavInfo", nameId(), satId, ephemeris->refTime.toYMDHMS(GPST));
                std::unique_lock guard(outputPins.at(OUTPUT_PORT_INDEX_GNSS_NAV_INFO).dataAccessMutex, std::defer_lock);
                if (!_postProcessingLock.has_value())
                {
                    guard.lock();
                }
                _gnssNavInfo.satelliteSystems |= satId.satSys;
                _gnssNavInfo.addSatelliteNavData(satId, ephemeris);
            }
        }
    };

    if (subFrameId == 1) // Third through tenth words of subframe 1 shall each contain 6 parity bits as their LSBs
    {
        // IS-GPS-200M: Figure 20-1. Data Format (sheet 1 of 11), p. 79
        // IS-GPS-200M: 20.3.3.3 Subframe 1, p. 91ff
        // IS-GPS-200M: Table 20-I. Subframe 1 Parameters, p. 96

        // T_GD, af2, af1, af0 shall be two's complement, with the sign bit (+ or -) occupying the MSB

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 3 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        [[maybe_unused]] auto WN = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 20) & 0b1111111111); // 10 BITS - Transmission Week Number
        auto L2ChannelCodes = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 18) & 0b11);               //  2 BITS
        auto uraIndex = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111);                   //  4 BITS
        auto svHealth = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 8) & 0b111111);                  //  6 BITS
        auto IODC = static_cast<uint16_t>(((sfrbx.dwrd.at(w) >> 6) & 0b11) << 8);                  //  2 MSBs - 10 BITS TOTAL
        LOG_DATA("{}: [{}]       WN {}, L2ChannelCodes {}, uraIndex {}, svHealth {}, IODC {}", nameId(), satId, WN, L2ChannelCodes, uraIndex, svHealth, std::bitset<10>(IODC));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 4 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto L2DataFlagPCode = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 29) & 0b1); // 1 BIT
        LOG_DATA("{}: [{}]       L2DataFlagPCode {}", nameId(), satId, L2DataFlagPCode);

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 5 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 6 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 7 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto T_GD = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 6) & 0b11111111);
        LOG_DATA("{}: [{}]       T_GD {}", nameId(), satId, T_GD);

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 8 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        IODC |= static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111);           //  8 LSBs - 10 BITS TOTAL
        auto toc = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 6) & 0b1111111111111111); // 16 LSBs
        LOG_DATA("{}: [{}]       IODC {} ({}), toc {}", nameId(), satId, IODC, std::bitset<10>(IODC), toc);

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 9 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto af2 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111);         // 8 BITS
        auto af1 = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 6) & 0b1111111111111111); // 16 BITS
        LOG_DATA("{}: [{}]       af2 {}, af1 {}", nameId(), satId, af2, af1);

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 10 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto af0 = math::interpretAs<int32_t, 22>(sfrbx.dwrd.at(w) >> 8); // 22 BITS
        LOG_DATA("{}: [{}]       af0 {}", nameId(), satId, af0);

        InsTime insTimeToc(gpsWeekToW.gpsCycle, gpsWeekToW.gpsWeek, toc * std::pow(2, 4), GPST);

        auto& ephemerisBuilder = getEphemerisBuilder(satId, insTimeToc);
        auto ephemeris = std::dynamic_pointer_cast<GPSEphemeris>(ephemerisBuilder.navData);
        ephemeris->toc = insTimeToc;

        ephemeris->L2ChannelCodes = L2ChannelCodes;
        ephemeris->svAccuracy = gpsUraIdx2Val(uraIndex);
        ephemeris->svHealth = svHealth;
        ephemeris->IODC = IODC;
        ephemeris->L2DataFlagPCode = L2DataFlagPCode;
        ephemeris->T_GD = T_GD * std::pow(2, -31);
        ephemeris->refTime = ephemeris->toc;
        ephemeris->a = {
            af0 * std::pow(2, -31),
            af1 * std::pow(2, -43),
            af2 * std::pow(2, -55),
        };
        LOG_DATA("{}: [{}]     svAccuracy [{} m], svHealth [{}], IODC [{}], tgd [{:.3e} s]", nameId(), satId,
                 ephemeris->svAccuracy, ephemeris->svHealth, ephemeris->IODC, ephemeris->T_GD);
        LOG_DATA("{}: [{}]     toc [{}], a0 [{:.3e} s], a1 [{:.3e} s/s], a2 [{:.3e} s/s^2]", nameId(), satId,
                 ephemeris->toc.toYMDHMS(GPST), ephemeris->a[0], ephemeris->a[1], ephemeris->a[2]);

        finishSubFrame(ephemeris, subFrameId, ephemerisBuilder.subframes);
    }
    else if (subFrameId == 2) // The third through tenth words of subframes 2 and 3 shall each contain six parity bits as their LSBs
    {
        // IS-GPS-200M: Figure 20-1. Data Format (sheet 2 of 11), p. 80
        // IS-GPS-200M: 20.3.3.4 Subframes 2 and 3, p. 101ff
        // IS-GPS-200M: Table 20-III. Ephemeris Parameters, p. 105

        // Crs, delta_n, M_0, Cuc, Cus shall be two's complement, with the sign bit (+ or -) occupying the MSB

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 3 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto IODE = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111);       // 8 BITS
        auto Crs = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 6) & 0b1111111111111111); // 16 BITS
        LOG_DATA("{}: [{}]       IODE {}, Crs {}", nameId(), satId, IODE, Crs);

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 4 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto delta_n = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111111); // 16 BITS
        auto M_0 = static_cast<int32_t>(((sfrbx.dwrd.at(w) >> 6) & 0b11111111) << 24);      //  8 MSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       delta_n {}, M_0 {}", nameId(), satId, delta_n, std::bitset<32>(static_cast<uint32_t>(M_0)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 5 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        M_0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 6) & 0b111111111111111111111111); // 24 LSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       M_0 {} ({})", nameId(), satId, M_0, std::bitset<32>(static_cast<uint32_t>(M_0)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 6 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto Cuc = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111111); // 16 BITS
        uint32_t e = ((sfrbx.dwrd.at(w) >> 6) & 0b11111111) << 24;                      //  8 MSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       Cuc {}, e {}", nameId(), satId, Cuc, std::bitset<32>(e));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 7 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        e |= (sfrbx.dwrd.at(w) >> 6) & 0b111111111111111111111111; // 24 LSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       e {} ({})", nameId(), satId, e, std::bitset<32>(e));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 8 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto Cus = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111111); // 16 BITS
        uint32_t sqrt_A = ((sfrbx.dwrd.at(w) >> 6) & 0b11111111) << 24;                 //  8 MSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       Cus {}, sqrt_A {}", nameId(), satId, Cus, std::bitset<32>(sqrt_A));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 9 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        sqrt_A |= (sfrbx.dwrd.at(w) >> 6) & 0b111111111111111111111111; // 24 LSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       sqrt_A {} ({})", nameId(), satId, sqrt_A, std::bitset<32>(sqrt_A));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 10 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto toe = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111111);      // 16 BITS
        auto fitInterval = static_cast<bool>((sfrbx.dwrd.at(w) >> 13) & 0b1);                 // 1 BIT
        [[maybe_unused]] auto AODO = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 8) & 0b11111); // 5 BITS
        LOG_DATA("{}: [{}]       toe {}, fitInterval {}, AODO {}", nameId(), satId, toe, fitInterval, AODO);

        InsTime insTimeToe(gpsWeekToW.gpsCycle, gpsWeekToW.gpsWeek, toe * std::pow(2, 4), GPST);

        auto& ephemerisBuilder = getEphemerisBuilder(satId, insTimeToe);
        auto ephemeris = std::dynamic_pointer_cast<GPSEphemeris>(ephemerisBuilder.navData);
        ephemeris->toe = insTimeToe;

        ephemeris->IODE = IODE;
        ephemeris->Crs = Crs * std::pow(2, -5);
        ephemeris->delta_n = semicircles2rad(delta_n * std::pow(2, -43));
        ephemeris->M_0 = semicircles2rad(M_0 * std::pow(2, -31));
        ephemeris->Cuc = Cuc * std::pow(2, -29);
        ephemeris->e = e * std::pow(2, -33);
        ephemeris->Cus = Cus * std::pow(2, -29);
        ephemeris->sqrt_A = sqrt_A * std::pow(2, -19);
        ephemeris->toe = InsTime(gpsWeekToW.gpsCycle, gpsWeekToW.gpsWeek, toe * std::pow(2, 4), GPST);
        ephemeris->fitInterval = fitInterval ? 8.0 : 4.0;
        // AODO

        LOG_DATA("{}: [{}]     IODE [{}], Crs [{:.3e} m], delta_n [{:.3e} rad/s], M_0 [{:.3e} rad], Cuc [{:.3e} rad]", nameId(), satId,
                 ephemeris->IODE, ephemeris->Crs, ephemeris->delta_n, ephemeris->M_0, ephemeris->Cuc);
        LOG_DATA("{}: [{}]     e [{:.3e}], Cus [{:.3e} rad], sqrt_A [{:.3e} m^(1/2)], toe [{}], fitInterval [{:.1f}]", nameId(), satId,
                 ephemeris->e, ephemeris->Cus, ephemeris->sqrt_A, ephemeris->toe.toGPSweekTow(GPST), ephemeris->fitInterval);

        finishSubFrame(ephemeris, subFrameId, ephemerisBuilder.subframes);
    }
    else if (subFrameId == 3) // The third through tenth words of subframes 2 and 3 shall each contain six parity bits as their LSBs
    {
        // IS-GPS-200M: Figure 20-1. Data Format (sheet 3 of 11), p. 81
        // IS-GPS-200M: 20.3.3.4 Subframes 2 and 3, p. 101ff
        // IS-GPS-200M: Table 20-III. Ephemeris Parameters, p. 105

        // Cic, Omega_0, Cis, i_0, Crc, omega, Omega_dot, IDOT shall be two's complement, with the sign bit (+ or -) occupying the MSB

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 3 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto Cic = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111111);    // 16 BITS
        auto Omega_0 = static_cast<int32_t>(((sfrbx.dwrd.at(w) >> 6) & 0b11111111) << 24); //  8 MSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       Cic {}, Omega_0 {}", nameId(), satId, Cic, std::bitset<32>(static_cast<uint32_t>(Omega_0)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 4 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        Omega_0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 6) & 0b111111111111111111111111); // 24 LSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       Omega_0 {} ({})", nameId(), satId, Omega_0, std::bitset<32>(static_cast<uint32_t>(Omega_0)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 5 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto Cis = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111111); // 16 BITS
        auto i_0 = static_cast<int32_t>(((sfrbx.dwrd.at(w) >> 6) & 0b11111111) << 24);  //  8 MSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       Cis {}, i_0 {}", nameId(), satId, Cis, std::bitset<32>(static_cast<uint32_t>(i_0)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 6 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        i_0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 6) & 0b111111111111111111111111); // 24 LSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       i_0 {} ({})", nameId(), satId, i_0, std::bitset<32>(static_cast<uint32_t>(i_0)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 7 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto Crc = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111111);  // 16 BITS
        auto omega = static_cast<int32_t>(((sfrbx.dwrd.at(w) >> 6) & 0b11111111) << 24); //  8 MSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       Crc {}, omega {}", nameId(), satId, Crc, std::bitset<32>(static_cast<uint32_t>(omega)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 8 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        omega |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 6) & 0b111111111111111111111111); // 24 LSBs - 32 BITS TOTAL
        LOG_DATA("{}: [{}]       omega {} ({})", nameId(), satId, omega, std::bitset<32>(static_cast<uint32_t>(omega)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 9 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto Omega_dot = math::interpretAs<int32_t, 24>(sfrbx.dwrd.at(w) >> 6); // 24 BITS
        LOG_DATA("{}: [{}]       Omega_dot {} ({})", nameId(), satId, Omega_dot, std::bitset<32>(static_cast<uint32_t>(Omega_dot)));

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 10 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        auto IODE = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111); // 8 BITS
        auto i_dot = math::interpretAs<int16_t, 14>(sfrbx.dwrd.at(w) >> 8);      // 14 BITS
        LOG_DATA("{}: [{}]       IODE {}, i_dot {} ({})", nameId(), satId, IODE, i_dot, std::bitset<16>(static_cast<uint16_t>(i_dot)));

        auto ephemerisBuilder = getEphemerisBuilder(satId, IODE);
        if (!ephemerisBuilder.has_value())
        {
            LOG_WARN("{}: [{}]       Could not find Ephemeris builder for IODE {}", nameId(), satId, IODE);
            return;
        }
        auto ephemeris = std::dynamic_pointer_cast<GPSEphemeris>(ephemerisBuilder->get().navData);

        ephemeris->Cic = Cic * std::pow(2, -29);
        ephemeris->Omega_0 = semicircles2rad(Omega_0 * std::pow(2, -31));
        ephemeris->Cis = Cis * std::pow(2, -29);
        ephemeris->i_0 = semicircles2rad(i_0 * std::pow(2, -31));
        ephemeris->Crc = Crc * std::pow(2, -5);
        ephemeris->omega = semicircles2rad(omega * std::pow(2, -31));
        ephemeris->Omega_dot = semicircles2rad(Omega_dot * std::pow(2, -43));
        ephemeris->IODE = IODE;
        ephemeris->i_dot = semicircles2rad(i_dot * std::pow(2, -43));

        LOG_DATA("{}: [{}]     Cic [{:.3e} rad], Omega_0 [{:.3e} rad], Cis [{:.3e} rad], i_0 [{:.3e} rad], Crc [{:.3e} m]", nameId(), satId,
                 ephemeris->Cic, ephemeris->Omega_0, ephemeris->Cis, ephemeris->i_0, ephemeris->Crc);
        LOG_DATA("{}: [{}]     omega [{:.3e} rad], Omega_dot [{:.3e} rad/s], IODE [{}], i_dot [{:.3e} rad/s]", nameId(), satId,
                 ephemeris->omega, ephemeris->Omega_dot, ephemeris->IODE, ephemeris->i_dot);

        finishSubFrame(ephemeris, subFrameId, ephemerisBuilder->get().subframes);
    }
    else if (subFrameId == 4) // Words three through ten of each page contain six parity bits as their LSBs
    {
        // Page description
        // 1, 6, 11, 16 and 21:        Reserved
        // 2, 3, 4, 5, 7, 8, 9 and 10: almanac data for SV 25 through 32 respectively
        // 12, 19, 20, 22, 23 and 24:  Reserved
        // 13:                         NMCT
        // 14 and 15:                  Reserved for system use
        // 17:                         Special messages
        // 18:                         Ionospheric and UTC data
        // 25:                         A-S flags/SV configurations for 32 SVs, plus SV health for SV 25 through 32
        //
        // IS-GPS-200M: Table 20-V. Data IDs and SV IDs in Subframes 4 and 5, p. 114
        // IS-GPS-200M: 20.3.3.5 Subframes 4 and 5, p. 112ff

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 3 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        [[maybe_unused]] auto dataId = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 28) & 0b11); // 2 BITS
        auto svId = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 22) & 0b111111);                // 6 BITS
        LOG_DATA("{}: [{}]       dataId {}, svId {}", nameId(), satId, dataId, svId);

        if (svId == 56) // IS-GPS-200M: Figure 20-1. Data Format (sheet  8 of 11), p. 86 - for page 18
        {
            // IS-GPS-200M: 20.3.3.5.1.6 Coordinated Universal Time (UTC) Parameters, p. 121
            // IS-GPS-200M: Table 20-IX. UTC Parameters, p. 122

            // IS-GPS-200M: 20.3.3.5.1.7 Ionospheric Data, p. 121
            // IS-GPS-200M: Table 20-X. onospheric Parameters, p. 123

            auto alpha0 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 14) & 0b11111111); // 8 BITS
            auto alpha1 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 6) & 0b11111111);  // 8 BITS
            LOG_DATA("{}: [{}]       alpha0 {}, alpha1 {}", nameId(), satId, alpha0, alpha1);

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 4 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            auto alpha2 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111); // 8 BITS
            auto alpha3 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 14) & 0b11111111); // 8 BITS
            auto beta0 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 6) & 0b11111111);   // 8 BITS
            LOG_DATA("{}: [{}]       alpha2 {}, alpha3 {}, beta0 {}", nameId(), satId, alpha2, alpha3, beta0);

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 5 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            auto beta1 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111); // 8 BITS
            auto beta2 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 14) & 0b11111111); // 8 BITS
            auto beta3 = static_cast<int8_t>((sfrbx.dwrd.at(w) >> 6) & 0b11111111);  // 8 BITS
            LOG_DATA("{}: [{}]       beta1 {}, beta2 {}, beta3 {}", nameId(), satId, beta1, beta2, beta3);

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 6 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            auto A1 = math::interpretAs<int32_t, 24>(sfrbx.dwrd.at(w) >> 6); // 24 BITS
            LOG_DATA("{}: [{}]       A1 {}", nameId(), satId, A1);

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 7 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            auto A0 = static_cast<int32_t>(((sfrbx.dwrd.at(w) >> 6) & 0b111111111111111111111111) << 8); // 24 MSBs - 32 BITS TOTAL
            LOG_DATA("{}: [{}]       A0 {}", nameId(), satId, std::bitset<32>(static_cast<uint32_t>(A0)));

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 8 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            A0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111); //  8 LSBs - 32 BITS TOTAL
            LOG_DATA("{}: [{}]       A0 {} ({})", nameId(), satId, A0, std::bitset<32>(static_cast<uint32_t>(A0)));

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 9 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 10 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));

            std::unique_lock guard(outputPins.at(OUTPUT_PORT_INDEX_GNSS_NAV_INFO).dataAccessMutex, std::defer_lock);
            if (!_postProcessingLock.has_value())
            {
                guard.lock();
            }

            _gnssNavInfo.ionosphericCorrections.insert(satId.satSys, IonosphericCorrections::Alpha,
                                                       {
                                                           alpha0 * std::pow(2, -30) /* [s] */,
                                                           alpha1 * std::pow(2, -27) /* [s/semi-circle] */,
                                                           alpha2 * std::pow(2, -24) /* [s/semi-circle^2] */,
                                                           alpha3 * std::pow(2, -24) /* [s/semi-circle^3] */,
                                                       });
            _gnssNavInfo.ionosphericCorrections.insert(satId.satSys, IonosphericCorrections::Beta,
                                                       {
                                                           beta0 * std::pow(2, 11) /* [s] */,
                                                           beta1 * std::pow(2, 14) /* [s/semi-circle] */,
                                                           beta2 * std::pow(2, 16) /* [s/semi-circle^2] */,
                                                           beta3 * std::pow(2, 16) /* [s/semi-circle^3] */,
                                                       });
            _gnssNavInfo.timeSysCorr[{ satId.satSys.getTimeSystem(), UTC }] = GnssNavInfo::TimeSystemCorrections{
                .a0 = A0 * std::pow(2, -30),
                .a1 = A1 * std::pow(2, -50),
            };
        }
        else
        {
            // IS-GPS-200M: Figure 20-1. Data Format (sheet  6 of 11), p. 84 - for pages 1, 6, 11, 16 and 21
            // IS-GPS-200M: Figure 20-1. Data Format (sheet  7 of 11), p. 85 - for pages 12, 19, 20, 22, 23 and 24
            // IS-GPS-200M: Figure 20-1. Data Format (sheet  9 of 11), p. 87 - for page 25
            // IS-GPS-200M: Figure 20-1. Data Format (sheet 10 of 11), p. 88 - for page 13
            // IS-GPS-200M: Figure 20-1. Data Format (sheet 11 of 11), p. 89 - for pages 14, 15 and 17

            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 4 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 5 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 6 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 7 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 8 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 9 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
            w++;
            LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 10 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        }
    }
    else if (subFrameId == 5) // Words three through ten of each page contain six parity bits as their LSBs
    {
        // Page description
        // 1 through 24: almanac data for SV 1 through 24
        // 25:           SV health data for SV 1 through 24, the almanac reference time, the almanac reference week number
        //
        // IS-GPS-200M: Figure 20-1. Data Format (sheet 4 of 11), p. 82 - for page 1 through 24
        // IS-GPS-200M: Figure 20-1. Data Format (sheet 5 of 11), p. 83 - for page 25
        // IS-GPS-200M: 20.3.3.5 Subframes 4 and 5, p. 112ff

        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 3 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 4 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 5 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 6 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 7 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 8 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 9 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
        w++;
        LOG_DATA("{}: [{}]     word {:2}: {} {}", nameId(), satId, w + 1 /* 10 */, std::bitset<2>(sfrbx.dwrd.at(w) >> 30), std::bitset<30>(sfrbx.dwrd.at(w)));
    }
}

void NAV::UbloxGnssOrbitCollector::decryptGalileo(const SatId& satId, const ubx::UbxRxmSfrbx& sfrbx, const InsTime& insTime)
{
    // u-blox 8 / u-blox M8: Receiver description - Including protocol specification, ch. 10.5, p. 32f
    // > The Galileo E1OS and E5b signals both transmit the I/NAV message but in different configurations.
    // > For Galileo E1OS signals, each reported subframe contains a pair of I/NAV pages as described in
    // > the Galileo ICD.
    // > Galileo pages can either be "Nominal" or "Alert" pages. For Nominal pages the eight words are
    // > arranged as follows:
    // >
    // >    MSB                                                                                          LSB
    // > 1: Even/Odd (1 bit)   Page type (1 bit)   Word type (6 bits)   Data (122 - 99) (24 bits)
    // > 2:                                   Data (98 - 67) (32 bits)
    // > 3:                                   Data (66 - 35) (32 bits)
    // > 4: Data (34 - 17) (18 bits)     Tail (6 bits)     Pad (8 bits)
    // > 5: Even/Odd (1 bit)   Page type (1 bit)   Data (16 - 1) (16 bits)    Reserved 1 (40 - 27) (14 bits)
    // > 6: Reserved 2 (26 - 1) (26 bits)     SAR (22 - 17) (6 bits)
    // > 7: SAR (16 - 1) (16 bits)     Spare (2 bits)     CRC (24 - 11) (14 bits)
    // > 8: CRC (10 - 1) (10 bits)    Reserved 2 (8 bits)     Tail (6 bits)     Pad (8 bits)
    // >
    // > Alert pages are reported in very similar manner, but the page type bits will have value 1 and the
    // > structure of the eight words will be slightly different (as indicated by the Galileo ICD).

    if (sfrbx.numWords != 8)
    {
        LOG_ERROR("{}: [{}] Received {} instead of 8 words", nameId(), satId, sfrbx.numWords);
        return;
    }

    size_t w = 0; // Word counter

    auto even = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 31) & 0b1);
    auto pageTypeEven = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 30) & 0b1);
    auto odd = static_cast<uint8_t>((sfrbx.dwrd.at(4) >> 31) & 0b1);
    auto pageTypeOdd = static_cast<uint8_t>((sfrbx.dwrd.at(4) >> 30) & 0b1);
    auto wordType = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 24) & 0b111111);
    LOG_DEBUG("{}: [{}]   wordType: {:2}, even {} ({} pageType), odd {} ({} pageType)", nameId(), satId, wordType, even, pageTypeEven, odd, pageTypeOdd);

    if (even != 0 || pageTypeEven == 1
        || odd != 1 || pageTypeOdd == 1)
    {
        LOG_DEBUG("{}: [{}]     Ignoring message, because one of the page types is alert page", nameId(), satId);
        return;
    }

    auto gpsWeekToW = insTime.toGPSweekTow(GST);

    auto finishWord = [&]<size_t N>(const std::shared_ptr<SatNavData>& ephemeris, size_t wordType, std::bitset<N>& subframesFound) {
        if (1 <= wordType && wordType <= 5)
        {
            subframesFound.set(wordType - 1);

            LOG_DATA("{}: [{}]     subframesFound: {}", nameId(), satId, subframesFound);
            if (subframesFound.count() == 5)
            {
                LOG_DATA("{}: [{}] [{}] All words found. Updating gnnsNavInfo", nameId(), satId, ephemeris->refTime.toYMDHMS(GPST));
                std::unique_lock guard(outputPins.at(OUTPUT_PORT_INDEX_GNSS_NAV_INFO).dataAccessMutex, std::defer_lock);
                if (!_postProcessingLock.has_value())
                {
                    guard.lock();
                }
                _gnssNavInfo.satelliteSystems |= satId.satSys;
                _gnssNavInfo.addSatelliteNavData(satId, ephemeris);
            }
        }
    };

    if (wordType == 1) // Ephemeris (1/4)
    {
        // Galileo-OS-SIS-ICD-v2.0: ch. 4.3.5, Table 40 Bits Allocation for I/NAV Word Type 1, p. 37
        //
        // Even/Odd  PageType  WordType  IODnav  toe   M_0    e   sqrt_A  Tail  Pad   Even/Odd  PageType  sqrt_A  Reserved  Reserved 1
        //     1        1         6        10     14    32   32    18      6     8       1          1       14        2        14
        // └───────────────────┬───────────────────┘    │     │   └───────┬───────┘   └───────────────────────┬──────────────────────┘
        // Ublox               1                        2     3           4                                   5

        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.1, Table 60 Ephemeris Parameters, p. 43f
        // > M_0 shall be two's complement, with the sign bit (+ or -) occupying the MSB

        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 1 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto IODnav = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111);
        auto toe = static_cast<uint16_t>(sfrbx.dwrd.at(w) & 0b11111111111111);
        LOG_DEBUG("{}: [{}]       IODnav {}, toe {}", nameId(), satId, IODnav, toe);

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 2 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto M_0 = static_cast<int32_t>(sfrbx.dwrd.at(w));
        LOG_DEBUG("{}: [{}]       M_0 {}", nameId(), satId, M_0);

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 3 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        uint32_t e = sfrbx.dwrd.at(w);
        LOG_DEBUG("{}: [{}]       e {}", nameId(), satId, e);

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 4 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        uint32_t sqrt_A = ((sfrbx.dwrd.at(w) >> 14) & 0b111111111111111111) << 14;
        LOG_DEBUG("{}: [{}]       sqrt_A {}", nameId(), satId, std::bitset<32>(sqrt_A));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 5 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        sqrt_A |= (sfrbx.dwrd.at(w) >> 16) & 0b11111111111111;
        LOG_DEBUG("{}: [{}]       sqrt_A {} ({})", nameId(), satId, sqrt_A, std::bitset<32>(sqrt_A));

        InsTime insTimeToe(gpsWeekToW.gpsCycle, gpsWeekToW.gpsWeek, toe * 60.0, GST);

        auto& ephemerisBuilder = getEphemerisBuilder(satId, insTimeToe, IODnav);
        auto ephemeris = std::dynamic_pointer_cast<GalileoEphemeris>(ephemerisBuilder.navData);
        ephemeris->toe = insTimeToe;

        ephemeris->IODnav = IODnav;
        ephemeris->M_0 = semicircles2rad(M_0 * std::pow(2.0, -31));
        ephemeris->e = e * std::pow(2.0, -33);
        ephemeris->sqrt_A = sqrt_A * std::pow(2.0, -19);

        LOG_DEBUG("{}: [{}]     IODnav [{}], toe [{}], M_0 [{:.3e} rad], e [{:.3e}], sqrt_A [{:.3e} m^(1/2)]", nameId(), satId,
                  ephemeris->IODnav, ephemeris->toe.toYMDHMS(GPST), ephemeris->M_0, ephemeris->e, ephemeris->sqrt_A);

        finishWord(ephemeris, wordType, ephemerisBuilder.subframes);
    }
    else if (wordType == 2) // Ephemeris (2/4)
    {
        // Galileo-OS-SIS-ICD-v2.0: ch. 4.3.5, Table 41 Bits Allocation for I/NAV Word Type 2, p. 37
        //
        // Even/Odd  PageType  WordType  IODnav  Omega_0   Omega_0  i_0   i_0  omega   omega    Tail  Pad   Even/Odd  PageType  i_dot  Reserved  Reserved 1
        //     1        1         6        10       14        18     14    18     14     18      6     8       1          1       14        2        14
        // └──────────────────────┬────────────────────┘   └────┬─────┘   └────┬───┘   └───────┬────────┘   └──────────────────────┬──────────────────────┘
        // Ublox                  1                             2              3               4                                   5

        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.1, Table 60 Ephemeris Parameters, p. 43f
        // > Omega_0, i_0, omega, i_dot shall be two's complement, with the sign bit (+ or -) occupying the MSB

        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 1 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto IODnav = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111);
        auto Omega_0 = static_cast<int32_t>((sfrbx.dwrd.at(w) & 0b11111111111111) << 18);
        LOG_DEBUG("{}: [{}]       IODnav {}, Omega_0 {}", nameId(), satId, IODnav, std::bitset<32>(static_cast<uint32_t>(Omega_0)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 2 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        Omega_0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 14) & 0b111111111111111111);
        auto i_0 = static_cast<int32_t>((sfrbx.dwrd.at(w) & 0b11111111111111) << 18);
        LOG_DEBUG("{}: [{}]       Omega_0 {} ({}), i_0 {}", nameId(), satId, Omega_0, std::bitset<32>(static_cast<uint32_t>(Omega_0)), std::bitset<32>(static_cast<uint32_t>(i_0)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 3 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        i_0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 14) & 0b111111111111111111);
        auto omega = static_cast<int32_t>((sfrbx.dwrd.at(w) & 0b11111111111111) << 18);
        LOG_DEBUG("{}: [{}]       i_0 {} ({}), omega {}", nameId(), satId, i_0, std::bitset<32>(static_cast<uint32_t>(i_0)), std::bitset<32>(static_cast<uint32_t>(omega)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 4 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        omega |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 14) & 0b111111111111111111);
        LOG_DEBUG("{}: [{}]       omega {} ({})", nameId(), satId, omega, std::bitset<32>(static_cast<uint32_t>(omega)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 5 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto i_dot = math::interpretAs<int16_t, 14>(sfrbx.dwrd.at(w) >> 16);
        LOG_DEBUG("{}: [{}]       i_dot {} ({})", nameId(), satId, i_dot, std::bitset<16>(static_cast<uint16_t>(i_dot)));

        auto ephemerisBuilder = getEphemerisBuilder(satId, IODnav);
        if (!ephemerisBuilder.has_value())
        {
            LOG_WARN("{}: [{}]       Could not find Ephemeris builder for IODnav {}", nameId(), satId, IODnav);
            return;
        }
        auto ephemeris = std::dynamic_pointer_cast<GalileoEphemeris>(ephemerisBuilder->get().navData);

        ephemeris->Omega_0 = semicircles2rad(Omega_0 * std::pow(2.0, -31));
        ephemeris->i_0 = semicircles2rad(i_0 * std::pow(2.0, -31));
        ephemeris->omega = semicircles2rad(omega * std::pow(2.0, -31));
        ephemeris->i_dot = semicircles2rad(i_dot * std::pow(2.0, -43));

        LOG_DEBUG("{}: [{}]     IODnav [{}], Omega_0 [{} rad], i_0 [{:.3e} rad], omega [{:.3e} rad], i_dot [{:.3e} rad/s]", nameId(), satId,
                  ephemeris->IODnav, ephemeris->Omega_0, ephemeris->i_0, ephemeris->omega, ephemeris->i_dot);

        finishWord(ephemeris, wordType, ephemerisBuilder->get().subframes);
    }
    else if (wordType == 3) // Ephemeris (3/4) and SISA
    {
        // Galileo-OS-SIS-ICD-v2.0: ch. 4.3.5, Table 42 Bits Allocation for I/NAV Word Type 3, p. 38
        //
        // Even/Odd  PageType  WordType  IODnav  Omega_dot   Omega_dot  delta_n  Cuc   Cuc  Cus  Crc   Crc  Crs  Tail  Pad   Even/Odd  PageType  Crs  SISA(E1,E5b)  Reserved 1
        //     1        1         6        10       14          10        16      6     10   16   6     10   8    6     8       1          1      8      8              14
        // └───────────────────────┬─────────────────────┘   └──────────┬──────────┘   └─────┬─────┘   └────────┬────────┘   └───────────────────────┬───────────────────────┘
        // Ublox                   1                                    2                    3                  4                                    5

        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.1, Table 60 Ephemeris Parameters, p. 43f
        // > Omega_dot, delta_n, Cuc, Cus, Crc, Crs shall be two's complement, with the sign bit (+ or -) occupying the MSB

        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 1 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto IODnav = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111);
        auto Omega_dot = static_cast<int32_t>((sfrbx.dwrd.at(w) & 0b11111111111111) << 10);
        LOG_DEBUG("{}: [{}]       IODnav {}, Omega_dot {}", nameId(), satId, IODnav, std::bitset<32>(static_cast<uint32_t>(Omega_dot)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 2 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        Omega_dot |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 22) & 0b1111111111);
        Omega_dot = math::interpretAs<int32_t, 24>(Omega_dot);
        auto delta_n = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 6) & 0b1111111111111111);
        auto Cuc = static_cast<int16_t>((sfrbx.dwrd.at(w) & 0b111111) << 10);
        LOG_DEBUG("{}: [{}]       Omega_dot {} ({}), delta_n {}, Cuc {}", nameId(), satId,
                  Omega_dot, std::bitset<32>(static_cast<uint32_t>(Omega_dot)), delta_n, std::bitset<16>(static_cast<uint16_t>(Cuc)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 3 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        Cuc |= static_cast<int16_t>((sfrbx.dwrd.at(w) >> 22) & 0b1111111111); // NOLINT(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
        auto Cus = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 6) & 0b1111111111111111);
        auto Crc = static_cast<int16_t>((sfrbx.dwrd.at(w) & 0b111111) << 10);
        LOG_DEBUG("{}: [{}]       Cuc {} ({}), Cus {}, Crc {}", nameId(), satId, Cuc, std::bitset<16>(static_cast<uint16_t>(Cuc)), Cus, std::bitset<16>(static_cast<uint16_t>(Crc)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 4 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        Crc |= static_cast<int16_t>((sfrbx.dwrd.at(w) >> 22) & 0b1111111111); // NOLINT(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
        auto Crs = static_cast<int16_t>(((sfrbx.dwrd.at(w) >> 14) & 0b11111111) << 8);
        LOG_DEBUG("{}: [{}]       Crc {} ({}), Crs {}", nameId(), satId, Crc, std::bitset<16>(static_cast<uint16_t>(Crc)), std::bitset<16>(static_cast<uint16_t>(Crs)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 5 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        Crs |= static_cast<int16_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111); // NOLINT(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
        auto SISA = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 14) & 0b11111111);
        LOG_DEBUG("{}: [{}]       Crs {} ({}), SISA {}", nameId(), satId, Crs, std::bitset<16>(static_cast<uint16_t>(Crs)), SISA);

        auto ephemerisBuilder = getEphemerisBuilder(satId, IODnav);
        if (!ephemerisBuilder.has_value())
        {
            LOG_WARN("{}: [{}]       Could not find Ephemeris builder for IODnav {}", nameId(), satId, IODnav);
            return;
        }
        auto ephemeris = std::dynamic_pointer_cast<GalileoEphemeris>(ephemerisBuilder->get().navData);

        ephemeris->Omega_dot = semicircles2rad(Omega_dot * std::pow(2.0, -43));
        ephemeris->delta_n = semicircles2rad(delta_n * std::pow(2.0, -43));
        ephemeris->Cuc = Cuc * std::pow(2.0, -29);
        ephemeris->Cus = Cus * std::pow(2.0, -29);
        ephemeris->Crc = Crc * std::pow(2.0, -5);
        ephemeris->Crs = Crs * std::pow(2.0, -5);
        ephemeris->signalAccuracy = galSisaIdx2Val(SISA);

        LOG_DEBUG("{}: [{}]     IODnav [{}], Omega_dot [{} rad/s], delta_n [{:.3e} rad/s], Cuc [{:.3e} rad], Cus [{:.3e} rad/s], Crc [{:.3e} m], Crs [{:.3e} m], SISA [{:.3e} m]", nameId(), satId,
                  ephemeris->IODnav, ephemeris->Omega_dot, ephemeris->delta_n, ephemeris->Cuc, ephemeris->Cus, ephemeris->Crc, ephemeris->Crs, ephemeris->signalAccuracy);

        finishWord(ephemeris, wordType, ephemerisBuilder->get().subframes);
    }
    else if (wordType == 4) // SVID, Ephemeris (4/4), and Clock correction parameters
    {
        // Galileo-OS-SIS-ICD-v2.0: ch. 4.3.5, Table 43 Bits Allocation for I/NAV Word Type 4, p. 38
        //
        // Even/Odd  PageType  WordType  IODnav  SVID  Cic   Cic  Cis  toc   toc  af0   af0  af1  Tail  Pad   Even/Odd  PageType  af1  af2  Spare  Reserved 1
        //     1        1         6        10      6    8     8   16    8     6    26    5   13    6     8       1          1      8    6     2        14
        // └───────────────────────┬─────────────────────┘   └─────┬─────┘   └──┬───┘   └────────┬────────┘   └───────────────────────┬─────────────────────┘
        // Ublox                   1                               2            3                4                                    5

        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.1, Table 60 Ephemeris Parameters, p. 43f
        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.3, Table 63 Galileo Clock Correction Parameters, p. 46
        // > Cic, Cis, af0, af1, af2 shall be two's complement, with the sign bit (+ or -) occupying the MSB

        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 1 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto IODnav = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 14) & 0b1111111111);
        [[maybe_unused]] auto SVID = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 8) & 0b111111);
        auto Cic = static_cast<int16_t>((sfrbx.dwrd.at(w) & 0b11111111) << 8);
        LOG_DEBUG("{}: [{}]       IODnav {}, SVID {}, Cic {}", nameId(), satId, IODnav, SVID, std::bitset<16>(static_cast<uint16_t>(Cic)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 2 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        Cic |= static_cast<int16_t>((sfrbx.dwrd.at(w) >> 24) & 0b11111111); // NOLINT(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
        auto Cis = static_cast<int16_t>((sfrbx.dwrd.at(w) >> 8) & 0b1111111111111111);
        auto toc = static_cast<uint16_t>((sfrbx.dwrd.at(w) & 0b11111111) << 6);
        LOG_DEBUG("{}: [{}]       Cic {} ({}), Cis {}, toc {}", nameId(), satId, Cic, std::bitset<16>(static_cast<uint16_t>(Cic)), Cis, std::bitset<16>(toc));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 3 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        toc |= static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 26) & 0b111111);
        auto af0 = static_cast<int32_t>((sfrbx.dwrd.at(w) & 0b11111111111111111111111111) << 5);
        LOG_DEBUG("{}: [{}]       toc {} ({}), af0 {}", nameId(), satId, toc, std::bitset<16>(toc), std::bitset<32>(static_cast<uint32_t>(af0)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 4 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        af0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 27) & 0b11111);
        af0 = math::interpretAs<int32_t, 31>(af0);
        auto af1 = static_cast<int32_t>(((sfrbx.dwrd.at(w) >> 14) & 0b1111111111111) << 8);
        LOG_DEBUG("{}: [{}]       af0 {} ({}), af1 {}", nameId(), satId, af0, std::bitset<32>(static_cast<uint32_t>(af0)), std::bitset<32>(static_cast<uint32_t>(af1)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 5 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        af1 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 22) & 0b11111111);
        af1 = math::interpretAs<int32_t, 21>(af1);
        auto af2 = math::interpretAs<int8_t, 6>(sfrbx.dwrd.at(w) >> 16);
        LOG_DEBUG("{}: [{}]       af1 {} ({}), af1 {}", nameId(), satId, af1, std::bitset<32>(static_cast<uint32_t>(af1)), std::bitset<8>(static_cast<uint8_t>(af2)));

        auto ephemerisBuilder = getEphemerisBuilder(satId, IODnav);
        if (!ephemerisBuilder.has_value())
        {
            LOG_WARN("{}: [{}]       Could not find Ephemeris builder for IODnav {}", nameId(), satId, IODnav);
            return;
        }
        auto ephemeris = std::dynamic_pointer_cast<GalileoEphemeris>(ephemerisBuilder->get().navData);

        InsTime insTimeToc(gpsWeekToW.gpsCycle, gpsWeekToW.gpsWeek, toc * 60.0, GST);

        ephemeris->Cic = Cic * std::pow(2.0, -29);
        ephemeris->Cis = Cis * std::pow(2.0, -29);
        ephemeris->toc = insTimeToc;
        ephemeris->refTime = ephemeris->toc;
        ephemeris->a = {
            af0 * std::pow(2, -34),
            af1 * std::pow(2, -46),
            af2 * std::pow(2, -59),
        };

        LOG_DEBUG("{}: [{}]     IODnav [{}], Cic [{:.3e} rad], Cis [{:.3e} rad]", nameId(), satId, ephemeris->IODnav, ephemeris->Cic, ephemeris->Cis);
        LOG_DEBUG("{}: [{}]     toc [{}], a0 [{:.3e} s], a1 [{:.3e} s/s], a2 [{:.3e} s/s^2]", nameId(), satId,
                  ephemeris->toc.toYMDHMS(GPST), ephemeris->a[0], ephemeris->a[1], ephemeris->a[2]);

        finishWord(ephemeris, wordType, ephemerisBuilder->get().subframes);
    }
    else if (wordType == 5) // Ionospheric correction, BGD, signal health and data validity status and GST
    {
        // Galileo-OS-SIS-ICD-v2.0: ch. 4.3.5, Table 44 Bits Allocation for I/NAV Word Type 5, p. 38
        //
        // Even/Odd  PageType  WordType  ai0  ai1  ai2   ai2  Region 1-5  BGD(E1,E5a)  BGD(E1,E5b)   BGD(E1,E5b)  E5b_HS  E1b_HS  E5b_DVS  E1b_DVS  WN  TOW   TOW  Spare  Tail  Pad   Even/Odd  PageType  Spare  Reserved 1
        //     1        1         6       11   11   2     12      5x1         10           5               5        2       2         1       1     12   9     11    7     13    6       1          1       16      14
        // └─────────────────────┬───────────────────┘   └───────────────────┬───────────────────┘   └─────────────────────────┬──────────────────────────┘   └─────────┬─────────┘   └──────────────────┬────────────────┘
        // Ublox                 1                                           2                                                 3                                        4                                5

        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.6, Table 67 Ionospheric Correction Parameters, p. 48
        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.5, Table 65 BGD Parameters, p. 47
        // > ai1, ai2, BGD(E1,E5a), BGD(E1,E5b) shall be two's complement, with the sign bit (+ or -) occupying the MSB

        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 1 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto ai0 = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 13) & 0b11111111111);
        auto ai1 = math::interpretAs<int16_t, 11>((sfrbx.dwrd.at(w) >> 2) & 0b11111111111);
        auto ai2 = static_cast<int16_t>((sfrbx.dwrd.at(w) & 0b11) << 12);
        LOG_DEBUG("{}: [{}]       ai0 {} ({}), ai1 {} ({}), ai2 {}", nameId(), satId,
                  ai0, std::bitset<16>(ai0), ai1, std::bitset<16>(static_cast<uint16_t>(ai1)), std::bitset<16>(static_cast<uint16_t>(ai2)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 2 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        ai2 |= static_cast<int16_t>((sfrbx.dwrd.at(w) >> 20) & 0b111111111111); // NOLINT(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
        ai2 = math::interpretAs<int16_t, 14>(ai2);
        auto BGD_E1_E5a = math::interpretAs<int16_t, 10>((sfrbx.dwrd.at(w) >> 5) & 0b1111111111);
        auto BGD_E1_E5b = static_cast<int16_t>((sfrbx.dwrd.at(w) & 0b11111) << 5);
        LOG_DEBUG("{}: [{}]       ai2 {} ({}), BGD_E1_E5a {}, BGD_E1_E5b {}", nameId(), satId, ai2, std::bitset<16>(static_cast<uint16_t>(ai2)), BGD_E1_E5a, std::bitset<16>(static_cast<uint16_t>(BGD_E1_E5b)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 3 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        BGD_E1_E5b |= static_cast<int16_t>((sfrbx.dwrd.at(w) >> 27) & 0b11111); // NOLINT(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
        BGD_E1_E5b = math::interpretAs<int16_t, 10>(BGD_E1_E5b);
        auto E5b_HS = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 25) & 0b11);
        auto E1b_HS = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 23) & 0b11);
        auto E5b_DVS = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 22) & 0b1);
        auto E1b_DVS = static_cast<uint8_t>((sfrbx.dwrd.at(w) >> 21) & 0b1);
        [[maybe_unused]] auto WN = static_cast<uint16_t>((sfrbx.dwrd.at(w) >> 9) & 0b111111111111);
        [[maybe_unused]] uint32_t TOW = (sfrbx.dwrd.at(w) & 0b111111111) << 11;
        LOG_DEBUG("{}: [{}]       BGD_E1_E5b {} ({}), E5b_HS {}, E1b_HS {}, E5b_DVS {}, E1b_DVS {}, WN {}, TOW {}", nameId(), satId,
                  BGD_E1_E5b, std::bitset<16>(static_cast<uint16_t>(BGD_E1_E5b)), E5b_HS, E1b_HS, E5b_DVS, E1b_DVS, WN, std::bitset<32>(TOW));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 4 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        TOW |= (sfrbx.dwrd.at(w) >> 21) & 0b11111111111;
        LOG_DEBUG("{}: [{}]       TOW {} ({})", nameId(), satId, TOW, std::bitset<32>(TOW));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 5 */, std::bitset<32>(sfrbx.dwrd.at(w)));

        auto ephemerisBuilder = getLastEphemerisBuilder(satId);
        if (!ephemerisBuilder.has_value())
        {
            LOG_WARN("{}: [{}]       Could not find any ephemeris builder", nameId(), satId);
            return;
        }
        auto ephemeris = std::dynamic_pointer_cast<GalileoEphemeris>(ephemerisBuilder->get().navData);

        ephemeris->BGD_E1_E5a = BGD_E1_E5a * std::pow(2.0, -32);
        ephemeris->BGD_E1_E5b = BGD_E1_E5b * std::pow(2.0, -32);
        ephemeris->svHealth = {
            .E5a_DataValidityStatus = {},
            .E5b_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>(E5b_DVS),
            .E1B_DataValidityStatus = static_cast<GalileoEphemeris::SvHealth::DataValidityStatus>(E1b_DVS),
            .E5a_SignalHealthStatus = {},
            .E5b_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>(E5b_HS),
            .E1B_SignalHealthStatus = static_cast<GalileoEphemeris::SvHealth::SignalHealthStatus>(E1b_HS),
        };
        ephemeris->dataSource[0] = true; // I/NAV E1-B
        ephemeris->dataSource[9] = true; // af0-af2, Toc, SISA are for E5b,E1

        {
            std::unique_lock guard(outputPins.at(OUTPUT_PORT_INDEX_GNSS_NAV_INFO).dataAccessMutex, std::defer_lock);
            if (!_postProcessingLock.has_value())
            {
                guard.lock();
            }
            // ‘sfu’ (solar flux unit) is not a SI unit but can be converted as: 1 sfu = 10e-22 W/(m2*Hz)
            _gnssNavInfo.ionosphericCorrections.insert(satId.satSys, IonosphericCorrections::Alpha,
                                                       {
                                                           ai0 * std::pow(2.0, -2) /* [sfu] */,
                                                           ai1 * std::pow(2.0, -8) /* [sfu/degree] */,
                                                           ai2 * std::pow(2.0, -15) /* [sfu/degree^2] */,
                                                           0.0,
                                                       });
        }

        LOG_DEBUG("{}: [{}]     BGD_E1_E5a [{:.3e} s], BGD_E1_E5b [{:.3e} rad]", nameId(), satId, ephemeris->BGD_E1_E5a, ephemeris->BGD_E1_E5b);

        finishWord(ephemeris, wordType, ephemerisBuilder->get().subframes);
    }
    else if (wordType == 6) // GST-UTC conversion parameters
    {
        // Galileo-OS-SIS-ICD-v2.0: ch. 4.3.5, Table 44 Bits Allocation for I/NAV Word Type 6, p. 38
        //
        // Even/Odd  PageType  WordType  A0   A0  A1   dt_LS  t0t  WNot  WN_LSF   DN  dt_LSF  TOW  Tail  Pad   Even/Odd  PageType  TOW  Spare  Reserved 1
        //     1        1         6      24    8  24     8     8    8     8        3    8      7    6     8       1          1      13    3        14
        // └───────────────┬──────────────┘   └──┬─┘   └──────────┬───────────┘   └────────────┬───────────┘   └────────────────────┬───────────────────┘
        // Ublox           1                     2                  3                          4                                    5

        // Galileo-OS-SIS-ICD-v2.0: ch. 5.1.7, Table 68 Ionospheric Correction Parameters, p. 49
        // > A0, A1, dt_LS, dt_LSF shall be two's complement, with the sign bit (+ or -) occupying the MSB

        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 1 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        auto A0 = static_cast<int32_t>((sfrbx.dwrd.at(w) & 0b111111111111111111111111) << 8);
        LOG_DEBUG("{}: [{}]       A0 {}", nameId(), satId, std::bitset<32>(static_cast<uint32_t>(A0)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 2 */, std::bitset<32>(sfrbx.dwrd.at(w)));
        A0 |= static_cast<int32_t>((sfrbx.dwrd.at(w) >> 24) & 0b11111111);
        auto A1 = math::interpretAs<int32_t, 24>(sfrbx.dwrd.at(w) & 0b111111111111111111111111);
        LOG_DEBUG("{}: [{}]       A0 {} ({}), A1 {} ({})", nameId(), satId, A0, std::bitset<32>(static_cast<uint32_t>(A0)), A1, std::bitset<32>(static_cast<uint32_t>(A1)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 3 */, std::bitset<32>(sfrbx.dwrd.at(w)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 4 */, std::bitset<32>(sfrbx.dwrd.at(w)));

        w++;
        LOG_DEBUG("{}: [{}]     word {:2}: {}", nameId(), satId, w + 1 /* 5 */, std::bitset<32>(sfrbx.dwrd.at(w)));

        std::unique_lock guard(outputPins.at(OUTPUT_PORT_INDEX_GNSS_NAV_INFO).dataAccessMutex, std::defer_lock);
        if (!_postProcessingLock.has_value())
        {
            guard.lock();
        }

        _gnssNavInfo.timeSysCorr[{ satId.satSys.getTimeSystem(), UTC }] = GnssNavInfo::TimeSystemCorrections{
            .a0 = A0 * std::pow(2, -30),
            .a1 = A1 * std::pow(2, -50),
        };
    }
}

void NAV::UbloxGnssOrbitCollector::decryptGLONASS([[maybe_unused]] const SatId& satId, const ubx::UbxRxmSfrbx& /* sfrbx */, const InsTime& /* insTime */)
{
    // u-blox 8 / u-blox M8: Receiver description - Including protocol specification, ch. 10.3, p. 31
    // > For GLONASS L1OF and L2OF signals, each reported subframe contains a string as described in
    // > the GLONASS ICD. This string comprises 85 data bits which are reported over three 32 bit words
    // > in the UBX-RXM-SFRBX message. Data bits 1 to 8 are always a hamming code, whilst bits 81 to 84
    // > are a string number and bit 85 is the idle chip, which should always have a value of zero. The
    // > meaning of other bits vary with string and frame number.
    // > The fourth and final 32 bit word in the UBX-RXM-SFRBX message contains frame and superframe
    // > numbers (where available). These values aren't actually transmitted by the SVs, but are deduced
    // > by the receiver and are included to aid decoding of the transmitted data. However, the receiver
    // > does not always know these values, in which case a value of zero is reported.
    // >
    // >    MSB                                                                       LSB
    // > 1: Idle chip (1 bit, always 0)    String # (4 bits)     Data (80 - 54) (27 bits)
    // > 2:                              Data (53 - 22) (32 bits)
    // > 3: Data (21 - 9) (13 bits)      Hammering code (8 bits)   Pad (11 bits)
    // > 4: Superframe # (16 bits)       Pad (8 bits)             Frame # (8 bits)
    // >
    // > In some circumstances, (especially on startup) the receiver may be able to decode data from a
    // > GLONASS SV before it can identify the SV. When this occurs UBX-RXM-SFRBX messages will be
    // > issued with an svId of 255 to indicate "unknown".

    if (!_warningsNotImplemented.contains(satId.satSys))
    {
        LOG_WARN("{}: ubx orbit collector is not implemented for GLONASS yet.", nameId()); // TODO: not implemented yet.
        _warningsNotImplemented.insert(satId.satSys);
    }

    // auto ephemeris = std::dynamic_pointer_cast<GLONASSEphemeris>(ephemerisBuilder->get().navData);

    // sfrbx.freqId;
    // _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<GLONASSEphemeris>(epoch, tau_c,
    //                                                                                         -m_tau_n, gamma_n, static_cast<bool>(health),
    //                                                                                         pos, vel, accelLuniSolar,
    //                                                                                         frequencyNumber_accuracyCode));
}

void NAV::UbloxGnssOrbitCollector::decryptBeiDou([[maybe_unused]] const SatId& satId, const ubx::UbxRxmSfrbx& /* sfrbx */, const InsTime& /* insTime */)
{
    // u-blox 8 / u-blox M8: Receiver description - Including protocol specification, ch. 10.4, p. 32
    // > For BeiDou (B1I) signals, there is a fairly straightforward mapping between the reported subframe
    // > and the structure of subframe and words described in the BeiDou ICD. Each subframe comprises
    // > ten data words, which are reported in the same order they are received.
    // >
    // >          MSB                                           LSB
    // > 1 to 10: Pad (2 bits)    Data (22 bits)    Parity (8 bits)
    // >
    // > Note that as the BeiDou data words only comprise 30 bits, the 2 most significant bits in each word
    // > reported by UBX-RXM-SFRBX are padding and should be ignored.

    if (!_warningsNotImplemented.contains(satId.satSys))
    {
        LOG_WARN("{}: ubx orbit collector is not implemented for BeiDou yet.", nameId()); // TODO: not implemented yet.
        _warningsNotImplemented.insert(satId.satSys);
    }

    // auto ephemeris = std::dynamic_pointer_cast<BDSEphemeris>(ephemerisBuilder->get().navData);

    // _gnssNavInfo.addSatelliteNavData({ satSys, satNum }, std::make_shared<BDSEphemeris>(epoch, toe,
    //                                                                                     IODE_IODnav_AODE_IODEC, fitInterval_AODC, a,
    //                                                                                     sqrt_A, e, i_0, Omega_0, omega, M_0,
    //                                                                                     delta_n, Omega_dot, i_dot, Cus, Cuc,
    //                                                                                     Cis, Cic, Crs, Crc,
    //                                                                                     signalAccuracy, svHealth,
    //                                                                                     tgd_bgd5a_TGD1, IODC_bgd5b_TGD2));
}

void NAV::UbloxGnssOrbitCollector::decryptQZSS([[maybe_unused]] const SatId& satId, const ubx::UbxRxmSfrbx& /* sfrbx */, const InsTime& /* insTime */)
{
    // u-blox 8 / u-blox M8: Receiver description - Including protocol specification, ch. 10.7, p. 34
    // > The structure of the data delivered by QZSS L1C/A signals is effectively identical to that of GPS
    // > (L1C/A). Similarly the QZSS L2C signal is effectively identical to the GPS (L2C).
    // > The QZSS (L1SAIF) signal is different and uses the same data block format as used by SBAS
    // > (L1C/A). QZSS (SAIF) signals can be distinguished from QZSS (L1C/A and L2C) by noting that they
    // > have 8 words, instead of 10 for QZSS (L1C/A and L2C).

    if (!_warningsNotImplemented.contains(satId.satSys))
    {
        LOG_WARN("{}: ubx orbit collector is not implemented for QZSS yet.", nameId()); // TODO: not implemented yet.
        _warningsNotImplemented.insert(satId.satSys);
    }
}

void NAV::UbloxGnssOrbitCollector::decryptIRNSS([[maybe_unused]] const SatId& satId, const ubx::UbxRxmSfrbx& /* sfrbx */, const InsTime& /* insTime */)
{
    if (!_warningsNotImplemented.contains(satId.satSys))
    {
        LOG_WARN("{}: ubx orbit collector is not implemented for IRNSS yet.", nameId()); // TODO: not implemented yet.
        _warningsNotImplemented.insert(satId.satSys);
    }
}

void NAV::UbloxGnssOrbitCollector::decryptSBAS([[maybe_unused]] const SatId& satId, const ubx::UbxRxmSfrbx& /* sfrbx */, const InsTime& /* insTime */)
{
    // u-blox 8 / u-blox M8: Receiver description - Including protocol specification, ch. 10.6, p. 33f

    if (!_warningsNotImplemented.contains(satId.satSys))
    {
        LOG_WARN("{}: ubx orbit collector is not implemented for SBAS yet.", nameId()); // TODO: not implemented yet.
        _warningsNotImplemented.insert(satId.satSys);
    }
}