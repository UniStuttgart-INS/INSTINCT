// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UbloxGnssObsConverter.hpp"

#include <map>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

NAV::UbloxGnssObsConverter::UbloxGnssObsConverter()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = false;

    nm::CreateInputPin(this, "UbloxObs", Pin::Type::Flow, { NAV::UbloxObs::type() }, &UbloxGnssObsConverter::receiveObs);

    nm::CreateOutputPin(this, "GnssObs", Pin::Type::Flow, { NAV::GnssObs::type() });
}

NAV::UbloxGnssObsConverter::~UbloxGnssObsConverter()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UbloxGnssObsConverter::typeStatic()
{
    return "UbloxGnssObsConverter";
}

std::string NAV::UbloxGnssObsConverter::type() const
{
    return typeStatic();
}

std::string NAV::UbloxGnssObsConverter::category()
{
    return "Converter";
}

bool NAV::UbloxGnssObsConverter::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _lastEpochObs.clear();

    return true;
}

void NAV::UbloxGnssObsConverter::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto ubloxObs = std::static_pointer_cast<const UbloxObs>(queue.extract_front());

    namespace ubx = vendor::ublox;

    if (ubloxObs->msgClass == ubx::UBX_CLASS_RXM)
    {
        if (static_cast<ubx::UbxRxmMessages>(ubloxObs->msgId) == ubx::UbxRxmMessages::UBX_RXM_RAWX)
        {
            LOG_DATA("{}: Converting message at [{}]", nameId(), ubloxObs->insTime.toYMDHMS(GPST));
            auto gnssObs = std::make_shared<GnssObs>();
            gnssObs->insTime = ubloxObs->insTime;

            const auto& ubxRxmRawx = std::get<ubx::UbxRxmRawx>(ubloxObs->data);

            for (const auto& satSys : SatelliteSystem::GetAll())
            {
                std::map<SatSigId, GnssObs::ObservationData> sortedObsData;
                for (const auto& data : ubxRxmRawx.data)
                {
                    if (ubx::getSatSys(data.gnssId) != satSys) { continue; }

                    SatSigId satSigId(ubx::getCode(data.gnssId, data.sigId), data.svId);
                    LOG_DATA("{}: [{}][{}], prValid {}, cpValid {}, halfCycValid {}, subHalfSubtractedFromPhase {}, trkStat {}, observedLastEpoch {}",
                             nameId(), ubloxObs->insTime.toYMDHMS(GPST), satSigId, data.prValid(), data.cpValid(), data.halfCycValid(), data.subHalfSubtractedFromPhase(), data.trkStat,
                             _lastEpochObs.contains(satSigId));
                    GnssObs::ObservationData obsData(satSigId);
                    if (data.prValid())
                    {
                        if (data.prMes > 100'000'000) { return; } // Sometimes at the start of the ublox receiver, it reports absurd high numbers
                        obsData.pseudorange = GnssObs::ObservationData::Pseudorange{
                            .value = data.prMes,
                            .SSI = 0,
                        };
                    }
                    if (data.cpValid())
                    {
                        std::bitset<4> LLI;
                        LLI[0] = !_lastEpochObs.contains(satSigId);
                        LLI[1] = !data.halfCycValid();
                        obsData.carrierPhase = GnssObs::ObservationData::CarrierPhase{
                            .value = data.cpMes,
                            .SSI = 0,
                            .LLI = static_cast<uint8_t>(LLI.to_ulong()),
                        };
                    }
                    obsData.doppler = data.doMes;
                    obsData.CN0 = data.cno;

                    sortedObsData.insert(std::make_pair(satSigId, obsData));
                    gnssObs->satData(satSigId.toSatId()).frequencies |= satSigId.freq();
                }
                std::erase_if(_lastEpochObs, [&](const SatSigId& satSigId) { return satSigId.freq().getSatSys() == satSys; });
                for (const auto& obsData : sortedObsData)
                {
                    // LOG_DATA("{}: Adding [{}]", nameId(), obsData.second.satSigId);
                    gnssObs->data.push_back(obsData.second);
                    if (obsData.second.carrierPhase)
                    {
                        _lastEpochObs.insert(obsData.first);
                    }
                }
            }
            if (gnssObs->data.empty()) { return; }
            invokeCallbacks(OUTPUT_PORT_INDEX_GNSS_OBS, gnssObs);
        }
    }
}