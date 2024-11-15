// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Satellite.hpp"

#include <limits>

namespace NAV
{

Orbit::Pos Satellite::calcSatellitePos(const InsTime& transTime) const
{
    return searchNavigationData(transTime)->calcSatellitePos(transTime);
}

Orbit::PosVel Satellite::calcSatellitePosVel(const InsTime& transTime) const
{
    return searchNavigationData(transTime)->calcSatellitePosVel(transTime);
}

Orbit::PosVelAccel Satellite::calcSatellitePosVelAccel(const InsTime& transTime) const
{
    return searchNavigationData(transTime)->calcSatellitePosVelAccel(transTime);
}

double Satellite::calcSatellitePositionVariance(const InsTime& recvTime) const
{
    return searchNavigationData(recvTime)->calcSatellitePositionVariance();
}

[[nodiscard]] Clock::Corrections Satellite::calcClockCorrections(const InsTime& recvTime, double dist, const Frequency& freq) const
{
    return searchNavigationData(recvTime)->calcClockCorrections(recvTime, dist, freq);
}

bool Satellite::isHealthy(const InsTime& recvTime) const
{
    return searchNavigationData(recvTime)->isHealthy();
}

void Satellite::addSatNavData(const std::shared_ptr<SatNavData>& satNavData)
{
    // First check if item with same time already exists
    auto iter = std::ranges::find_if(m_navigationData, [&satNavData](const std::shared_ptr<SatNavData>& navData) {
        return navData->refTime == satNavData->refTime;
    });
    if (iter != m_navigationData.end()) // Item with this time does already exist, so we update this item
    {
        *iter = satNavData;
    }
    else // Otherwise we insert the item into the list in a time sorted way
    {
        iter = std::ranges::find_if(m_navigationData, [&satNavData](const std::shared_ptr<SatNavData>& navData) {
            return navData->refTime > satNavData->refTime;
        });

        m_navigationData.insert(iter, satNavData);
    }
}

const std::vector<std::shared_ptr<SatNavData>>& Satellite::getNavigationData() const
{
    return m_navigationData;
}

std::shared_ptr<SatNavData> Satellite::searchNavigationData(const InsTime& time) const
{
    if (m_navigationData.empty()) { return nullptr; }

    auto prevDiff = std::numeric_limits<long double>::max();
    // auto diff = prevDiff;

    auto riter = m_navigationData.rbegin();
    for (; riter != m_navigationData.rend(); riter++)
    {
        auto diff = std::abs(((*riter)->refTime - time).count());
        if (diff < prevDiff)
        {
            prevDiff = diff;
        }
        else
        {
            // diff = prevDiff;
            break;
        }
    }

    // This limits the time the navigation data can be used
    // switch (m_navigationData.front()->type)
    // {
    // case NAV::SatNavData::Type::GPSEphemeris:
    // case NAV::SatNavData::Type::GalileoEphemeris:
    // case NAV::SatNavData::Type::BeiDouEphemeris:
    // case NAV::SatNavData::Type::IRNSSEphemeris:
    // case NAV::SatNavData::Type::QZSSEphemeris:
    //     if (diff > InsTimeUtil::SECONDS_PER_HOUR * 2 + 1e-6)
    //     {
    //         return nullptr;
    //     }
    //     break;
    // case NAV::SatNavData::Type::GLONASSEphemeris:
    // case NAV::SatNavData::Type::SBASEphemeris:
    //     if (diff > InsTimeUtil::SECONDS_PER_MINUTE * 15 + 1e-6)
    //     {
    //         return nullptr;
    //     }
    //     break;
    // }

    return *(--riter);
}

} // namespace NAV
