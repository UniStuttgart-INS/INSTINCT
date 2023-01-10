// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Satellite.hpp"

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
    auto iter = std::find_if(m_navigationData.begin(), m_navigationData.end(),
                             [&satNavData](const std::shared_ptr<SatNavData>& navData) { return navData->refTime == satNavData->refTime; });
    if (iter != m_navigationData.end()) // Item with this time does already exist, so we update this item
    {
        *iter = satNavData;
    }
    else // Otherwise we insert the item into the list in a time sorted way
    {
        iter = std::find_if(m_navigationData.begin(), m_navigationData.end(),
                            [&satNavData](const std::shared_ptr<SatNavData>& navData) { return navData->refTime > satNavData->refTime; });

        m_navigationData.insert(iter, satNavData);
    }
}

const std::vector<std::shared_ptr<SatNavData>>& Satellite::getNavigationData() const
{
    return m_navigationData;
}

std::shared_ptr<SatNavData> Satellite::searchNavigationData(const InsTime& time) const
{
    if (m_navigationData.size() == 1)
    {
        return m_navigationData.front();
    }

    long double prevDiff = InsTimeUtil::SECONDS_PER_WEEK;
    for (auto riter = m_navigationData.rbegin(); riter != m_navigationData.rend(); riter++)
    {
        long double diff = std::abs(((*riter)->refTime - time).count());
        if (diff < prevDiff)
        {
            prevDiff = diff;
        }
        else
        {
            return *(--riter);
        }
    }
    return m_navigationData.front();
}

} // namespace NAV
