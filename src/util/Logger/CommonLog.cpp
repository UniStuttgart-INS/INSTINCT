// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CommonLog.hpp"

#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

namespace NAV
{

CommonLog::CommonLog()
{
    std::lock_guard lk(_mutex);
    _index = _wantsInit.size(); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    _wantsInit.push_back(false);
}

CommonLog::~CommonLog()
{
    std::lock_guard lk(_mutex);
    _wantsInit.erase(_wantsInit.begin() + static_cast<int64_t>(_index));
}

void CommonLog::initialize() const
{
    std::lock_guard lk(_mutex);
    _wantsInit.at(_index) = true;

    if (std::all_of(_wantsInit.begin(), _wantsInit.end(), [](bool val) { return val; }))
    {
        LOG_DEBUG("Resetting common log variables.");
        _startTime.reset();
        _originLatitude = std::nan("");
        _originLongitude = std::nan("");

        std::fill(_wantsInit.begin(), _wantsInit.end(), false);
    }
}

double CommonLog::calcTimeIntoRun(const InsTime& insTime)
{
    if (std::lock_guard lk(_mutex);
        _startTime.empty())
    {
        _startTime = insTime;
        LOG_DEBUG("Common log setting start time to {} ({}) GPST.", _startTime.toYMDHMS(GPST), _startTime.toGPSweekTow(GPST));
    }
    return static_cast<double>((insTime - _startTime).count());
}

CommonLog::LocalPosition CommonLog::calcLocalPosition(const Eigen::Vector3d& lla_position)
{
    {
        std::lock_guard lk(_mutex);
        if (std::isnan(_originLatitude))
        {
            _originLatitude = lla_position.x();
            LOG_DEBUG("Common log setting latitude of origin to {} [deg].", rad2deg(_originLatitude));
        }
        if (std::isnan(_originLongitude))
        {
            _originLongitude = lla_position.y();
            LOG_DEBUG("Common log setting longitude of origin to {} [deg].", rad2deg(_originLongitude));
        }
    }

    // North/South deviation [m]
    double northSouth = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                                 _originLatitude, lla_position.y())
                        * (lla_position.x() > _originLatitude ? 1 : -1);

    // East/West deviation [m]
    double eastWest = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                               lla_position.x(), _originLongitude)
                      * (lla_position.y() > _originLongitude ? 1 : -1);

    return { .northSouth = northSouth, .eastWest = eastWest };
}

} // namespace NAV