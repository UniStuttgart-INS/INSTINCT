#include "CommonLog.hpp"

#include "util/Logger.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV
{

void CommonLog::initialize()
{
    if (_locked)
    {
        _referenceCounter--;
        _locked = false;

        if (_referenceCounter == 0)
        {
            LOG_DEBUG("Resetting common log variables.");
            _startTime.reset();
            _originLatitude = std::nan("");
            _originLongitude = std::nan("");
        }
    }
}

double CommonLog::calcTimeIntoRun(const InsTime& insTime)
{
    if (!_locked)
    {
        _referenceCounter++;
        _locked = true;
    }

    if (_startTime.empty())
    {
        _startTime = insTime;
        LOG_DEBUG("Common log setting start time to {} ({}).", _startTime.toYMDHMS(), _startTime.toGPSweekTow());
    }
    return static_cast<double>((insTime - _startTime).count());
}

CommonLog::LocalPosition CommonLog::calcLocalPosition(const Eigen::Vector3d& lla_position)
{
    if (!_locked)
    {
        _referenceCounter++;
        _locked = true;
    }

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