#include "Observation.hpp"
#include <array>
#include <cstddef>
#include <unordered_set>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

namespace NAV
{

void Observations::recalcObservableCounts()
{
    std::array<std::unordered_set<SatId>, GnssObs::ObservationType_COUNT> nMeasUniqueSat;
    systems.clear();
    satellites.clear();
    nObservables.fill(0);
    nObservablesUniqueSatellite.fill(0);

    for (const auto& [satSigId, sigObs] : signals)
    {
        auto satId = satSigId.toSatId();
        satellites.insert(satId);
        systems.insert(satId.satSys);

        const auto& recvObs = sigObs.recvObs.front();
        for (const auto& obs : recvObs.obs)
        {
            auto obsType = static_cast<size_t>(obs.first);
            nObservables.at(obsType)++;
            nMeasUniqueSat.at(obsType).insert(satId);
        }
    }

    for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
    {
        nObservablesUniqueSatellite.at(obsType) = nMeasUniqueSat.at(obsType).size();
    }
}

} // namespace NAV