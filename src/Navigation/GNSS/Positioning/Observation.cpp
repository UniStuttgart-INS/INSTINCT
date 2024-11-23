#include "Observation.hpp"
#include <array>
#include <cstddef>
#include <unordered_set>

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

namespace NAV
{

size_t Observations::countObservations(const SatSigId& satSigId, const GnssObs::ObservationType& obsType) const
{
    if (!signals.contains(satSigId)) { return 0; }

    const auto& recvObs = signals.at(satSigId).recvObs;
    return static_cast<size_t>(std::ranges::count_if(recvObs, [&obsType](const auto& obs) {
        return obs.second->obs.contains(obsType);
    }));
}

void Observations::recalcObservableCounts([[maybe_unused]] const std::string& nameId)
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

        const auto& recvObs = sigObs.recvObs.begin();
        for (const auto& obs : recvObs->second->obs)
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

#if LOG_LEVEL <= LOG_LEVEL_DATA
    LOG_DATA("{}: systems: [{}]", nameId, fmt::join(systems, ", "));
    LOG_DATA("{}: satellites: [{}]", nameId, fmt::join(satellites, ", "));
    std::string observables;
    std::string observablesUnique;
    for (size_t obsType = 0; obsType < GnssObs::ObservationType_COUNT; obsType++)
    {
        observables += fmt::format("{} {}", nObservables.at(obsType), static_cast<GnssObs::ObservationType>(obsType));
        observablesUnique += fmt::format("{} {} [{}]", nObservablesUniqueSatellite.at(obsType),
                                         static_cast<GnssObs::ObservationType>(obsType), fmt::join(nMeasUniqueSat.at(obsType), ", "));
        if (obsType < GnssObs::ObservationType_COUNT - 1)
        {
            observables += ", ";
            observablesUnique += ", ";
        }
    }
    LOG_DATA("{}: nObservables: {}", nameId, observables);
    LOG_DATA("{}: nObservablesUniqueSatellite: {}", nameId, observablesUnique);
#endif
}

bool Observations::removeSignal(const SatSigId& satSigId, [[maybe_unused]] const std::string& nameId)
{
    bool somethingRemoved = false;
    if (signals.contains(satSigId))
    {
        LOG_DATA("{}: Erasing signal [{}]", nameId, satSigId);
        signals.erase(satSigId);
        somethingRemoved = true;
    }
    if (somethingRemoved) { recalcObservableCounts(nameId); }
    return somethingRemoved;
}

bool Observations::removeSignal(const SatSigId& satSigId, const GnssObs::ObservationType& obsType, [[maybe_unused]] const std::string& nameId)
{
    bool somethingRemoved = false;
    if (signals.contains(satSigId))
    {
        LOG_DATA("{}: Erasing signal [{}][{}]", nameId, satSigId, obsType);
        size_t emptyReceiver = 0;
        for (auto& recvObs : signals.at(satSigId).recvObs)
        {
            if (recvObs.second->obs.contains(obsType))
            {
                recvObs.second->obs.erase(obsType);
            }
            if (recvObs.second->obs.empty()) { emptyReceiver++; }
        }
        if (emptyReceiver == signals.at(satSigId).recvObs.size())
        {
            signals.erase(satSigId);
        }
        somethingRemoved = true;
    }
    if (somethingRemoved) { recalcObservableCounts(nameId); }
    return somethingRemoved;
}

bool Observations::removeSatellite(const SatId& satId, [[maybe_unused]] const std::string& nameId)
{
    std::vector<SatSigId> toRemove;
    for (const auto& signal : signals)
    {
        if (signal.first.toSatId() == satId)
        {
            toRemove.push_back(signal.first);
        }
    }
    for (const auto& satSigId : toRemove)
    {
        LOG_DATA("{}: Erasing signal [{}]", nameId, satSigId);
        signals.erase(satSigId);
    }
    if (!toRemove.empty()) { recalcObservableCounts(nameId); }
    return !toRemove.empty();
}

} // namespace NAV