// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file IonosphericCorrections.cpp
/// @brief Ionospheric corrections
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-23

#include "IonosphericCorrections.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"

namespace NAV
{

IonosphericCorrections::IonosphericCorrections(const std::vector<const GnssNavInfo*>& gnssNavInfos)
{
    for (const auto* gnssNavInfo : gnssNavInfos)
    {
        for (const auto& correction : gnssNavInfo->ionosphericCorrections.data())
        {
            if (!this->contains(correction.satSys, correction.alphaBeta))
            {
                this->insert(correction.satSys, correction.alphaBeta, correction.data);
            }
        }
    }
}

IonosphericCorrections::IonosphericCorrections(const std::vector<Corrections>& corrections)
    : m_ionosphericCorrections(corrections) {}

} // namespace NAV