// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SatNavData.hpp"

namespace NAV
{

SatNavData::SatNavData(Type type, const InsTime& refTime)
    : type(type), refTime(refTime) {}

} // namespace NAV