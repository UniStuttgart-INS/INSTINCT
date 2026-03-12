// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Keys.hpp"

std::ostream& operator<<(std::ostream& os, const NAV::Keys::BarometerModelKey& obj)
{
    return os << fmt::format("{}", obj);
}

std::ostream& operator<<(std::ostream& os, const NAV::MeasKeys::BaroHeightDiff& obj)
{
    return os << fmt::format("{}", obj);
}