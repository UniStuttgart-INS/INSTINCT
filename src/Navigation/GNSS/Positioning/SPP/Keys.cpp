// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Keys.cpp
/// @brief Keys for the SPP algorithm for use inside the KeyedMatrices
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-12

#include "Keys.hpp"

#include <iostream>

std::ostream& operator<<(std::ostream& os, const NAV::SPP::States::SppStates& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::SPP::States::InterSysBias& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::SPP::States::InterSysDrift& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::SPP::States::InterFreqBias& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Meas::Psr& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Meas::Doppler& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::SPP::States::StateKeyTypes& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::SPP::Meas::MeasKeyTypes& obj)
{
    return os << fmt::format("{}", obj);
}