// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SystemModel.hpp"

#include <iostream>
#include <fmt/format.h>
#include <imgui.h>

#include "MotionModel.hpp"
#include "InterFrequencyBiasModel.hpp"
#include "ReceiverClockModel.hpp"

#include "util/Logger.hpp"

bool NAV::SystemModelGui(SystemModelCalcAlgorithm& algorithm, float itemWidth, const char* id)
{
    bool changed = false;

    ImGui::SetNextItemWidth(itemWidth);
    if (ImGui::Combo(fmt::format("Q calculation algorithm##{}", id).c_str(), reinterpret_cast<int*>(&algorithm),
                     "Van Loan\0Taylor 1st Order (Groves 2013)\0\0"))
    {
        LOG_DEBUG("{}: Q calculation algorithm changed to {}", id, fmt::underlying(algorithm));
        changed = true;
    }

    return changed;
}

std::ostream& operator<<(std::ostream& os, const NAV::Keys::MotionModelKey& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::Keys::RecvClkBias& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::Keys::RecvClkDrift& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::Keys::InterFreqBias& obj)
{
    return os << fmt::format("{}", obj);
}