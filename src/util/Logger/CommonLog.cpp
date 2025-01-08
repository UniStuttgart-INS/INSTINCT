// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CommonLog.hpp"

#include <imgui.h>

#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

namespace NAV
{

json CommonLog::save()
{
    json j;
    j["useGuiInputs"] = _useGuiInputs;
    if (_originPosition) { j["originPosition"] = *_originPosition; }

    return j;
}

void CommonLog::restore(const json& j)
{
    if (j.contains("useGuiInputs")) { j.at("useGuiInputs").get_to(_useGuiInputs); }
    else { _useGuiInputs = false; }
    if (j.contains("originPosition")) { _originPosition = j.at("originPosition").get<gui::widgets::PositionWithFrame>(); }
    else { _originPosition.reset(); }
}

CommonLog::CommonLog()
{
    std::scoped_lock lk(_mutex);
    _index = _wantsInit.size(); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    _wantsInit.push_back(false);
}

CommonLog::~CommonLog()
{
    std::scoped_lock lk(_mutex);
    _wantsInit.erase(_wantsInit.begin() + static_cast<int64_t>(_index));
}

void CommonLog::initialize() const
{
    std::scoped_lock lk(_mutex);
    if (_useGuiInputs) { return; }
    _wantsInit.at(_index) = true;

    if (std::ranges::all_of(_wantsInit, [](bool val) { return val; }))
    {
        LOG_DEBUG("Resetting common log variables.");
        _startTime.reset();
        if (!_useGuiInputs)
        {
            _originPosition.reset();
        }

        std::ranges::fill(_wantsInit, false);
    }
}

double CommonLog::calcTimeIntoRun(const InsTime& insTime)
{
    if (std::scoped_lock lk(_mutex);
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
        std::scoped_lock lk(_mutex);
        if (!_originPosition.has_value())
        {
            _originPosition = gui::widgets::PositionWithFrame();
            _originPosition->e_position = trafo::lla2ecef_WGS84(lla_position);
            LOG_DEBUG("Common log setting position of origin to {}, {}, {} [deg, deg, m]",
                      rad2deg(lla_position.x()), rad2deg(lla_position.y()), lla_position.z());
        }
    }

    // North/South deviation [m]
    double northSouth = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                                 _originPosition->latitude(), lla_position.y())
                        * (lla_position.x() > _originPosition->latitude() ? 1 : -1);

    // East/West deviation [m]
    double eastWest = calcGeographicalDistance(lla_position.x(), lla_position.y(),
                                               lla_position.x(), _originPosition->longitude())
                      * (lla_position.y() > _originPosition->longitude() ? 1 : -1);

    return { .northSouth = northSouth, .eastWest = eastWest };
}

bool CommonLog::ShowOriginInput(const char* id)
{
    bool changed = false;
    if (ImGui::Checkbox(fmt::format("Override position origin (for all common logging)##{}", id).c_str(), &_useGuiInputs))
    {
        LOG_DEBUG("{}: useGuiInputs changed to {}", id, _useGuiInputs);
        changed = true;
    }
    if (_useGuiInputs)
    {
        if (!_originPosition) { _originPosition = gui::widgets::PositionWithFrame(); }
        ImGui::Indent();
        std::scoped_lock lk(_mutex);
        if (gui::widgets::PositionInput(fmt::format("Origin##{}", id).c_str(), _originPosition.value(), gui::widgets::PositionInputLayout::SINGLE_ROW))
        {
            changed = true;
        }
        ImGui::Unindent();
    }

    return changed;
}

} // namespace NAV