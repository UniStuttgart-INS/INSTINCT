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

#include "internal/gui/widgets/TimeEdit.hpp"
#include "util/Logger.hpp"

namespace NAV
{

json CommonLog::save()
{
    std::scoped_lock lk(_mutex);
    json j;
    j["useGuiInputs"] = _useGuiInputs;
    j["overrideStartTime"] = _overrideStartTime;
    if (_overrideStartTime && !_startTime.empty()) { j["startTime"] = _startTime; }
    j["startTimeFormat"] = _startTimeFormat;
    if (_originPosition) { j["originPosition"] = *_originPosition; }

    return j;
}

void CommonLog::restore(const json& j)
{
    std::scoped_lock lk(_mutex);
    if (j.contains("useGuiInputs")) { j.at("useGuiInputs").get_to(_useGuiInputs); }
    else
    {
        _useGuiInputs = false;
    }
    if (j.contains("originPosition")) { _originPosition = j.at("originPosition").get<gui::widgets::PositionWithFrame>(); }
    else
    {
        _originPosition.reset();
    }

    if (j.contains("overrideStartTime")) { j.at("overrideStartTime").get_to(_overrideStartTime); }
    else
    {
        _overrideStartTime = false;
    }

    if (j.contains("startTime")) { j.at("startTime").get_to(_startTime); }
    else
    {
        _startTime.reset();
    }

    if (j.contains("startTimeFormat")) { j.at("startTimeFormat").get_to(_startTimeFormat); }
}

CommonLog::CommonLog()
{
    std::scoped_lock lk(_mutex);
    _index = _wantsInit.size(); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    _wantsInit.emplace_back(false);
}

CommonLog::~CommonLog()
{
    std::scoped_lock lk(_mutex);
    _wantsInit.at(_index).reset();
}

void CommonLog::initialize() const
{
    std::scoped_lock lk(_mutex);
    if (_useGuiInputs && _overrideStartTime) { return; }
    _wantsInit.at(_index) = true;

    if (std::ranges::all_of(_wantsInit, [](std::optional<bool> val) { return !val.has_value() || (*val); }))
    {
        LOG_DEBUG("Resetting common log variables.");
        if (!_overrideStartTime) { _startTime.reset(); }
        if (!_useGuiInputs) { _originPosition.reset(); }

        for (auto& init : _wantsInit)
        {
            if (init) { init = false; }
        }
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
            if (lla_position.hasNaN())
            {
                LOG_WARN("Not setting common log origin to {}, {}, {} [deg, deg, m]",
                         rad2deg(lla_position.x()), rad2deg(lla_position.y()), lla_position.z());
            }
            else
            {
                _originPosition = gui::widgets::PositionWithFrame();
                _originPosition->e_position = trafo::lla2ecef_WGS84(lla_position);
                LOG_DEBUG("Common log setting position of origin to {}, {}, {} [deg, deg, m]",
                          rad2deg(lla_position.x()), rad2deg(lla_position.y()), lla_position.z());
            }
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
    if (ImGui::Checkbox(fmt::format("Override start time (for all common logging)##{}", id).c_str(), &_overrideStartTime))
    {
        LOG_DEBUG("{}: overrideStartTime changed to {}", id, _overrideStartTime);
        changed = true;
    }
    if (_overrideStartTime)
    {
        ImGui::Indent();
        std::scoped_lock lk(_mutex);
        if (gui::widgets::TimeEdit(fmt::format("Start time##{}", id).c_str(), _startTime, _startTimeFormat, 170.0F, 2))
        {
            changed = true;
        }
        ImGui::Unindent();
    }

    return changed;
}

} // namespace NAV