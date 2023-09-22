// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ColormapTests.cpp
/// @brief Colormap tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-22

#include <catch2/catch_test_macros.hpp>
#include "Logger.hpp"

#include <imgui.h>
#include <fmt/format.h>
#include "util/Plot/Colormap.hpp"

constexpr bool operator==(const ImColor& lhs, const ImColor& rhs)
{
    return lhs.Value.x == rhs.Value.x && lhs.Value.y == rhs.Value.y && lhs.Value.z == rhs.Value.z && lhs.Value.w == rhs.Value.w;
}
namespace Catch
{
template<>
struct StringMaker<std::pair<double, ImVec4>>
{
    static std::string convert(const std::pair<double, ImVec4>& value)
    {
        return fmt::format("{}, [{}, {}, {}, {}] ", value.first, value.second.x, value.second.y, value.second.z, value.second.w);
    }
};
} // namespace Catch

namespace NAV::TESTS
{

TEST_CASE("[Colormap] Add/Remove colors", "[Colormap]")
{
    auto logger = initializeTestLogger();

    Colormap cm;

    cm.addColor(1.0, ImVec4(1.0, 1.0, 1.0, 1.0));
    cm.addColor(3.0, ImVec4(3.0, 1.0, 1.0, 1.0));
    cm.addColor(2.0, ImVec4(2.0, 1.0, 1.0, 1.0));

    REQUIRE(cm.getColormap().size() == 3);
    REQUIRE(cm.getColormap().at(0) == std::make_pair(1.0, ImColor(1.0F, 1.0F, 1.0F, 1.0F)));
    REQUIRE(cm.getColormap().at(1) == std::make_pair(2.0, ImColor(2.0F, 1.0F, 1.0F, 1.0F)));
    REQUIRE(cm.getColormap().at(2) == std::make_pair(3.0, ImColor(3.0F, 1.0F, 1.0F, 1.0F)));

    cm.removeColor(1);
    REQUIRE(cm.getColormap().size() == 2);
    REQUIRE(cm.getColormap().at(0) == std::make_pair(1.0, ImColor(1.0F, 1.0F, 1.0F, 1.0F)));
    REQUIRE(cm.getColormap().at(1) == std::make_pair(3.0, ImColor(3.0F, 1.0F, 1.0F, 1.0F)));

    cm.removeColor(3); // NoOp
    REQUIRE(cm.getColormap().size() == 2);
    REQUIRE(cm.getColormap().at(0) == std::make_pair(1.0, ImColor(1.0F, 1.0F, 1.0F, 1.0F)));
    REQUIRE(cm.getColormap().at(1) == std::make_pair(3.0, ImColor(3.0F, 1.0F, 1.0F, 1.0F)));

    cm.removeColor(0);
    cm.removeColor(0);
    REQUIRE(cm.getColormap().empty());

    cm.removeColor(0);
}

} // namespace NAV::TESTS