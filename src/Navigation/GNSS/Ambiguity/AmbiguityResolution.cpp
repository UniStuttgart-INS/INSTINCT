// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AmbiguityResolution.cpp
/// @brief Ambiguity resolution algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-20

#include "AmbiguityResolution.hpp"
#include <imgui.h>

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/EnumComboWithAbbreviation.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

namespace NAV
{

const char* to_string(AmbiguityResolutionStrategy ambiguityResolutionStrategy)
{
    switch (ambiguityResolutionStrategy)
    {
    case AmbiguityResolutionStrategy::Continuous:
        return "Continuous";
    case AmbiguityResolutionStrategy::FixAndHold:
        return "Fix and Hold";
    case AmbiguityResolutionStrategy::COUNT:
        break;
    }
    return "";
}

const char* to_string(AmbiguityResolutionParameters::DecorrelationAlgorithm decorrelationAlgorithm)
{
    switch (decorrelationAlgorithm)
    {
    case AmbiguityResolutionParameters::DecorrelationAlgorithm::None:
        return "None";
    case AmbiguityResolutionParameters::DecorrelationAlgorithm::Z_Transformation:
        return "Z-Transformation";
    case AmbiguityResolutionParameters::DecorrelationAlgorithm::COUNT:
        break;
    }
    return "";
}

const char* to_string(AmbiguityResolutionParameters::SearchAlgorithm searchAlgorithm)
{
    switch (searchAlgorithm)
    {
    case AmbiguityResolutionParameters::SearchAlgorithm::None:
        return "None";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerRounding:
        return "Integer Rounding (IR)";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerBootstrapping:
        return "Integer Bootstrapping (IB)";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearch:
        return "Integer least-squares (ILS) Search (LAMBDA)";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearchAndShrink:
        return "Integer least-squares (ILS) Search-And-Shrink (MLAMBDA)";
    case AmbiguityResolutionParameters::SearchAlgorithm::COUNT:
        break;
    }
    return "";
}

const char* to_string_short(AmbiguityResolutionParameters::SearchAlgorithm searchAlgorithm)
{
    switch (searchAlgorithm)
    {
    case AmbiguityResolutionParameters::SearchAlgorithm::None:
        return "None";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerRounding:
        return "Integer Rounding (IR)";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerBootstrapping:
        return "Integer Bootstrapping (IB)";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearch:
        return "ILS Search (LAMBDA)";
    case AmbiguityResolutionParameters::SearchAlgorithm::IntegerLeastSquaresSearchAndShrink:
        return "ILS Search-And-Shrink (MLAMBDA)";
    case AmbiguityResolutionParameters::SearchAlgorithm::COUNT:
        break;
    }
    return "";
}

const char* to_string(AmbiguityResolutionParameters::ValidationAlgorithm validationAlgorithm)
{
    switch (validationAlgorithm)
    {
    case AmbiguityResolutionParameters::ValidationAlgorithm::None:
        return "None";
    case AmbiguityResolutionParameters::ValidationAlgorithm::DifferenceTest:
        return "Difference Test";
    case AmbiguityResolutionParameters::ValidationAlgorithm::ProjectorTest:
        return "Projector Test";
    case AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestCriticalValue:
        return "Ratio Test (critical value)";
    case AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestFailureRate:
        return "Ratio Test (failure rate)";
    case AmbiguityResolutionParameters::ValidationAlgorithm::COUNT:
        break;
    }
    return "";
}

bool GuiAmbiguityResolution(const char* id, AmbiguityResolutionParameters& params, float width)
{
    bool changed = false;

    if (params.searchAlgorithm == AmbiguityResolutionParameters::SearchAlgorithm::None) { ImGui::BeginDisabled(); }
    ImGui::SetNextItemWidth(width - ImGui::GetStyle().IndentSpacing);
    changed |= gui::widgets::EnumCombo(fmt::format("Decorrelation Algorithm##{}", id).c_str(), params.decorrelationAlgorithm);
    if (params.searchAlgorithm == AmbiguityResolutionParameters::SearchAlgorithm::None) { ImGui::EndDisabled(); }

    ImGui::SetNextItemWidth(width - ImGui::GetStyle().IndentSpacing);
    changed |= gui::widgets::EnumComboAbbreviation(fmt::format("Search Algorithm##{}", id).c_str(), params.searchAlgorithm);

    changed |= ImGui::Checkbox(fmt::format("Partial fixing##{}", id).c_str(), &params.partialFixing);

    if (params.searchAlgorithm == AmbiguityResolutionParameters::SearchAlgorithm::None) { ImGui::BeginDisabled(); }
    {
        ImGui::SetNextItemWidth(width - ImGui::GetStyle().IndentSpacing);
        changed |= gui::widgets::EnumCombo(fmt::format("Validation Algorithm##{}", id).c_str(), params.validationAlgorithm);

        if (params.validationAlgorithm == AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestFailureRate)
        {
            changed |= ImGui::Checkbox(fmt::format("Validate with Bootstrapped Upper Bound##{}", id).c_str(), &params.validationBootstrappedSuccessRate);
            ImGui::SameLine();
            gui::widgets::HelpMarker("Bootstrapped failure rate is an upper bound for the ILS failure rate\n"
                                     "and can be analytically calculated. If the bootstrapped failure rate is\n"
                                     "smaller than the selected failure rate, the ratio test can be skipped.");
        }

        ImGui::SetNextItemWidth(width - ImGui::GetStyle().IndentSpacing);
        if (params.validationAlgorithm == AmbiguityResolutionParameters::ValidationAlgorithm::DifferenceTest)
        {
            changed |= ImGui::InputDouble(fmt::format("Critical Value c (R2 - R1 ≥ c)##{}", id).c_str(), &params.validationTestCriticalValueC, 0.0, 0.0, "%.3g");
        }
        else if (params.validationAlgorithm == AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestCriticalValue)
        {
            changed |= ImGui::InputDoubleL(fmt::format("Critical Value µ ∈ (0, 1] (R1/R2 ≤ µ)##{}", id).c_str(), &params.validationTestCriticalValueMu, 1e-10, 1.0, 0.0, 0.0, "%.3g");
        }
        else if (params.validationAlgorithm == AmbiguityResolutionParameters::ValidationAlgorithm::ProjectorTest)
        {
            changed |= ImGui::InputDoubleL(fmt::format("Critical Value µ ∈ (0, 1]##{}", id).c_str(), &params.validationTestCriticalValueMu, 1e-10, 1.0, 0.0, 0.0, "%.3g");
        }
        else if (params.validationAlgorithm == AmbiguityResolutionParameters::ValidationAlgorithm::RatioTestFailureRate)
        {
            int item_current = 0;
            for (size_t i = 0; i < AmbiguityResolutionParameters::allowedFailureRateValues.size(); i++)
            {
                if (params.validationRatioTestFailureRate == AmbiguityResolutionParameters::allowedFailureRateValues.at(i))
                {
                    item_current = static_cast<int>(i);
                    break;
                }
            }
            if (ImGui::Combo(fmt::format("Failure rate##{}", id).c_str(), &item_current, "0.1 %\0001 %\0\0"))
            {
                params.validationRatioTestFailureRate = AmbiguityResolutionParameters::allowedFailureRateValues.at(static_cast<size_t>(item_current));
                changed = true;
            }
        }
    }
    if (params.searchAlgorithm == AmbiguityResolutionParameters::SearchAlgorithm::None) { ImGui::EndDisabled(); }

    return changed;
}

void to_json(json& j, const AmbiguityResolutionParameters& obj)
{
    j = json{
        { "decorrelationAlgorithm", obj.decorrelationAlgorithm },
        { "searchAlgorithm", obj.searchAlgorithm },
        { "partialFixing", obj.partialFixing },
        { "validationBootstrappedSuccessRate", obj.validationBootstrappedSuccessRate },
        { "validationAlgorithm", obj.validationAlgorithm },
        { "validationTestCriticalValueC", obj.validationTestCriticalValueC },
        { "validationTestCriticalValueMu", obj.validationTestCriticalValueMu },
        { "validationRatioTestFailureRate", obj.validationRatioTestFailureRate },
    };
}

void from_json(const json& j, AmbiguityResolutionParameters& obj)
{
    if (j.contains("decorrelationAlgorithm")) { j.at("decorrelationAlgorithm").get_to(obj.decorrelationAlgorithm); }
    if (j.contains("searchAlgorithm")) { j.at("searchAlgorithm").get_to(obj.searchAlgorithm); }
    if (j.contains("partialFixing")) { j.at("partialFixing").get_to(obj.partialFixing); }
    if (j.contains("validationBootstrappedSuccessRate")) { j.at("validationBootstrappedSuccessRate").get_to(obj.validationBootstrappedSuccessRate); }
    if (j.contains("validationAlgorithm")) { j.at("validationAlgorithm").get_to(obj.validationAlgorithm); }
    if (j.contains("validationTestCriticalValueC")) { j.at("validationTestCriticalValueC").get_to(obj.validationTestCriticalValueC); }
    if (j.contains("validationTestCriticalValueMu")) { j.at("validationTestCriticalValueMu").get_to(obj.validationTestCriticalValueMu); }
    if (j.contains("validationRatioTestFailureRate")) { j.at("validationRatioTestFailureRate").get_to(obj.validationRatioTestFailureRate); }
}

} // namespace NAV
