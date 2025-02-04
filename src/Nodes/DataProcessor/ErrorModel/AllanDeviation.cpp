// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "AllanDeviation.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <imgui.h>
#include <mutex>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
#include <Eigen/src/Core/util/Meta.h>
#include <fmt/core.h>
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "implot.h"

#include "NodeData/IMU/ImuObs.hpp"

#include <chrono>

#include <unsupported/Eigen/MatrixFunctions>

NAV::AllanDeviation::AllanDeviation()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _lockConfigDuringRun = false;
    _guiConfigDefaultWindowSize = { 630, 530 };

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &AllanDeviation::receiveImuObs);
}

NAV::AllanDeviation::~AllanDeviation()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::AllanDeviation::typeStatic()
{
    return "AllanDeviation";
}

std::string NAV::AllanDeviation::type() const
{
    return typeStatic();
}

std::string NAV::AllanDeviation::category()
{
    return "Data Processor";
}

void NAV::AllanDeviation::guiConfig()
{
    if (ImGui::Checkbox("Compute Allan Deviation last", &_updateLast)) { flow::ApplyChanges(); }
    if (ImGui::Checkbox("Display Confidence Intervals", &_displayConfidence)) { flow::ApplyChanges(); }
    if (_displayConfidence)
    {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(200.0F);
        if (ImGui::SliderFloat("Confidence Alpha Channel", &_confidenceFillAlpha, 0.0F, 1.0F, "%.2f"))
        {
            flow::ApplyChanges();
        }
    }

    const std::array<std::string, 3> legendEntries{ "x", "y", "z" };
    constexpr std::array<double, 5> slopeTicks = { -2, -1, 0, 1, 2 };

    if (ImGui::BeginTabBar("AllanDeviationTabBar", ImGuiTabBarFlags_None))
    {
        for (size_t sensorIter = 0; sensorIter < SensorType_COUNT; sensorIter++)
        {
            auto sensorType = static_cast<SensorType>(sensorIter);
            const char* sensorName = to_string(sensorType);
            auto& sensor = _sensors.at(sensorIter);
            std::scoped_lock<std::mutex> lg(sensor.mutex);

            if (ImGui::BeginTabItem(fmt::format("{}##TabItem {}", sensorName, size_t(id)).c_str()))
            {
                if (ImPlot::BeginPlot(fmt::format("Allan Deviation of {}##Plot {}", sensorName, size_t(id)).c_str()))
                {
                    ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                    ImPlot::SetupAxes("τ [s]", fmt::format("σ(τ) [{}]", unitString(sensorType)).c_str(),
                                      ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                    ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
                    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, _confidenceFillAlpha);
                    for (size_t d = 0; d < 3; d++)
                    {
                        if (_displayConfidence & !_averagingTimes.empty())
                        {
                            ImPlot::PlotShaded(legendEntries.at(d).data(),
                                               _averagingTimes.data(),
                                               sensor.allanDeviationConfidenceIntervals.at(d).at(0).data(),
                                               sensor.allanDeviationConfidenceIntervals.at(d).at(1).data(),
                                               static_cast<int>(_averagingTimes.size()));
                        }
                        ImPlot::PlotLine(legendEntries.at(d).data(),
                                         _averagingTimes.data(),
                                         sensor.allanDeviation.at(d).data(),
                                         static_cast<int>(_averagingTimes.size()));
                    }
                    ImPlot::EndPlot();
                }
                if (ImGui::TreeNode("Slopes"))
                {
                    if (ImPlot::BeginPlot("Slopes of Allan Variance"))
                    {
                        ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_None);
                        ImPlot::SetupAxis(ImAxis_X1, "τ [s]", ImPlotAxisFlags_AutoFit);
                        ImPlot::SetupAxis(ImAxis_Y1, "µ [ ]", ImPlotAxisFlags_Lock);
                        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, -2.5, 2.5);
                        ImPlot::SetupAxisTicks(ImAxis_Y1, slopeTicks.data(), 5, nullptr, false);
                        for (size_t d = 0; d < 3; d++)
                        {
                            ImPlot::PlotLine(legendEntries.at(d).data(),
                                             _averagingTimes.data(),
                                             sensor.slope.at(d).data(),
                                             static_cast<int>(_averagingTimes.size()));
                        }
                        ImPlot::EndPlot();
                    }

                    ImGui::TreePop();
                }
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
}

[[nodiscard]] json NAV::AllanDeviation::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "displayConfidence", _displayConfidence },
        { "confidenceFillAlpha", _confidenceFillAlpha },
        { "updateLast", _updateLast },
    };
}

void NAV::AllanDeviation::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("displayConfidence")) { j.at("displayConfidence").get_to(_displayConfidence); }
    if (j.contains("confidenceFillAlpha")) { j.at("confidenceFillAlpha").get_to(_confidenceFillAlpha); }
    if (j.contains("updateLast")) { j.at("updateLast").get_to(_updateLast); }
}

bool NAV::AllanDeviation::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _startingInsTime.reset();

    for (auto& sensor : _sensors)
    {
        std::scoped_lock<std::mutex> lg(sensor.mutex);
        sensor.cumSum = { Eigen::Vector3d::Zero() };
        sensor.allanSum = {};
        sensor.allanVariance = {};
        sensor.allanDeviation = {};
        sensor.slope = {};
        sensor.allanDeviationConfidenceIntervals = {};
    }

    _averagingFactors.clear();
    _averagingTimes.clear();
    _observationCount.clear();

    _imuObsCount = 0;

    _nextAveragingFactorExponent = 1;
    _nextAveragingFactor = 1;
    _samplingInterval = 0.0;

    return true;
}

void NAV::AllanDeviation::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::AllanDeviation::receiveImuObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    bool lastMessage = false;
    if (queue.empty()
        && inputPins[INPUT_PORT_INDEX_IMU_OBS].isPinLinked()
        && inputPins[INPUT_PORT_INDEX_IMU_OBS].link.getConnectedPin()->noMoreDataAvailable)
    {
        lastMessage = true;
    }

    // save InsTime of first imuObs for sampling interval computation
    if (_startingInsTime.empty()) { _startingInsTime = obs->insTime; }

    _imuObsCount++;
    {
        std::scoped_lock<std::mutex> lg(_sensors.at(Accel).mutex);
        _sensors.at(Accel).cumSum.emplace_back(_sensors.at(Accel).cumSum.back() + obs->p_acceleration);
    }
    {
        std::scoped_lock<std::mutex> lg(_sensors.at(Gyro).mutex);
        _sensors.at(Gyro).cumSum.emplace_back(_sensors.at(Gyro).cumSum.back() + obs->p_angularRate);
    }

    // extending _averagingFactors if necessary
    if (_imuObsCount == _nextAveragingFactor * 2)
    {
        _averagingFactors.push_back(_nextAveragingFactor);
        _observationCount.push_back(0);

        for (auto& sensor : _sensors)
        {
            for (size_t d = 0; d < 3; d++)
            {
                sensor.allanSum.at(d).push_back(0);
            }
        }

        // computation of next averaging factor
        while (static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade))) == _nextAveragingFactor)
        {
            _nextAveragingFactorExponent++;
        }

        _nextAveragingFactor = static_cast<unsigned int>(round(pow(10., static_cast<double>(_nextAveragingFactorExponent) / _averagingFactorsPerDecade)));
    }

    // computation Allan sum
    for (size_t k = 0; k < _averagingFactors.size(); k++)
    {
        for (auto& sensor : _sensors)
        {
            std::scoped_lock<std::mutex> lg(sensor.mutex);
            Eigen::Vector3d tempSum = sensor.cumSum.at(_imuObsCount)
                                      - 2 * sensor.cumSum.at(_imuObsCount - static_cast<unsigned int>(_averagingFactors.at(k)))
                                      + sensor.cumSum.at(_imuObsCount - 2 * static_cast<unsigned int>(_averagingFactors.at(k)));

            for (size_t d = 0; d < 3; d++)
            {
                sensor.allanSum.at(d).at(k) += pow(tempSum(static_cast<Eigen::Index>(d)), 2);
            }
        }
        _observationCount.at(k)++;
    }

    // computation of Allan Variance and Deviation
    if (!_updateLast || lastMessage)
    {
        _samplingInterval = static_cast<double>((obs->insTime - _startingInsTime).count()) / _imuObsCount;
        _averagingTimes.resize(_averagingFactors.size(), 0.);
        _confidenceMultiplicationFactor.resize(_averagingFactors.size(), 0.);
        for (size_t k = 0; k < _averagingFactors.size(); k++)
        {
            _averagingTimes.at(k) = _averagingFactors.at(k) * _samplingInterval;
            _confidenceMultiplicationFactor.at(k) = sqrt(0.5 / (_imuObsCount / _averagingFactors.at(k) - 1));
        }

        for (auto& sensor : _sensors)
        {
            std::scoped_lock<std::mutex> lg(sensor.mutex);
            for (size_t d = 0; d < 3; d++)
            {
                sensor.allanVariance.at(d).resize(_averagingFactors.size(), 0.);
                sensor.allanDeviation.at(d).resize(_averagingFactors.size(), 0.);

                for (size_t j = 0; j < 2; j++)
                {
                    sensor.allanDeviationConfidenceIntervals.at(d).at(j).resize(_averagingFactors.size(), 0.);
                }

                for (size_t k = 0; k < _averagingFactors.size(); k++)
                {
                    sensor.allanVariance.at(d).at(k) = sensor.allanSum.at(d).at(k) / (2 * pow(_averagingFactors.at(k), 2) * _observationCount.at(k));

                    sensor.allanDeviation.at(d).at(k) = sqrt(sensor.allanVariance.at(d).at(k));

                    sensor.allanDeviationConfidenceIntervals.at(d).at(0).at(k) = sensor.allanDeviation.at(d).at(k) * (1 - _confidenceMultiplicationFactor.at(k));
                    sensor.allanDeviationConfidenceIntervals.at(d).at(1).at(k) = sensor.allanDeviation.at(d).at(k) * (1 + _confidenceMultiplicationFactor.at(k));
                }
            }

            // Compute Slopes
            for (size_t d = 0; d < 3; d++) { sensor.slope.at(d).resize(_averagingFactors.size(), 0.); }

            for (size_t k = 0; k < _averagingFactors.size(); k++)
            {
                size_t lo = (k == 0 ? k : k - 1);
                size_t hi = (k == _averagingFactors.size() - 1 ? k : k + 1);

                double divisor = log(_averagingFactors.at(hi) / _averagingFactors.at(lo));

                for (size_t d = 0; d < 3; d++)
                {
                    sensor.slope.at(d).at(k) = log(sensor.allanVariance.at(d).at(hi) / sensor.allanVariance.at(d).at(lo)) / divisor;
                }
            }
        }
    }

    // empty _cumSum for performance's sake
    if (lastMessage)
    {
        for (auto& sensor : _sensors) { sensor.cumSum.clear(); }
    }
}

const char* NAV::AllanDeviation::to_string(SensorType sensorType)
{
    switch (sensorType)
    {
    case SensorType::Accel:
        return "Accelerometer";
    case SensorType::Gyro:
        return "Gyroscope";
    case SensorType::SensorType_COUNT:
        return "Error: Count";
    }
    return "";
}

const char* NAV::AllanDeviation::unitString(SensorType sensorType)
{
    switch (sensorType)
    {
    case SensorType::Accel:
        return "m/s²";
    case SensorType::Gyro:
        return "rad/s";
    case SensorType::SensorType_COUNT:
        return "Error: Count";
    }
    return "";
}