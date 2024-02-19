// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TightlyCoupledKF.hpp"

#include "util/Eigen.hpp"
#include <cmath>

#include <imgui_internal.h>
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "internal/FlowManager.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/ProcessNoise.hpp"
#include "Navigation/INS/EcefFrame/ErrorEquations.hpp"
#include "Navigation/INS/LocalNavFrame/ErrorEquations.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Math/VanLoan.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "Navigation/GNSS/Satellite/internal/SatNavData.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"

#include "util/Logger.hpp"
#include "util/Container/Vector.hpp"

/// @brief Scale factor to convert the attitude error
constexpr double SCALE_FACTOR_ATTITUDE = 180. / M_PI;
/// @brief Scale factor to convert the latitude and longitude error
constexpr double SCALE_FACTOR_LAT_LON = NAV::InsConst<>::pseudometre;
/// @brief Scale factor to convert the acceleration error
constexpr double SCALE_FACTOR_ACCELERATION = 1e3 / NAV::InsConst<>::G_NORM;
/// @brief Scale factor to convert the angular rate error
constexpr double SCALE_FACTOR_ANGULAR_RATE = 1e3;

NAV::TightlyCoupledKF::TightlyCoupledKF()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 866, 938 };

    nm::CreateInputPin(this, "InertialNavSol", Pin::Type::Flow, { NAV::InertialNavSol::type() }, &TightlyCoupledKF::recvInertialNavigationSolution, nullptr, 1);
    inputPins.back().neededForTemporalQueueCheck = false;
    nm::CreateInputPin(this, "GnssObs", Pin::Type::Flow, { NAV::GnssObs::type() }, &TightlyCoupledKF::recvGnssObs,
                       [](const Node* node, const InputPin& inputPin) {
                           const auto* tckf = static_cast<const TightlyCoupledKF*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
                           return !inputPin.queue.empty() && tckf->_lastPredictRequestedTime < inputPin.queue.front()->insTime;
                       });
    inputPins.back().dropQueueIfNotFirable = false;
    updateNumberOfInputPins();
    nm::CreateOutputPin(this, "Errors", Pin::Type::Flow, { NAV::TcKfInsGnssErrors::type() });
    outputPins.back().blocksConnectedNodeFromFinishing = false; // To prevent a deadlock between ImuIntegrator and this node
    nm::CreateOutputPin(this, "Sync", Pin::Type::Flow, { NAV::NodeData::type() });
}

NAV::TightlyCoupledKF::~TightlyCoupledKF()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::TightlyCoupledKF::typeStatic()
{
    return "TightlyCoupledKF";
}

std::string NAV::TightlyCoupledKF::type() const
{
    return typeStatic();
}

std::string NAV::TightlyCoupledKF::category()
{
    return "Data Processor";
}

void NAV::TightlyCoupledKF::addKalmanMatricesPins()
{
    LOG_TRACE("{}: called", nameId());

    if (outputPins.size() == 2)
    {
        nm::CreateOutputPin(this, "x", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.x);
        nm::CreateOutputPin(this, "P", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.P);
        nm::CreateOutputPin(this, "Phi", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.Phi);
        nm::CreateOutputPin(this, "Q", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.Q);
        nm::CreateOutputPin(this, "z", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.z);
        nm::CreateOutputPin(this, "H", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.H);
        nm::CreateOutputPin(this, "R", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.R);
        nm::CreateOutputPin(this, "K", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.K);
    }
}

void NAV::TightlyCoupledKF::removeKalmanMatricesPins()
{
    LOG_TRACE("{}: called", nameId());
    while (outputPins.size() > 2)
    {
        nm::DeleteOutputPin(outputPins.back());
    }
}

void NAV::TightlyCoupledKF::guiConfig()
{
    float configWidth = 380.0F * gui::NodeEditorApplication::windowFontRatio();
    float unitWidth = 150.0F * gui::NodeEditorApplication::windowFontRatio();

    float taylorOrderWidth = 75.0F * gui::NodeEditorApplication::windowFontRatio();

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader(fmt::format("Input settings##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Pin settings##{}", size_t(id)).c_str()))
        {
            if (ImGui::BeginTable(fmt::format("Pin Settings##{}", size_t(id)).c_str(), inputPins.size() > 1 ? 3 : 2,
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
            {
                ImGui::TableSetupColumn("Pin");
                ImGui::TableSetupColumn("# Sat");
                if (inputPins.size() > 3)
                {
                    ImGui::TableSetupColumn("");
                }
                ImGui::TableHeadersRow();

                // Used to reset the member variabel _dragAndDropPinIndex in case no plot does a drag and drop action
                bool dragAndDropPinStillInProgress = false;

                auto showDragDropTargetPin = [this](size_t pinIdxTarget) {
                    ImGui::Dummy(ImVec2(-1.F, 2.F));

                    bool selectableDummy = true;
                    ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5F, 0.5F));
                    ImGui::PushStyleColor(ImGuiCol_Header, IM_COL32(16, 173, 44, 79));
                    ImGui::Selectable(fmt::format("[drop here]").c_str(), &selectableDummy, ImGuiSelectableFlags_None,
                                      ImVec2(std::max(ImGui::GetColumnWidth(0), ImGui::CalcTextSize("[drop here]").x), 20.F));
                    ImGui::PopStyleColor();
                    ImGui::PopStyleVar();

                    if (ImGui::BeginDragDropTarget())
                    {
                        if (const ImGuiPayload* payloadData = ImGui::AcceptDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str()))
                        {
                            auto pinIdxSource = *static_cast<size_t*>(payloadData->Data);

                            if (pinIdxSource < pinIdxTarget)
                            {
                                --pinIdxTarget;
                            }

                            move(inputPins, pinIdxSource, pinIdxTarget);
                            flow::ApplyChanges();
                        }
                        ImGui::EndDragDropTarget();
                    }
                    ImGui::Dummy(ImVec2(-1.F, 2.F));
                };

                for (size_t pinIndex = 0; pinIndex < inputPins.size(); pinIndex++)
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn(); // Pin

                    if (pinIndex == INPUT_PORT_INDEX_GNSS_NAV_INFO && _dragAndDropPinIndex > static_cast<int>(INPUT_PORT_INDEX_GNSS_NAV_INFO))
                    {
                        showDragDropTargetPin(INPUT_PORT_INDEX_GNSS_NAV_INFO);
                    }

                    bool selectablePinDummy = false;
                    ImGui::Selectable(fmt::format("{}##{}", inputPins.at(pinIndex).name, size_t(id)).c_str(), &selectablePinDummy);
                    if (pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO && ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
                    {
                        dragAndDropPinStillInProgress = true;
                        _dragAndDropPinIndex = static_cast<int>(pinIndex);
                        // Data is copied into heap inside the drag and drop
                        ImGui::SetDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str(), &pinIndex, sizeof(pinIndex));
                        ImGui::TextUnformatted(inputPins.at(pinIndex).name.c_str());
                        ImGui::EndDragDropSource();
                    }
                    if (_dragAndDropPinIndex > 0 && pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO
                        && pinIndex != static_cast<size_t>(_dragAndDropPinIndex - 1)
                        && pinIndex != static_cast<size_t>(_dragAndDropPinIndex))
                    {
                        showDragDropTargetPin(pinIndex + 1);
                    }
                    if (pinIndex >= INPUT_PORT_INDEX_GNSS_NAV_INFO && ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("This item can be dragged to reorder the pins");
                    }

                    ImGui::TableNextColumn(); // # Sat
                    if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(pinIndex))
                    {
                        size_t usedSatNum = 0;
                        std::string usedSats;
                        std::string allSats;

                        std::string filler = ", ";
                        for (const auto& satellite : gnssNavInfo->satellites())
                        {
                            if ((satellite.first.satSys & _filterFreq)
                                && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satellite.first) == _excludedSatellites.end())
                            {
                                usedSatNum++;
                                usedSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                            }
                            allSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                        }
                        ImGui::TextUnformatted(fmt::format("{} / {}", usedSatNum, gnssNavInfo->nSatellites()).c_str());
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::SetTooltip("Used satellites: %s\n"
                                              "All  satellites: %s",
                                              usedSats.c_str(), allSats.c_str());
                        }
                    }

                    if (inputPins.size() > 2)
                    {
                        ImGui::TableNextColumn(); // Delete
                        if (ImGui::Button(fmt::format("x##{} - {}", size_t(id), pinIndex).c_str()))
                        {
                            _nNavInfoPins--;
                            nm::DeleteInputPin(inputPins.at(pinIndex));
                            flow::ApplyChanges();
                        }
                        if (ImGui::IsItemHovered())
                        {
                            ImGui::SetTooltip("Delete the pin");
                        }
                    }
                }

                if (!dragAndDropPinStillInProgress)
                {
                    _dragAndDropPinIndex = -1;
                }

                ImGui::TableNextRow();
                ImGui::TableNextColumn(); // Pin
                if (ImGui::Button(fmt::format("Add Pin##{}", size_t(id)).c_str()))
                {
                    _nNavInfoPins++;
                    LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nNavInfoPins);
                    flow::ApplyChanges();
                    updateNumberOfInputPins();
                }

                ImGui::EndTable();
            }

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("GNSS input settings##{}", size_t(id)).c_str()))
        {
            ImGui::BeginDisabled();
            ImGui::SetNextItemWidth(configWidth);
            if (ShowFrequencySelector(fmt::format("Satellite Frequencies##{}", size_t(id)).c_str(), _filterFreq))
            {
                flow::ApplyChanges();
            }
            ImGui::EndDisabled();

            ImGui::SetNextItemWidth(configWidth);
            if (ShowCodeSelector(fmt::format("Signal Codes##{}", size_t(id)).c_str(), _filterCode, _filterFreq))
            {
                flow::ApplyChanges();
            }

            ImGui::SetNextItemWidth(configWidth);
            if (ShowSatelliteSelector(fmt::format("Excluded satellites##{}", size_t(id)).c_str(), _excludedSatellites))
            {
                flow::ApplyChanges();
            }

            double elevationMaskDeg = rad2deg(_elevationMask);
            ImGui::SetNextItemWidth(configWidth);
            if (ImGui::InputDoubleL(fmt::format("Elevation mask##{}", size_t(id)).c_str(), &elevationMaskDeg, 0.0, 90.0, 5.0, 5.0, "%.1f°", ImGuiInputTextFlags_AllowTabInput))
            {
                _elevationMask = deg2rad(elevationMaskDeg);
                LOG_DEBUG("{}: Elevation mask changed to {}°", nameId(), elevationMaskDeg);
                flow::ApplyChanges();
            }

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Compensation models##{}", size_t(id)).c_str()))
        {
            ImGui::SetNextItemWidth(configWidth - ImGui::GetStyle().IndentSpacing);
            if (ComboIonosphereModel(fmt::format("Ionosphere Model##{}", size_t(id)).c_str(), _ionosphereModel))
            {
                LOG_DEBUG("{}: Ionosphere Model changed to {}", nameId(), NAV::to_string(_ionosphereModel));
                flow::ApplyChanges();
            }
            if (ComboTroposphereModel(fmt::format("Troposphere Model##{}", size_t(id)).c_str(), _troposphereModels, configWidth - ImGui::GetStyle().IndentSpacing))
            {
                flow::ApplyChanges();
            }
            ImGui::TreePop();
        }
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader(fmt::format("Kalman filter settings##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::Combo(fmt::format("Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_frame), "ECEF\0NED\0\0"))
        {
            LOG_DEBUG("{}: Frame changed to {}", nameId(), _frame == Frame::NED ? "NED" : "ECEF");
            flow::ApplyChanges();
        }

        if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
        {
            ImGui::SetNextItemWidth(configWidth - taylorOrderWidth);
        }
        else
        {
            ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        }
        if (ImGui::Combo(fmt::format("##Phi calculation algorithm {}", size_t(id)).c_str(), reinterpret_cast<int*>(&_phiCalculationAlgorithm), "Van Loan\0Taylor\0\0"))
        {
            LOG_DEBUG("{}: Phi calculation algorithm changed to {}", nameId(), fmt::underlying(_phiCalculationAlgorithm));
            flow::ApplyChanges();
        }

        if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(taylorOrderWidth);
            if (ImGui::InputIntL(fmt::format("##Phi calculation Taylor Order {}", size_t(id)).c_str(), &_phiCalculationTaylorOrder, 1, 9))
            {
                LOG_DEBUG("{}: Phi calculation  Taylor Order changed to {}", nameId(), _phiCalculationTaylorOrder);
                flow::ApplyChanges();
            }
        }
        ImGui::SameLine();
        ImGui::Text("Phi calculation algorithm%s", _phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor ? " (up to order)" : "");

        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::Combo(fmt::format("Q calculation algorithm##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_qCalculationAlgorithm), "Van Loan\0Taylor 1st Order (Groves 2013)\0\0"))
        {
            LOG_DEBUG("{}: Q calculation algorithm changed to {}", nameId(), fmt::underlying(_qCalculationAlgorithm));
            flow::ApplyChanges();
        }
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader(fmt::format("Kalman filter parameters##{}", size_t(id)).c_str()))
    {
        // ###########################################################################################################
        //                                Q - System/Process noise covariance matrix
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Q - System/Process noise covariance matrix##{}", size_t(id)).c_str()))
        {
            // --------------------------------------------- Accelerometer -----------------------------------------------

            if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
            {
                ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
                if (ImGui::Combo(fmt::format("Random Process Accelerometer##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_randomProcessAccel), "Random Walk\0"
                                                                                                                                                    "Gauss-Markov 1st Order\0\0"))
                {
                    LOG_DEBUG("{}: randomProcessAccel changed to {}", nameId(), fmt::underlying(_randomProcessAccel));
                    flow::ApplyChanges();
                }
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the noise on the\naccelerometer specific-force measurements##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _stdev_ra.data(), reinterpret_cast<int*>(&_stdevAccelNoiseUnits), "mg/√(Hz)\0m/s^2/√(Hz)\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_ra changed to {}", nameId(), _stdev_ra.transpose());
                LOG_DEBUG("{}: stdevAccelNoiseUnits changed to {}", nameId(), fmt::underlying(_stdevAccelNoiseUnits));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation σ of the accel {}##{}",
                                                               _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1
                                                                   ? "dynamic bias, in σ²/τ"
                                                                   : (_randomProcessAccel == RandomProcess::RandomWalk ? "bias noise" : "bias noise, in √(2σ²β)"),
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _stdev_bad.data(), reinterpret_cast<int*>(&_stdevAccelBiasUnits), "µg\0m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_bad changed to {}", nameId(), _stdev_bad.transpose());
                LOG_DEBUG("{}: stdevAccelBiasUnits changed to {}", nameId(), fmt::underlying(_stdevAccelBiasUnits));
                flow::ApplyChanges();
            }

            if (_randomProcessAccel == RandomProcess::GaussMarkov1 || _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
            {
                ImGui::SetNextItemWidth(configWidth - unitWidth);
                if (ImGui::InputDouble3L(fmt::format("##Correlation length τ of the accel dynamic bias {}", size_t(id)).c_str(), _tau_bad.data(), 0., std::numeric_limits<double>::max(), "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: tau_bad changed to {}", nameId(), _tau_bad);
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                int unitCorrelationLength = 0;
                ImGui::SetNextItemWidth(unitWidth);
                ImGui::Combo(fmt::format("##Correlation length of the accel dynamic bias unit {}", size_t(id)).c_str(), &unitCorrelationLength, "s\0\0");
                ImGui::SameLine();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
                if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
                {
                    ImGui::TextUnformatted("Correlation length τ of the accel bias noise");
                }
                else if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
                {
                    ImGui::TextUnformatted("Correlation length τ of the accel dynamic bias");
                }
            }

            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 20.F);

            // ----------------------------------------------- Gyroscope -------------------------------------------------

            if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
            {
                ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
                if (ImGui::Combo(fmt::format("Random Process Gyroscope##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_randomProcessGyro), "Random Walk\0"
                                                                                                                                               "Gauss-Markov 1st Order\0\0"))
                {
                    LOG_DEBUG("{}: randomProcessGyro changed to {}", nameId(), fmt::underlying(_randomProcessGyro));
                    flow::ApplyChanges();
                }
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the noise on\nthe gyro angular-rate measurements##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _stdev_rg.data(), reinterpret_cast<int*>(&_stdevGyroNoiseUnits), "deg/hr/√(Hz)\0rad/s/√(Hz)\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_rg changed to {}", nameId(), _stdev_rg.transpose());
                LOG_DEBUG("{}: stdevGyroNoiseUnits changed to {}", nameId(), fmt::underlying(_stdevGyroNoiseUnits));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation σ of the gyro {}##{}",
                                                               _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1
                                                                   ? "dynamic bias, in σ²/τ"
                                                                   : (_randomProcessGyro == RandomProcess::RandomWalk ? "bias noise" : "bias noise, in √(2σ²β)"),
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _stdev_bgd.data(), reinterpret_cast<int*>(&_stdevGyroBiasUnits), "°/h\0rad/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_bgd changed to {}", nameId(), _stdev_bgd.transpose());
                LOG_DEBUG("{}: stdevGyroBiasUnits changed to {}", nameId(), fmt::underlying(_stdevGyroBiasUnits));
                flow::ApplyChanges();
            }

            if (_randomProcessGyro == RandomProcess::GaussMarkov1 || _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
            {
                ImGui::SetNextItemWidth(configWidth - unitWidth);
                if (ImGui::InputDouble3L(fmt::format("##Correlation length τ of the gyro dynamic bias {}", size_t(id)).c_str(), _tau_bgd.data(), 0., std::numeric_limits<double>::max(), "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: tau_bgd changed to {}", nameId(), _tau_bgd);
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                int unitCorrelationLength = 0;
                ImGui::SetNextItemWidth(unitWidth);
                ImGui::Combo(fmt::format("##Correlation length of the gyro dynamic bias unit {}", size_t(id)).c_str(), &unitCorrelationLength, "s\0\0");
                ImGui::SameLine();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
                if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
                {
                    ImGui::TextUnformatted("Correlation length τ of the gyro bias noise");
                }
                else if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
                {
                    ImGui::TextUnformatted("Correlation length τ of the gyro dynamic bias");
                }
            }

            // --------------------------------------------- Clock -----------------------------------------------

            auto* _stdev_cpPointer = &_stdev_cp;
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the receiver\nclock phase drift (RW)##{}", size_t(id)).c_str(),
                                                  configWidth, unitWidth, _stdev_cpPointer, reinterpret_cast<int*>(&_stdevClockPhaseUnits), "m/√(Hz)\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_cf changed to {}", nameId(), _stdev_cp);
                LOG_DEBUG("{}: stdevClockPhaseUnits changed to {}", nameId(), fmt::underlying(_stdevClockPhaseUnits));
                flow::ApplyChanges();
            }

            auto* _stdev_cfPointer = &_stdev_cf;
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the receiver\nclock frequency drift (IRW)##{}", size_t(id)).c_str(),
                                                  configWidth, unitWidth, _stdev_cfPointer, reinterpret_cast<int*>(&_stdevClockFreqUnits), "m/s/√(Hz)\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_cf changed to {}", nameId(), _stdev_cf);
                LOG_DEBUG("{}: stdevClockFreqUnits changed to {}", nameId(), fmt::underlying(_stdevClockFreqUnits));
                flow::ApplyChanges();
            }

            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                        Measurement Uncertainties 𝐑
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
        {
            auto* _gnssMeasurementUncertaintyPseudorangePointer = &_gnssMeasurementUncertaintyPseudorange;
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Pseudorange covariance ({})##{}",
                                                              _gnssMeasurementUncertaintyPseudorangeUnit == GnssMeasurementUncertaintyPseudorangeUnit::meter2
                                                                  ? "Variance σ²"
                                                                  : "Standard deviation σ",
                                                              size_t(id))
                                                      .c_str(),
                                                  configWidth, unitWidth, _gnssMeasurementUncertaintyPseudorangePointer, reinterpret_cast<int*>(&_gnssMeasurementUncertaintyPseudorangeUnit), "m^2\0"
                                                                                                                                                                                              "m\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gnssMeasurementUncertaintyPseudorange changed to {}", nameId(), _gnssMeasurementUncertaintyPseudorange);
                LOG_DEBUG("{}: gnssMeasurementUncertaintyPseudorangeUnit changed to {}", nameId(), fmt::underlying(_gnssMeasurementUncertaintyPseudorangeUnit));
                flow::ApplyChanges();
            }

            auto* _gnssMeasurementUncertaintyPseudorangeRatePointer = &_gnssMeasurementUncertaintyPseudorangeRate;
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Pseudorange-rate covariance ({})##{}",
                                                              _gnssMeasurementUncertaintyPseudorangeRateUnit == GnssMeasurementUncertaintyPseudorangeRateUnit::m2_s2
                                                                  ? "Variance σ²"
                                                                  : "Standard deviation σ",
                                                              size_t(id))
                                                      .c_str(),
                                                  configWidth, unitWidth, _gnssMeasurementUncertaintyPseudorangeRatePointer, reinterpret_cast<int*>(&_gnssMeasurementUncertaintyPseudorangeRateUnit), "m^2/s^2\0"
                                                                                                                                                                                                      "m/s\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gnssMeasurementUncertaintyPseudorangeRate changed to {}", nameId(), _gnssMeasurementUncertaintyPseudorangeRate);
                LOG_DEBUG("{}: gnssMeasurementUncertaintyPseudorangeRateUnit changed to {}", nameId(), fmt::underlying(_gnssMeasurementUncertaintyPseudorangeRateUnit));
                flow::ApplyChanges();
            }

            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                        𝐏 Error covariance matrix
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("P - Error covariance matrix (init)##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Position covariance ({})##{}",
                                                               _initCovariancePositionUnit == InitCovariancePositionUnit::rad2_rad2_m2
                                                                       || _initCovariancePositionUnit == InitCovariancePositionUnit::meter2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovariancePosition.data(), reinterpret_cast<int*>(&_initCovariancePositionUnit), "rad^2, rad^2, m^2\0"
                                                                                                                                                                 "rad, rad, m\0"
                                                                                                                                                                 "m^2, m^2, m^2\0"
                                                                                                                                                                 "m, m, m\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovariancePosition changed to {}", nameId(), _initCovariancePosition);
                LOG_DEBUG("{}: initCovariancePositionUnit changed to {}", nameId(), fmt::underlying(_initCovariancePositionUnit));
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity covariance ({})##{}",
                                                               _initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceVelocity.data(), reinterpret_cast<int*>(&_initCovarianceVelocityUnit), "m^2/s^2\0"
                                                                                                                                                                 "m/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceVelocity changed to {}", nameId(), _initCovarianceVelocity);
                LOG_DEBUG("{}: initCovarianceVelocityUnit changed to {}", nameId(), fmt::underlying(_initCovarianceVelocityUnit));
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Flight Angles covariance ({})##{}",
                                                               _initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad2
                                                                       || _initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceAttitudeAngles.data(), reinterpret_cast<int*>(&_initCovarianceAttitudeAnglesUnit), "rad^2\0"
                                                                                                                                                                             "deg^2\0"
                                                                                                                                                                             "rad\0"
                                                                                                                                                                             "deg\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceAttitudeAngles changed to {}", nameId(), _initCovarianceAttitudeAngles);
                LOG_DEBUG("{}: initCovarianceAttitudeAnglesUnit changed to {}", nameId(), fmt::underlying(_initCovarianceAttitudeAnglesUnit));
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker(_frame == Frame::ECEF
                                         ? "Angles rotating the ECEF frame into the body frame."
                                         : "Angles rotating the local navigation frame into the body frame (Roll, Pitch, Yaw)");

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer Bias covariance ({})##{}",
                                                               _initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceBiasAccel.data(), reinterpret_cast<int*>(&_initCovarianceBiasAccelUnit), "m^2/s^4\0"
                                                                                                                                                                   "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceBiasAccel changed to {}", nameId(), _initCovarianceBiasAccel);
                LOG_DEBUG("{}: initCovarianceBiasAccelUnit changed to {}", nameId(), fmt::underlying(_initCovarianceBiasAccelUnit));
                flow::ApplyChanges();
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Gyroscope Bias covariance ({})##{}",
                                                               _initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad2_s2
                                                                       || _initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg2_s2
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceBiasGyro.data(), reinterpret_cast<int*>(&_initCovarianceBiasGyroUnit), "rad^2/s^2\0"
                                                                                                                                                                 "deg^2/s^2\0"
                                                                                                                                                                 "rad/s\0"
                                                                                                                                                                 "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceBiasGyro changed to {}", nameId(), _initCovarianceBiasGyro);
                LOG_DEBUG("{}: initCovarianceBiasGyroUnit changed to {}", nameId(), fmt::underlying(_initCovarianceBiasGyroUnit));
                flow::ApplyChanges();
            }

            auto* initCovariancePhasePointer = &_initCovariancePhase;
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Receiver clock phase drift covariance ({})##{}",
                                                              _initCovariancePhaseUnit == InitCovarianceClockPhaseUnit::m2
                                                                      || _initCovariancePhaseUnit == InitCovarianceClockPhaseUnit::s2
                                                                  ? "Variance σ²"
                                                                  : "Standard deviation σ",
                                                              size_t(id))
                                                      .c_str(),
                                                  configWidth, unitWidth, initCovariancePhasePointer, reinterpret_cast<int*>(&_initCovariancePhaseUnit), "m^2\0"
                                                                                                                                                         "s^2\0"
                                                                                                                                                         "m\0"
                                                                                                                                                         "s\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovariancePhase changed to {}", nameId(), _initCovariancePhase);
                LOG_DEBUG("{}: InitCovarianceClockPhaseUnit changed to {}", nameId(), fmt::underlying(_initCovariancePhaseUnit));
                flow::ApplyChanges();
            }

            auto* initCovarianceFreqPointer = &_initCovarianceFreq;
            if (gui::widgets::InputDoubleWithUnit(fmt::format("Receiver clock frequency drift covariance ({})##{}",
                                                              _initCovarianceFreqUnit == InitCovarianceClockFreqUnit::m2_s2
                                                                  ? "Variance σ²"
                                                                  : "Standard deviation σ",
                                                              size_t(id))
                                                      .c_str(),
                                                  configWidth, unitWidth, initCovarianceFreqPointer, reinterpret_cast<int*>(&_initCovarianceFreqUnit), "m^2/s^2\0"
                                                                                                                                                       "m/s\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceFreq changed to {}", nameId(), _initCovarianceFreq);
                LOG_DEBUG("{}: initCovarianceFreqUnit changed to {}", nameId(), fmt::underlying(_initCovarianceFreqUnit));
                flow::ApplyChanges();
            }

            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                                IMU biases
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("IMU biases (init)##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer biases##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _initBiasAccel.data(), reinterpret_cast<int*>(&_initBiasAccelUnit), "m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initBiasAccel changed to {}", nameId(), _initBiasAccel.transpose());
                LOG_DEBUG("{}: initBiasAccelUnit changed to {}", nameId(), fmt::underlying(_initBiasAccelUnit));
                flow::ApplyChanges();
            }
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Gyro biases##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _initBiasGyro.data(), reinterpret_cast<int*>(&_initBiasGyroUnit), "rad/s\0deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initBiasGyro changed to {}", nameId(), _initBiasGyro.transpose());
                LOG_DEBUG("{}: initBiasGyroUnit changed to {}", nameId(), fmt::underlying(_initBiasGyroUnit));
                flow::ApplyChanges();
            }

            ImGui::TreePop();
        }

        ImGui::Separator();

        if (ImGui::Checkbox(fmt::format("Rank check for Kalman filter matrices##{}", size_t(id)).c_str(), &_checkKalmanMatricesRanks))
        {
            LOG_DEBUG("{}: checkKalmanMatricesRanks {}", nameId(), _checkKalmanMatricesRanks);
            flow::ApplyChanges();
        }

        if (ImGui::Checkbox(fmt::format("Show Kalman Filter matrices as output pins##{}", size_t(id)).c_str(), &_showKalmanFilterOutputPins))
        {
            LOG_DEBUG("{}: showKalmanFilterOutputPins {}", nameId(), _showKalmanFilterOutputPins);
            if (_showKalmanFilterOutputPins)
            {
                addKalmanMatricesPins();
            }
            else
            {
                removeKalmanMatricesPins();
            }
            flow::ApplyChanges();
        }
    }
}

[[nodiscard]] json NAV::TightlyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["showKalmanFilterOutputPins"] = _showKalmanFilterOutputPins;
    j["nNavInfoPins"] = _nNavInfoPins;
    j["frequencies"] = Frequency_(_filterFreq);
    j["codes"] = _filterCode;
    j["excludedSatellites"] = _excludedSatellites;
    j["elevationMask"] = rad2deg(_elevationMask);
    j["ionosphereModel"] = _ionosphereModel;
    j["troposphereModels"] = _troposphereModels;

    j["checkKalmanMatricesRanks"] = _checkKalmanMatricesRanks;
    j["frame"] = _frame;
    j["phiCalculationAlgorithm"] = _phiCalculationAlgorithm;
    j["phiCalculationTaylorOrder"] = _phiCalculationTaylorOrder;
    j["qCalculationAlgorithm"] = _qCalculationAlgorithm;

    j["randomProcessAccel"] = _randomProcessAccel;
    j["randomProcessGyro"] = _randomProcessGyro;
    j["stdev_ra"] = _stdev_ra;
    j["stdevAccelNoiseUnits"] = _stdevAccelNoiseUnits;
    j["stdev_rg"] = _stdev_rg;
    j["stdevGyroNoiseUnits"] = _stdevGyroNoiseUnits;
    j["stdev_bad"] = _stdev_bad;
    j["tau_bad"] = _tau_bad;
    j["stdevAccelBiasUnits"] = _stdevAccelBiasUnits;
    j["stdev_bgd"] = _stdev_bgd;
    j["tau_bgd"] = _tau_bgd;
    j["stdevGyroBiasUnits"] = _stdevGyroBiasUnits;
    j["stdevClockFreqUnits"] = _stdevClockFreqUnits;
    j["stdev_cp"] = _stdev_cp;
    j["stdev_cf"] = _stdev_cf;
    j["initBiasAccel"] = _initBiasAccel;
    j["initBiasAccelUnit"] = _initBiasAccelUnit;
    j["initBiasGyro"] = _initBiasGyro;
    j["initBiasGyroUnit"] = _initBiasGyroUnit;

    j["initCovariancePositionUnit"] = _initCovariancePositionUnit;
    j["initCovariancePosition"] = _initCovariancePosition;
    j["initCovarianceVelocityUnit"] = _initCovarianceVelocityUnit;
    j["initCovarianceVelocity"] = _initCovarianceVelocity;
    j["initCovarianceAttitudeAnglesUnit"] = _initCovarianceAttitudeAnglesUnit;
    j["initCovarianceAttitudeAngles"] = _initCovarianceAttitudeAngles;
    j["initCovarianceBiasAccelUnit"] = _initCovarianceBiasAccelUnit;
    j["initCovarianceBiasAccel"] = _initCovarianceBiasAccel;
    j["initCovarianceBiasGyroUnit"] = _initCovarianceBiasGyroUnit;
    j["initCovarianceBiasGyro"] = _initCovarianceBiasGyro;
    j["initCovariancePhaseUnit"] = _initCovariancePhaseUnit;
    j["initCovariancePhase"] = _initCovariancePhase;
    j["initCovarianceFreqUnit"] = _initCovarianceFreqUnit;
    j["initCovarianceFreq"] = _initCovarianceFreq;
    j["gnssMeasurementUncertaintyPseudorangeUnit"] = _gnssMeasurementUncertaintyPseudorangeUnit;
    j["gnssMeasurementUncertaintyPseudorange"] = _gnssMeasurementUncertaintyPseudorange;
    j["gnssMeasurementUncertaintyPseudorangeRateUnit"] = _gnssMeasurementUncertaintyPseudorangeRateUnit;
    j["gnssMeasurementUncertaintyPseudorangeRate"] = _gnssMeasurementUncertaintyPseudorangeRate;

    return j;
}

void NAV::TightlyCoupledKF::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("showKalmanFilterOutputPins"))
    {
        j.at("showKalmanFilterOutputPins").get_to(_showKalmanFilterOutputPins);
        if (_showKalmanFilterOutputPins)
        {
            addKalmanMatricesPins();
        }
    }
    if (j.contains("nNavInfoPins"))
    {
        j.at("nNavInfoPins").get_to(_nNavInfoPins);
        updateNumberOfInputPins();
    }
    if (j.contains("frequencies"))
    {
        uint64_t value = 0;
        j.at("frequencies").get_to(value);
        _filterFreq = Frequency_(value);
    }
    if (j.contains("codes"))
    {
        j.at("codes").get_to(_filterCode);
    }
    if (j.contains("excludedSatellites"))
    {
        j.at("excludedSatellites").get_to(_excludedSatellites);
    }
    if (j.contains("elevationMask"))
    {
        j.at("elevationMask").get_to(_elevationMask);
        _elevationMask = deg2rad(_elevationMask);
    }
    if (j.contains("ionosphereModel"))
    {
        j.at("ionosphereModel").get_to(_ionosphereModel);
    }
    if (j.contains("troposphereModels"))
    {
        j.at("troposphereModels").get_to(_troposphereModels);
    }

    if (j.contains("checkKalmanMatricesRanks"))
    {
        j.at("checkKalmanMatricesRanks").get_to(_checkKalmanMatricesRanks);
    }
    if (j.contains("frame"))
    {
        j.at("frame").get_to(_frame);
    }
    if (j.contains("phiCalculationAlgorithm"))
    {
        j.at("phiCalculationAlgorithm").get_to(_phiCalculationAlgorithm);
    }
    if (j.contains("phiCalculationTaylorOrder"))
    {
        j.at("phiCalculationTaylorOrder").get_to(_phiCalculationTaylorOrder);
    }
    if (j.contains("qCalculationAlgorithm"))
    {
        j.at("qCalculationAlgorithm").get_to(_qCalculationAlgorithm);
    }
    // ------------------------------- 𝐐 System/Process noise covariance matrix ---------------------------------
    if (j.contains("randomProcessAccel"))
    {
        j.at("randomProcessAccel").get_to(_randomProcessAccel);
    }
    if (j.contains("randomProcessGyro"))
    {
        j.at("randomProcessGyro").get_to(_randomProcessGyro);
    }
    if (j.contains("stdev_ra"))
    {
        _stdev_ra = j.at("stdev_ra");
    }
    if (j.contains("stdevAccelNoiseUnits"))
    {
        j.at("stdevAccelNoiseUnits").get_to(_stdevAccelNoiseUnits);
    }
    if (j.contains("stdev_rg"))
    {
        _stdev_rg = j.at("stdev_rg");
    }
    if (j.contains("stdevGyroNoiseUnits"))
    {
        j.at("stdevGyroNoiseUnits").get_to(_stdevGyroNoiseUnits);
    }
    if (j.contains("stdev_bad"))
    {
        _stdev_bad = j.at("stdev_bad");
    }
    if (j.contains("tau_bad"))
    {
        _tau_bad = j.at("tau_bad");
    }
    if (j.contains("stdevAccelBiasUnits"))
    {
        j.at("stdevAccelBiasUnits").get_to(_stdevAccelBiasUnits);
    }
    if (j.contains("stdev_bgd"))
    {
        _stdev_bgd = j.at("stdev_bgd");
    }
    if (j.contains("tau_bgd"))
    {
        _tau_bgd = j.at("tau_bgd");
    }
    if (j.contains("stdevGyroBiasUnits"))
    {
        j.at("stdevGyroBiasUnits").get_to(_stdevGyroBiasUnits);
    }
    if (j.contains("stdevClockFreqUnits"))
    {
        j.at("stdevClockFreqUnits").get_to(_stdevClockFreqUnits);
    }
    if (j.contains("stdev_cp"))
    {
        _stdev_cp = j.at("stdev_cp");
    }
    if (j.contains("stdev_cf"))
    {
        _stdev_cf = j.at("stdev_cf");
    }
    if (j.contains("initBiasAccel"))
    {
        _initBiasAccel = j.at("initBiasAccel");
    }
    if (j.contains("initBiasAccelUnit"))
    {
        _initBiasAccelUnit = j.at("initBiasAccelUnit");
    }
    if (j.contains("initBiasGyro"))
    {
        _initBiasGyro = j.at("initBiasGyro");
    }
    if (j.contains("initBiasGyroUnit"))
    {
        _initBiasGyroUnit = j.at("initBiasGyroUnit");
    }

    // -------------------------------- 𝐑 Measurement noise covariance matrix -----------------------------------
    if (j.contains("gnssMeasurementUncertaintyPseudorangeUnit"))
    {
        _gnssMeasurementUncertaintyPseudorangeUnit = j.at("gnssMeasurementUncertaintyPseudorangeUnit");
    }
    if (j.contains("gnssMeasurementUncertaintyPseudorange"))
    {
        _gnssMeasurementUncertaintyPseudorange = j.at("gnssMeasurementUncertaintyPseudorange");
    }
    if (j.contains("gnssMeasurementUncertaintyPseudorangeRateUnit"))
    {
        _gnssMeasurementUncertaintyPseudorangeRateUnit = j.at("gnssMeasurementUncertaintyPseudorangeRateUnit");
    }
    if (j.contains("gnssMeasurementUncertaintyPseudorangeRate"))
    {
        _gnssMeasurementUncertaintyPseudorangeRate = j.at("gnssMeasurementUncertaintyPseudorangeRate");
    }

    // -------------------------------------- 𝐏 Error covariance matrix -----------------------------------------
    if (j.contains("initCovariancePositionUnit"))
    {
        j.at("initCovariancePositionUnit").get_to(_initCovariancePositionUnit);
    }
    if (j.contains("initCovariancePosition"))
    {
        _initCovariancePosition = j.at("initCovariancePosition");
    }
    if (j.contains("initCovarianceVelocityUnit"))
    {
        j.at("initCovarianceVelocityUnit").get_to(_initCovarianceVelocityUnit);
    }
    if (j.contains("initCovarianceVelocity"))
    {
        _initCovarianceVelocity = j.at("initCovarianceVelocity");
    }
    if (j.contains("initCovarianceAttitudeAnglesUnit"))
    {
        j.at("initCovarianceAttitudeAnglesUnit").get_to(_initCovarianceAttitudeAnglesUnit);
    }
    if (j.contains("initCovarianceAttitudeAngles"))
    {
        _initCovarianceAttitudeAngles = j.at("initCovarianceAttitudeAngles");
    }
    if (j.contains("initCovarianceBiasAccelUnit"))
    {
        j.at("initCovarianceBiasAccelUnit").get_to(_initCovarianceBiasAccelUnit);
    }
    if (j.contains("initCovarianceBiasAccel"))
    {
        _initCovarianceBiasAccel = j.at("initCovarianceBiasAccel");
    }
    if (j.contains("initCovarianceBiasGyroUnit"))
    {
        j.at("initCovarianceBiasGyroUnit").get_to(_initCovarianceBiasGyroUnit);
    }
    if (j.contains("initCovarianceBiasGyro"))
    {
        _initCovarianceBiasGyro = j.at("initCovarianceBiasGyro");
    }
    if (j.contains("initCovariancePhaseUnit"))
    {
        j.at("initCovariancePhaseUnit").get_to(_initCovariancePhaseUnit);
    }
    if (j.contains("initCovariancePhase"))
    {
        _initCovariancePhase = j.at("initCovariancePhase");
    }
    if (j.contains("initCovarianceFreqUnit"))
    {
        j.at("initCovarianceFreqUnit").get_to(_initCovarianceFreqUnit);
    }
    if (j.contains("initCovarianceFreq"))
    {
        _initCovarianceFreq = j.at("initCovarianceFreq");
    }
}

bool NAV::TightlyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (std::all_of(inputPins.begin() + INPUT_PORT_INDEX_GNSS_NAV_INFO, inputPins.end(), [](const InputPin& inputPin) { return !inputPin.isPinLinked(); }))
    {
        LOG_ERROR("{}: You need to connect a GNSS NavigationInfo provider", nameId());
        return false;
    }

    _recvClk = {};

    _kalmanFilter.setZero();

    _latestInertialNavSol = nullptr;
    _lastPredictTime.reset();
    _lastPredictRequestedTime.reset();

    // Bias inits
    _accumulatedAccelBiases = _initBiasAccel;
    _accumulatedGyroBiases = _initBiasGyro;

    // Initial Covariance of the attitude angles in [rad²]
    Eigen::Vector3d variance_angles = Eigen::Vector3d::Zero();
    if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad2)
    {
        variance_angles = _initCovarianceAttitudeAngles;
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg2)
    {
        variance_angles = deg2rad(_initCovarianceAttitudeAngles);
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad)
    {
        variance_angles = _initCovarianceAttitudeAngles.array().pow(2);
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg)
    {
        variance_angles = deg2rad(_initCovarianceAttitudeAngles).array().pow(2);
    }

    // Initial Covariance of the velocity in [m²/s²]
    Eigen::Vector3d variance_vel = Eigen::Vector3d::Zero();
    if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2)
    {
        variance_vel = _initCovarianceVelocity;
    }
    else if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m_s)
    {
        variance_vel = _initCovarianceVelocity.array().pow(2);
    }

    // Initial Covariance of the position in [m²]
    Eigen::Vector3d e_variance = Eigen::Vector3d::Zero();
    // Initial Covariance of the position in [rad² rad² m²]
    Eigen::Vector3d lla_variance = Eigen::Vector3d::Zero();
    if (_initCovariancePositionUnit == InitCovariancePositionUnit::rad2_rad2_m2)
    {
        e_variance = trafo::lla2ecef_WGS84(_initCovariancePosition.cwiseSqrt()).array().pow(2);
        lla_variance = _initCovariancePosition;
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::rad_rad_m)
    {
        e_variance = trafo::lla2ecef_WGS84(_initCovariancePosition).array().pow(2);
        lla_variance = _initCovariancePosition.array().pow(2);
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter)
    {
        e_variance = _initCovariancePosition.array().pow(2);
        lla_variance = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition, Eigen::Vector3d{ 0, 0, 0 }))).array().pow(2);
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter2)
    {
        e_variance = _initCovariancePosition;
        lla_variance = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition.cwiseSqrt(), Eigen::Vector3d{ 0, 0, 0 }))).array().pow(2);
    }

    // Initial Covariance of the accelerometer biases in [m^2/s^4]
    Eigen::Vector3d variance_accelBias = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m2_s4)
    {
        variance_accelBias = _initCovarianceBiasAccel;
    }
    else if (_initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m_s2)
    {
        variance_accelBias = _initCovarianceBiasAccel.array().pow(2);
    }

    // Initial Covariance of the gyroscope biases in [rad^2/s^2]
    Eigen::Vector3d variance_gyroBias = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad2_s2)
    {
        variance_gyroBias = _initCovarianceBiasGyro;
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg2_s2)
    {
        variance_gyroBias = deg2rad(_initCovarianceBiasGyro.array().sqrt()).array().pow(2);
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad_s)
    {
        variance_gyroBias = _initCovarianceBiasGyro.array().pow(2);
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg_s)
    {
        variance_gyroBias = deg2rad(_initCovarianceBiasGyro).array().pow(2);
    }

    // Initial Covariance of the receiver clock phase drift
    double variance_clkPhase{};
    if (_initCovariancePhaseUnit == InitCovarianceClockPhaseUnit::m2)
    {
        variance_clkPhase = _initCovariancePhase;
    }
    if (_initCovariancePhaseUnit == InitCovarianceClockPhaseUnit::s2)
    {
        variance_clkPhase = std::pow(InsConst<>::C, 2) * _initCovariancePhase;
    }
    if (_initCovariancePhaseUnit == InitCovarianceClockPhaseUnit::m)
    {
        variance_clkPhase = std::pow(_initCovariancePhase, 2);
    }
    if (_initCovariancePhaseUnit == InitCovarianceClockPhaseUnit::s)
    {
        variance_clkPhase = std::pow(InsConst<>::C * _initCovariancePhase, 2);
    }

    // Initial Covariance of the receiver clock frequency drift
    double variance_clkFreq{};
    if (_initCovarianceFreqUnit == InitCovarianceClockFreqUnit::m2_s2)
    {
        variance_clkFreq = _initCovarianceFreq;
    }
    if (_initCovarianceFreqUnit == InitCovarianceClockFreqUnit::m_s)
    {
        variance_clkFreq = std::pow(_initCovarianceFreq, 2);
    }

    // 𝐏 Error covariance matrix
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(variance_angles,                                  // Flight Angles covariance
                                                      variance_vel,                                     // Velocity covariance
                                                      _frame == Frame::NED ? lla_variance : e_variance, // Position (Lat, Lon, Alt) / ECEF covariance
                                                      variance_accelBias,                               // Accelerometer Bias covariance
                                                      variance_gyroBias,                                // Gyroscope Bias covariance
                                                      variance_clkPhase,                                // Receiver clock phase drift covariance
                                                      variance_clkFreq);                                // Receiver clock frequency drift covariance

    LOG_DEBUG("{}: initialized", nameId());
    LOG_DATA("{}: P_0 =\n{}", nameId(), _kalmanFilter.P);

    return true;
}

void NAV::TightlyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::TightlyCoupledKF::updateNumberOfInputPins()
{
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO < _nNavInfoPins)
    {
        nm::CreateInputPin(this, NAV::GnssNavInfo::type().c_str(), Pin::Type::Object, { NAV::GnssNavInfo::type() });
    }
    while (inputPins.size() - INPUT_PORT_INDEX_GNSS_NAV_INFO > _nNavInfoPins)
    {
        nm::DeleteInputPin(inputPins.back());
    }
}

void NAV::TightlyCoupledKF::recvInertialNavigationSolution(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */) // NOLINT(readability-convert-member-functions-to-static)
{
    auto inertialNavSol = std::static_pointer_cast<const InertialNavSol>(queue.extract_front());
    LOG_DATA("{}: recvInertialNavigationSolution at time [{} - {}]", nameId(), inertialNavSol->insTime.toYMDHMS(), inertialNavSol->insTime.toGPSweekTow());

    double tau_i = !_lastPredictTime.empty()
                       ? static_cast<double>((inertialNavSol->insTime - _lastPredictTime).count())
                       : 0.0;

    if (tau_i > 0)
    {
        _lastPredictTime = _latestInertialNavSol->insTime + std::chrono::duration<double>(tau_i);
        tightlyCoupledPrediction(_latestInertialNavSol, tau_i);
    }
    else
    {
        _lastPredictTime = inertialNavSol->insTime;
    }
    _latestInertialNavSol = inertialNavSol;

    if (!inputPins[INPUT_PORT_INDEX_GNSS_OBS].queue.empty() && inputPins[INPUT_PORT_INDEX_GNSS_OBS].queue.front()->insTime == _lastPredictTime)
    {
        tightlyCoupledUpdate(std::static_pointer_cast<const GnssObs>(inputPins[INPUT_PORT_INDEX_GNSS_OBS].queue.extract_front()));
        if (inputPins[INPUT_PORT_INDEX_GNSS_OBS].queue.empty() && inputPins[INPUT_PORT_INDEX_GNSS_OBS].link.getConnectedPin()->noMoreDataAvailable)
        {
            outputPins[OUTPUT_PORT_INDEX_SYNC].noMoreDataAvailable = true;
            for (auto& link : outputPins[OUTPUT_PORT_INDEX_SYNC].links)
            {
                link.connectedNode->wakeWorker();
            }
        }
    }
}

void NAV::TightlyCoupledKF::recvGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto gnssObs = queue.front();
    LOG_DATA("{}: recvGnssObs at time [{}]", nameId(), gnssObs->insTime.toYMDHMS());

    auto nodeData = std::make_shared<NodeData>();
    nodeData->insTime = gnssObs->insTime;
    _lastPredictRequestedTime = gnssObs->insTime;

    invokeCallbacks(OUTPUT_PORT_INDEX_SYNC, nodeData); // Prediction consists out of ImuIntegration and prediction (gets triggered from it)
}

// ###########################################################################################################
//                                               Kalman Filter
// ###########################################################################################################

void NAV::TightlyCoupledKF::tightlyCoupledPrediction(const std::shared_ptr<const InertialNavSol>& inertialNavSol, double tau_i)
{
    auto dt = fmt::format("{:0.5f}", tau_i);
    dt.erase(std::find_if(dt.rbegin(), dt.rend(), [](char ch) { return ch != '0'; }).base(), dt.end());

    [[maybe_unused]] InsTime predictTime = inertialNavSol->insTime + std::chrono::duration<double>(tau_i);
    LOG_DATA("{}: Predicting (dt = {}s) from [{} - {}] to [{} - {}]", nameId(), dt,
             inertialNavSol->insTime.toYMDHMS(), inertialNavSol->insTime.toGPSweekTow(), predictTime.toYMDHMS(), predictTime.toGPSweekTow());

    // ------------------------------------------- GUI Parameters ----------------------------------------------

    // 𝜎²_ra Variance of the noise on the accelerometer specific-force state [m^2 / s^5]
    Eigen::Vector3d sigma2_ra = Eigen::Vector3d::Zero();
    switch (_stdevAccelNoiseUnits)
    {
    case StdevAccelNoiseUnits::mg_sqrtHz: // [mg / √(Hz)]
        sigma2_ra = (_stdev_ra * 1e-3 * InsConst<>::G_NORM).array().square();
        break;
    case StdevAccelNoiseUnits::m_s2_sqrtHz: // [m / (s^2 · √(Hz))] = [m / (s · √(s))]
        sigma2_ra = _stdev_ra.array().square();
        break;
    }
    LOG_DATA("{}:     sigma2_ra = {} [m^2 / s^5]", nameId(), sigma2_ra.transpose());

    // 𝜎²_rg Variance of the noise on the gyro angular-rate state [rad^2 / s^3]
    Eigen::Vector3d sigma2_rg = Eigen::Vector3d::Zero();
    switch (_stdevGyroNoiseUnits)
    {
    case StdevGyroNoiseUnits::deg_hr_sqrtHz: // [deg / hr / √(Hz)] (see Woodman (2007) Chp. 3.2.2 - eq. 7 with seconds instead of hours)
        sigma2_rg = (deg2rad(_stdev_rg) / 3600.).array().square();
        break;
    case StdevGyroNoiseUnits::rad_s_sqrtHz: // [rad / (s · √(Hz))] = [rad / √(s)]
        sigma2_rg = _stdev_rg.array().square();
        break;
    }
    LOG_DATA("{}:     sigma2_rg = {} [rad^2 / s^3]", nameId(), sigma2_rg.transpose());

    // 𝜎²_bad Variance of the accelerometer dynamic bias [m^2 / s^4]
    Eigen::Vector3d sigma2_bad = Eigen::Vector3d::Zero();
    switch (_stdevAccelBiasUnits)
    {
    case StdevAccelBiasUnits::microg: // [µg]
        sigma2_bad = (_stdev_bad * 1e-6 * InsConst<>::G_NORM).array().square();
        break;
    case StdevAccelBiasUnits::m_s2: // [m / s^2]
        sigma2_bad = _stdev_bad.array().square();
        break;
    }
    LOG_DATA("{}:     sigma2_bad = {} [m^2 / s^4]", nameId(), sigma2_bad.transpose());

    // 𝜎²_bgd Variance of the gyro dynamic bias [rad^2 / s^2]
    Eigen::Vector3d sigma2_bgd = Eigen::Vector3d::Zero();
    switch (_stdevGyroBiasUnits)
    {
    case StdevGyroBiasUnits::deg_h: // [° / h]
        sigma2_bgd = (deg2rad(_stdev_bgd / 3600.0)).array().square();
        break;
    case StdevGyroBiasUnits::rad_s: // [rad / s]
        sigma2_bgd = _stdev_bgd.array().square();
        break;
    }
    LOG_DATA("{}:     sigma2_bgd = {} [rad^2 / s^2]", nameId(), sigma2_bgd.transpose());

    // 𝜎²_cPhi variance of the receiver clock phase-drift in [m^2]
    double sigma2_cPhi{};
    switch (_stdevClockPhaseUnits)
    {
    case StdevClockPhaseUnits::m_sqrtHz: // [m / s^2 / √(Hz)]
        sigma2_cPhi = std::pow(_stdev_cp, 2);
        break;
    }

    // 𝜎²_cf variance of the receiver clock frequency drift state [m^2 / s^4 / Hz]
    double sigma2_cf{};
    switch (_stdevClockFreqUnits)
    {
    case StdevClockFreqUnits::m_s_sqrtHz: // [m / s^2 / √(Hz)]
        sigma2_cf = std::pow(_stdev_cf, 2);
        break;
    }
    LOG_DATA("{}:     sigma2_cPhi = {} [m^2]", nameId(), sigma2_cPhi);
    LOG_DATA("{}:     sigma2_cf = {} [m^2/s^2]", nameId(), sigma2_cf);

    // ---------------------------------------------- Prediction -------------------------------------------------

    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& lla_position = inertialNavSol->lla_position();
    LOG_DATA("{}:     lla_position = {} [rad, rad, m]", nameId(), lla_position.transpose());
    // Prime vertical radius of curvature (East/West) [m]
    double R_E = calcEarthRadius_E(lla_position(0));
    LOG_DATA("{}:     R_E = {} [m]", nameId(), R_E);
    // Geocentric Radius in [m]
    double r_eS_e = calcGeocentricRadius(lla_position(0), R_E);
    LOG_DATA("{}:     r_eS_e = {} [m]", nameId(), r_eS_e);

    // a_p Acceleration in [m/s^2], in body coordinates
    auto b_acceleration = _latestInertialNavSol->imuObs == nullptr
                              ? Eigen::Vector3d::Zero()
                              : Eigen::Vector3d(inertialNavSol->imuObs->imuPos.b_quatAccel_p() * inertialNavSol->imuObs->accelUncompXYZ.value()
                                                - _accumulatedAccelBiases);
    LOG_DATA("{}:     b_acceleration = {} [m/s^2]", nameId(), b_acceleration.transpose());

    // System Matrix
    Eigen::Matrix<double, 17, 17> F;

    if (_frame == Frame::NED)
    {
        // n_velocity (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
        const Eigen::Vector3d& n_velocity = inertialNavSol->n_velocity();
        LOG_DATA("{}:     n_velocity = {} [m / s]", nameId(), n_velocity.transpose());
        // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond& n_Quat_b = inertialNavSol->n_Quat_b();
        LOG_DATA("{}:     n_Quat_b --> Roll, Pitch, Yaw = {} [deg]", nameId(), deg2rad(trafo::quat2eulerZYX(n_Quat_b).transpose()));

        // Meridian radius of curvature in [m]
        double R_N = calcEarthRadius_N(lla_position(0));
        LOG_DATA("{}:     R_N = {} [m]", nameId(), R_N);

        // Conversion matrix between cartesian and curvilinear perturbations to the position
        Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(lla_position, R_N, R_E);
        LOG_DATA("{}:     T_rn_p =\n{}", nameId(), T_rn_p);

        // Gravitation at surface level in [m/s^2]
        double g_0 = n_calcGravitation_EGM96(lla_position).norm();

        // omega_in^n = omega_ie^n + omega_en^n
        Eigen::Vector3d n_omega_in = inertialNavSol->n_Quat_e() * InsConst<>::e_omega_ie
                                     + n_calcTransportRate(lla_position, n_velocity, R_N, R_E);
        LOG_DATA("{}:     n_omega_in = {} [rad/s]", nameId(), n_omega_in.transpose());

        // System Matrix
        F = n_systemMatrix_F(n_Quat_b, b_acceleration, n_omega_in, n_velocity, lla_position, R_N, R_E, g_0, r_eS_e, _tau_bad, _tau_bgd);
        LOG_DATA("{}:     F =\n{}", nameId(), F);

        if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            // 2. Calculate the system noise covariance matrix Q_{k-1}
            if (_showKalmanFilterOutputPins)
            {
                auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_Q);
                _kalmanFilter.Q = n_systemNoiseCovarianceMatrix_Q(sigma2_ra, sigma2_rg,
                                                                  sigma2_bad, sigma2_bgd,
                                                                  _tau_bad, _tau_bgd,
                                                                  sigma2_cPhi, sigma2_cf,
                                                                  F.block<3, 3>(3, 0), T_rn_p,
                                                                  n_Quat_b.toRotationMatrix(), tau_i);
            }
            else
            {
                _kalmanFilter.Q = n_systemNoiseCovarianceMatrix_Q(sigma2_ra, sigma2_rg,
                                                                  sigma2_bad, sigma2_bgd,
                                                                  _tau_bad, _tau_bgd,
                                                                  sigma2_cPhi, sigma2_cf,
                                                                  F.block<3, 3>(3, 0), T_rn_p,
                                                                  n_Quat_b.toRotationMatrix(), tau_i);
            }
        }
    }
    else // if (_frame == Frame::ECEF)
    {
        // e_position (tₖ₋₁) Position in [m/s], in ECEF coordinates, at the time tₖ₋₁
        const Eigen::Vector3d& e_position = inertialNavSol->e_position();
        LOG_DATA("{}:     e_position = {} [m]", nameId(), e_position.transpose());
        // q (tₖ₋₁) Quaternion, from body to Earth coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond& e_Quat_b = inertialNavSol->e_Quat_b();
        LOG_DATA("{}:     e_Quat_b = {}", nameId(), e_Quat_b);

        // Gravitation in [m/s^2] in ECEF coordinates
        Eigen::Vector3d e_gravitation = trafo::e_Quat_n(lla_position(0), lla_position(1)) * n_calcGravitation_EGM96(lla_position);

        // System Matrix
        F = e_systemMatrix_F(e_Quat_b, b_acceleration, e_position, e_gravitation, r_eS_e, InsConst<>::e_omega_ie, _tau_bad, _tau_bgd);
        LOG_DATA("{}:     F =\n{}", nameId(), F);

        if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            // 2. Calculate the system noise covariance matrix Q_{k-1}
            if (_showKalmanFilterOutputPins)
            {
                auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_Q);

                _kalmanFilter.Q = e_systemNoiseCovarianceMatrix_Q(sigma2_ra, sigma2_rg,
                                                                  sigma2_bad, sigma2_bgd,
                                                                  _tau_bad, _tau_bgd,
                                                                  sigma2_cPhi, sigma2_cf,
                                                                  F.block<3, 3>(3, 0),
                                                                  e_Quat_b.toRotationMatrix(), tau_i);
            }
            else
            {
                _kalmanFilter.Q = e_systemNoiseCovarianceMatrix_Q(sigma2_ra, sigma2_rg,
                                                                  sigma2_bad, sigma2_bgd,
                                                                  _tau_bad, _tau_bgd,
                                                                  sigma2_cPhi, sigma2_cf,
                                                                  F.block<3, 3>(3, 0),
                                                                  e_Quat_b.toRotationMatrix(), tau_i);
            }
        }
    }

    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        // Noise Input Matrix
        Eigen::Matrix<double, 17, 14> G = noiseInputMatrix_G(_frame == Frame::NED ? inertialNavSol->n_Quat_b() : inertialNavSol->e_Quat_b());
        LOG_DATA("{}:     G =\n{}", nameId(), G);

        Eigen::Matrix<double, 14, 14> W = noiseScaleMatrix_W(sigma2_ra, sigma2_rg,
                                                             sigma2_bad, sigma2_bgd,
                                                             _tau_bad, _tau_bgd,
                                                             sigma2_cPhi, sigma2_cf);
        LOG_DATA("{}:     W =\n{}", nameId(), W);

        LOG_DATA("{}:     G*W*G^T =\n{}", nameId(), G * W * G.transpose());

        auto [Phi, Q] = calcPhiAndQWithVanLoanMethod(F, G, W, tau_i);

        // 1. Calculate the transition matrix 𝚽_{k-1}
        if (_showKalmanFilterOutputPins)
        {
            auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_Phi);
            _kalmanFilter.Phi = Phi;
        }
        else
        {
            _kalmanFilter.Phi = Phi;
        }

        // 2. Calculate the system noise covariance matrix Q_{k-1}
        if (_showKalmanFilterOutputPins)
        {
            auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_Q);
            _kalmanFilter.Q = Q;
        }
        else
        {
            _kalmanFilter.Q = Q;
        }
    }

    // If Q was calculated over Van Loan, then the Phi matrix was automatically calculated with the exponential matrix
    if (_phiCalculationAlgorithm != PhiCalculationAlgorithm::Exponential || _qCalculationAlgorithm != QCalculationAlgorithm::VanLoan)
    {
        auto calcPhi = [&]() {
            if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Exponential)
            {
                // 1. Calculate the transition matrix 𝚽_{k-1}
                _kalmanFilter.Phi = transitionMatrix_Phi_exp(F, tau_i);
            }
            else if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
            {
                // 1. Calculate the transition matrix 𝚽_{k-1}
                _kalmanFilter.Phi = transitionMatrix_Phi_Taylor(F, tau_i, static_cast<size_t>(_phiCalculationTaylorOrder));
            }
            else
            {
                LOG_CRITICAL("{}: Calculation algorithm '{}' for the system matrix Phi is not supported.", nameId(), fmt::underlying(_phiCalculationAlgorithm));
            }
        };
        if (_showKalmanFilterOutputPins)
        {
            auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_Phi);
            calcPhi();
        }
        else
        {
            calcPhi();
        }
    }

    LOG_DATA("{}:     KF.Phi =\n{}", nameId(), _kalmanFilter.Phi);
    LOG_DATA("{}:     KF.Q =\n{}", nameId(), _kalmanFilter.Q);
    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_Phi, predictTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_Q, predictTime);
    }
    LOG_DATA("{}:     Q - Q^T =\n{}", nameId(), _kalmanFilter.Q - _kalmanFilter.Q.transpose());
    LOG_DATA("{}:     KF.P (before prediction) =\n{}", nameId(), _kalmanFilter.P);

    // 3. Propagate the state vector estimate from x(+) and x(-)
    // 4. Propagate the error covariance matrix from P(+) and P(-)
    if (_showKalmanFilterOutputPins)
    {
        auto guard1 = requestOutputValueLock(OUTPUT_PORT_INDEX_x);
        auto guard2 = requestOutputValueLock(OUTPUT_PORT_INDEX_P);
        _kalmanFilter.predict();
    }
    else
    {
        _kalmanFilter.predict();
    }

    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_x, predictTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_P, predictTime);
    }
    LOG_DATA("{}:     KF.x = {}", nameId(), _kalmanFilter.x.transpose());
    LOG_DATA("{}:     KF.P (after prediction) =\n{}", nameId(), _kalmanFilter.P);

    // Averaging of P to avoid numerical problems with symmetry (did not work)
    // _kalmanFilter.P = ((_kalmanFilter.P + _kalmanFilter.P.transpose()) / 2.0);

    // LOG_DEBUG("{}: F\n{}\n", nameId(), F);
    // LOG_DEBUG("{}: Phi\n{}\n", nameId(), _kalmanFilter.Phi);

    // LOG_DEBUG("{}: Q\n{}\n", nameId(), _kalmanFilter.Q);
    // LOG_DEBUG("{}: Q - Q^T\n{}\n", nameId(), _kalmanFilter.Q - _kalmanFilter.Q.transpose());

    // LOG_DEBUG("{}: x\n{}\n", nameId(), _kalmanFilter.x);

    // LOG_DEBUG("{}: P\n{}\n", nameId(), _kalmanFilter.P);
    // LOG_DEBUG("{}: P - P^T\n{}\n", nameId(), _kalmanFilter.P - _kalmanFilter.P.transpose());

    if (_checkKalmanMatricesRanks)
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_kalmanFilter.P);
        auto rank = lu.rank();
        if (rank != _kalmanFilter.P.rows())
        {
            LOG_WARN("{}: P.rank = {}", nameId(), rank);
        }
    }
}

void NAV::TightlyCoupledKF::tightlyCoupledUpdate(const std::shared_ptr<const GnssObs>& gnssObs)
{
    LOG_DATA("{}: Updating to time {} - {} (lastInertial at {} - {})", nameId(), gnssObs->insTime.toYMDHMS(), gnssObs->insTime.toGPSweekTow(),
             _latestInertialNavSol->insTime.toYMDHMS(), _latestInertialNavSol->insTime.toGPSweekTow());

    // -------------------------------------------- GUI Parameters -----------------------------------------------

    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& lla_position = _latestInertialNavSol->lla_position();
    LOG_DATA("{}:     lla_position = {} [rad, rad, m]", nameId(), lla_position.transpose());

    // GNSS measurement uncertainty for the pseudorange (Variance σ²) in [m^2]
    double gnssSigmaSquaredPseudorange{};
    switch (_gnssMeasurementUncertaintyPseudorangeUnit)
    {
    case GnssMeasurementUncertaintyPseudorangeUnit::meter:
        gnssSigmaSquaredPseudorange = std::pow(_gnssMeasurementUncertaintyPseudorange, 2);
        break;
    case GnssMeasurementUncertaintyPseudorangeUnit::meter2:
        gnssSigmaSquaredPseudorange = _gnssMeasurementUncertaintyPseudorange;
        break;
    }
    LOG_DATA("{}:     gnssSigmaSquaredPseudorange = {} [m^2]", nameId(), gnssSigmaSquaredPseudorange);

    // GNSS measurement uncertainty for the pseudorange-rate (Variance σ²) in [m^2/s^2]
    double gnssSigmaSquaredPseudorangeRate{};
    switch (_gnssMeasurementUncertaintyPseudorangeRateUnit)
    {
    case GnssMeasurementUncertaintyPseudorangeRateUnit::m_s:
        gnssSigmaSquaredPseudorangeRate = std::pow(_gnssMeasurementUncertaintyPseudorangeRate, 2);
        break;
    case GnssMeasurementUncertaintyPseudorangeRateUnit::m2_s2:
        gnssSigmaSquaredPseudorangeRate = _gnssMeasurementUncertaintyPseudorangeRate;
        break;
    }
    LOG_DATA("{}:     gnssSigmaSquaredPseudorangeRate = {} [m^2/s^2]", nameId(), gnssSigmaSquaredPseudorangeRate);

    // TODO: Check whether it is necessary to distinguish the following three types (see Groves eq. 9.168)
    double sigma_rhoZ = std::sqrt(gnssSigmaSquaredPseudorange);
    // double sigma_rhoC{};
    // double sigma_rhoA{};
    double sigma_rZ = std::sqrt(gnssSigmaSquaredPseudorangeRate);
    // double sigma_rC{};
    // double sigma_rA{};

    // ----------------------------------------- Read observation data -------------------------------------------

    // Collection of all connected navigation data providers
    std::vector<const GnssNavInfo*> gnssNavInfos;
    std::vector<std::unique_lock<std::mutex>> guards;
    for (size_t i = 0; i < _nNavInfoPins; i++)
    {
        if (auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
        {
            guards.emplace_back(*mutex);
            if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
            {
                gnssNavInfos.push_back(gnssNavInfo);
            }
            else
            {
                guards.pop_back();
            }
        }
    }
    if (gnssNavInfos.empty()) { return; }

    // Collection of all connected Ionospheric Corrections
    IonosphericCorrections ionosphericCorrections;
    for (const auto* gnssNavInfo : gnssNavInfos)
    {
        for (const auto& correction : gnssNavInfo->ionosphericCorrections.data())
        {
            if (!ionosphericCorrections.contains(correction.satSys, correction.alphaBeta))
            {
                ionosphericCorrections.insert(correction.satSys, correction.alphaBeta, correction.data);
            }
        }
    }

    // Data calculated for each observation
    struct CalcData
    {
        // Constructor
        explicit CalcData(size_t obsIdx, std::shared_ptr<NAV::SatNavData> satNavData)
            : obsIdx(obsIdx), satNavData(std::move(satNavData)) {}

        size_t obsIdx = 0;                                     // Index in the provided GNSS Observation data
        std::shared_ptr<NAV::SatNavData> satNavData = nullptr; // Satellite Navigation data

        double satClkBias{};                    // Satellite clock bias [s]
        double satClkDrift{};                   // Satellite clock drift [s/s]
        Eigen::Vector3d e_satPos;               // Satellite position in ECEF frame coordinates [m]
        Eigen::Vector3d e_satVel;               // Satellite velocity in ECEF frame coordinates [m/s]
        double pseudorangeRate{ std::nan("") }; // Pseudorange rate [m/s]

        // Data recalculated each iteration

        bool skipped = false;                                            // Whether to skip the measurement
        Eigen::Vector3d e_lineOfSightUnitVector;                         // Line-of-sight unit vector in ECEF frame coordinates
        Eigen::Vector3d n_lineOfSightUnitVector;                         // Line-of-sight unit vector in NED frame coordinates
        double satElevation = calcSatElevation(n_lineOfSightUnitVector); // Elevation [rad]
        double satAzimuth = calcSatAzimuth(n_lineOfSightUnitVector);     // Azimuth [rad]
    };

    // Data calculated for each satellite (only satellites filtered by GUI filter & NAV data available)
    std::vector<CalcData> calcData;
    std::vector<SatelliteSystem> availSatelliteSystems; // List of satellite systems
    for (size_t obsIdx = 0; obsIdx < gnssObs->data.size(); obsIdx++)
    {
        const auto& obsData = gnssObs->data[obsIdx];
        auto satId = obsData.satSigId.toSatId();

        if ((obsData.satSigId.freq() & _filterFreq)                                                                   // frequency is selected in GUI
            && (obsData.satSigId.code & _filterCode)                                                                  // code is selected in GUI
            && obsData.pseudorange                                                                                    // has a valid pseudorange
            && std::find(_excludedSatellites.begin(), _excludedSatellites.end(), satId) == _excludedSatellites.end()) // is not excluded
        {
            for (const auto& gnssNavInfo : gnssNavInfos)
            {
                if (auto satNavData = gnssNavInfo->searchNavigationData(satId, gnssObs->insTime)) // can calculate satellite position
                {
                    if (!satNavData->isHealthy())
                    {
                        LOG_DATA("{}: Satellite {} is skipped because the signal is not healthy.", nameId(), satId);
                        continue;
                    }
                    LOG_DATA("{}: Using observation from {} {}", nameId(), obsData.satSigId, obsData.satSigId.code);
                    calcData.emplace_back(obsIdx, satNavData);
                    if (std::find(availSatelliteSystems.begin(), availSatelliteSystems.end(), satId.satSys) == availSatelliteSystems.end())
                    {
                        availSatelliteSystems.push_back(satId.satSys);
                    }
                    break;
                }
            }
        }
        else if (!obsData.pseudorange && obsData.doppler)
        {
            LOG_WARN("{}: Epoch contains Doppler ({}), but no Pseudorange and is thus skipped.", nameId(), obsData.doppler.value());
        }
    }

    size_t nMeas = calcData.size();
    LOG_DATA("{}: nMeas {}", nameId(), nMeas);
    size_t nParam = 4 + availSatelliteSystems.size() - 1; // 3x pos, 1x clk, (N-1)x clkDiff

    // Find all observations providing a doppler measurement (for velocity calculation)
    size_t nDopplerMeas = 0;
    for (size_t i = 0; i < nMeas; i++)
    {
        const auto& obsData = gnssObs->data[calcData[i].obsIdx];
        if (obsData.doppler)
        {
            // Frequency number (GLONASS only)
            int8_t freqNum = -128;

            nDopplerMeas++;
            // TODO: Find out what this is used for and find a way to use it, after the GLONASS orbit calculation is working
            if (obsData.satSigId.freq() & (R01 | R02))
            {
                if (auto satNavData = std::dynamic_pointer_cast<GLONASSEphemeris>(calcData[i].satNavData))
                {
                    freqNum = satNavData->frequencyNumber;
                }
            }

            calcData[i].pseudorangeRate = doppler2rangeRate(obsData.doppler.value(), obsData.satSigId.freq(), freqNum);
        }
    }

    // ----------------------------------------------- Satellite -------------------------------------------------

    for (size_t i = 0; i < nMeas; i++) // Calculate satellite clock, position and velocity
    {
        const auto& obsData = gnssObs->data[calcData[i].obsIdx];

        LOG_DATA("{}: satellite {}", nameId(), obsData.satSigId);
        LOG_DATA("{}:     pseudorange  {}", nameId(), obsData.pseudorange.value().value);

        auto satClk = calcData[i].satNavData->calcClockCorrections(gnssObs->insTime, obsData.pseudorange.value().value, obsData.satSigId.freq());
        calcData[i].satClkBias = satClk.bias;
        calcData[i].satClkDrift = satClk.drift;
        LOG_DATA("{}:     satClkBias {}, satClkDrift {}", nameId(), calcData[i].satClkBias, calcData[i].satClkDrift);

        auto satPosVel = calcData[i].satNavData->calcSatellitePosVel(satClk.transmitTime);
        calcData[i].e_satPos = satPosVel.e_pos;
        calcData[i].e_satVel = satPosVel.e_vel;
        LOG_DATA("{}:     e_satPos {}", nameId(), calcData[i].e_satPos.transpose());
        LOG_DATA("{}:     e_satVel {}", nameId(), calcData[i].e_satVel.transpose());
    }

    // Pseudorange estimates [m]
    Eigen::VectorXd psrEst = Eigen::VectorXd::Zero(static_cast<int>(nMeas));
    // Pseudorange measurements [m]
    Eigen::VectorXd psrMeas = Eigen::VectorXd::Zero(static_cast<int>(nMeas));

    // Corrected pseudorange-rate estimates [m/s]
    Eigen::VectorXd psrRateEst = Eigen::VectorXd::Zero(static_cast<int>(nDopplerMeas));
    // Corrected pseudorange-rate measurements [m/s]
    Eigen::VectorXd psrRateMeas = Eigen::VectorXd::Zero(static_cast<int>(nDopplerMeas));

    // Keeps track of skipped meausrements (because of elevation mask, ...)
    size_t cntSkippedMeas = 0;

    const Eigen::Vector3d& e_position = _latestInertialNavSol->e_position();
    const Eigen::Vector3d& e_velocity = _latestInertialNavSol->e_velocity();

    LOG_DATA("{}: _recvClk.bias {}", nameId(), _recvClk.bias.value);
    LOG_DATA("{}: _recvClk.drift {}", nameId(), _recvClk.drift.value);

    std::vector<SatelliteSystem> satelliteSystems = availSatelliteSystems; // List of satellite systems

    SatelliteSystem_ usedSatelliteSystems = SatSys_None;
    for (size_t i = 0; i < nMeas; i++)
    {
        const auto& obsData = gnssObs->data[calcData[i].obsIdx];
        LOG_DATA("{}: satellite {}", nameId(), obsData.satSigId);
        auto satId = obsData.satSigId.toSatId();

        // Line-of-sight unit vector in ECEF frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
        calcData[i].e_lineOfSightUnitVector = e_calcLineOfSightUnitVector(e_position, calcData[i].e_satPos);
        LOG_DATA("{}:     e_lineOfSightUnitVector {}", nameId(), calcData[i].e_lineOfSightUnitVector.transpose());
        // Line-of-sight unit vector in NED frame coordinates - Groves ch. 8.5.3, eq. 8.41, p. 341
        calcData[i].n_lineOfSightUnitVector = trafo::n_Quat_e(lla_position(0), lla_position(1)) * calcData[i].e_lineOfSightUnitVector;
        LOG_DATA("{}:     n_lineOfSightUnitVector {}", nameId(), calcData[i].n_lineOfSightUnitVector.transpose());
        // Elevation [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
        calcData[i].satElevation = calcSatElevation(calcData[i].n_lineOfSightUnitVector);
        LOG_DATA("{}:     satElevation {}°", nameId(), rad2deg(calcData[i].satElevation));
        // Azimuth [rad] - Groves ch. 8.5.4, eq. 8.57, p. 344
        calcData[i].satAzimuth = calcSatAzimuth(calcData[i].n_lineOfSightUnitVector);
        LOG_DATA("{}:     satAzimuth {}°", nameId(), rad2deg(calcData[i].satAzimuth));

        if (!e_position.isZero() && calcData[i].satElevation < _elevationMask) // Do not check elevation mask when not having a valid position
        {
            cntSkippedMeas++;
            calcData[i].skipped = true;
            LOG_DATA("{}: [{}] Measurement is skipped because of elevation {:.1f}° and mask of {}° ({} valid measurements remaining)",
                     nameId(), obsData.satSigId, rad2deg(calcData[i].satElevation), rad2deg(_elevationMask), nMeas - cntSkippedMeas);

            if (!(usedSatelliteSystems & satId.satSys)
                && calcData.begin() + static_cast<int64_t>(i + 1) != calcData.end()                                         // This is the last satellite and the system did not appear before
                && std::none_of(calcData.begin() + static_cast<int64_t>(i + 1), calcData.end(), [&](const CalcData& data) { // The satellite system has no satellites available anymore
                       return gnssObs->data[data.obsIdx].satSigId.toSatId().satSys == satId.satSys;
                   }))
            {
                LOG_DEBUG("{}: The satellite system {} won't be used this iteration because no satellite complies with the elevation mask.",
                          nameId(), satId.satSys);
                nParam--;
                satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), satId.satSys));
            }

            if (nMeas - cntSkippedMeas < nParam)
            {
                LOG_WARN("{}: [{}] Cannot calculate position because only {} valid measurements ({} needed). Try changing filter settings or reposition your antenna.",
                         nameId(), gnssObs->insTime, nMeas - cntSkippedMeas, nParam);
            }
            continue;
        }

        usedSatelliteSystems |= satId.satSys;
    }

    size_t ix = 0;
    size_t iv = 0;
    if (!satelliteSystems.empty()) // skip this, if there is e.g. no pseudorange available. Better than crashing
    {
        _recvClk.referenceTimeSatelliteSystem = satelliteSystems.front();
        for (const auto& availSatSys : satelliteSystems)
        {
            if (SatelliteSystem_(availSatSys) < SatelliteSystem_(_recvClk.referenceTimeSatelliteSystem))
            {
                _recvClk.referenceTimeSatelliteSystem = availSatSys;
            }
        }
        satelliteSystems.erase(std::find(satelliteSystems.begin(), satelliteSystems.end(), _recvClk.referenceTimeSatelliteSystem));
        LOG_DATA("{}: _recvClk.referenceTimeSatelliteSystem {} ({} other time systems)", nameId(), _recvClk.referenceTimeSatelliteSystem, satelliteSystems.size());
    }

    double tau_epoch = !_lastEpochTime.empty()
                           ? static_cast<double>((gnssObs->insTime - _lastEpochTime).count())
                           : 0.0;

    LOG_DATA("{}: tau_epoch = {}", nameId(), tau_epoch);

    _lastEpochTime = gnssObs->insTime;

    for (auto& calc : calcData)
    {
        if (calc.skipped) { continue; }

        const auto& obsData = gnssObs->data[calc.obsIdx];
        LOG_DATA("{}: satellite {}", nameId(), obsData.satSigId);
        // auto satId = obsData.satSigId.toSatId();

        // #############################################################################################################################
        //                                                    Position calculation
        // #############################################################################################################################

        // Pseudorange measurement [m] - Groves ch. 8.5.3, eq. 8.48, p. 342
        psrMeas(static_cast<int>(ix)) = obsData.pseudorange.value().value /* + (multipath and/or NLOS errors) + (tracking errors) */;
        LOG_DATA("{}:     psrMeas({}) {}", nameId(), ix, psrMeas(static_cast<int>(ix)));
        // Estimated modulation ionosphere propagation error [m]
        double dpsr_I = calcIonosphericDelay(static_cast<double>(gnssObs->insTime.toGPSweekTow().tow), obsData.satSigId.freq(), -128, lla_position,
                                             calc.satElevation, calc.satAzimuth, _ionosphereModel, &ionosphericCorrections);
        LOG_DATA("{}:     dpsr_I {} [m] (Estimated modulation ionosphere propagation error)", nameId(), dpsr_I);

        auto tropo = calcTroposphericDelayAndMapping(gnssObs->insTime, lla_position, calc.satElevation, calc.satAzimuth, _troposphereModels);
        LOG_DATA("{}:     ZHD {}", nameId(), tropo.ZHD);
        LOG_DATA("{}:     ZWD {}", nameId(), tropo.ZWD);
        LOG_DATA("{}:     zhdMappingFactor {}", nameId(), tropo.zhdMappingFactor);
        LOG_DATA("{}:     zwdMappingFactor {}", nameId(), tropo.zwdMappingFactor);

        // Estimated modulation troposphere propagation error [m]
        double dpsr_T = tropo.ZHD * tropo.zhdMappingFactor + tropo.ZWD * tropo.zwdMappingFactor;
        LOG_DATA("{}:     dpsr_T {} [m] (Estimated modulation troposphere propagation error)", nameId(), dpsr_T);

        // Sagnac correction - Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
        double dpsr_ie = 1.0 / InsConst<>::C * (e_position - calc.e_satPos).dot(InsConst<>::e_omega_ie.cross(e_position));
        LOG_DATA("{}:     dpsr_ie {}", nameId(), dpsr_ie);
        // Geometric distance [m]
        double geometricDist = (calc.e_satPos - e_position).norm();
        LOG_DATA("{}:     geometricDist {}", nameId(), geometricDist);

        _recvClk.bias.value += _kalmanFilter.x(15, 0) / InsConst<>::C;

        LOG_DATA("{}: _recvClk.bias {} s", nameId(), _recvClk.bias.value);

        // Pseudorange estimate [m]
        psrEst(static_cast<int>(ix)) = geometricDist
                                       + _recvClk.bias.value * InsConst<>::C
                                       + _recvClk.drift.value * InsConst<>::C * tau_epoch
                                       - calc.satClkBias * InsConst<>::C
                                       + dpsr_I
                                       + dpsr_T
                                       + dpsr_ie;
        LOG_DATA("{}:     psrEst({}) {}", nameId(), ix, psrEst(static_cast<int>(ix)));

        LOG_DATA("{}:     dpsr({}) {}", nameId(), ix, psrMeas(static_cast<int>(ix)) - psrEst(static_cast<int>(ix)));

        // #############################################################################################################################
        //                                                    Velocity calculation
        // #############################################################################################################################

        if (nDopplerMeas - cntSkippedMeas >= nParam && !std::isnan(calc.pseudorangeRate))
        {
            // Pseudorange-rate measurement [m/s] - Groves ch. 8.5.3, eq. 8.48, p. 342
            psrRateMeas(static_cast<int>(iv)) = calc.pseudorangeRate /* + (multipath and/or NLOS errors) + (tracking errors) */;
            LOG_DATA("{}:     psrRateMeas({}) {}", nameId(), iv, psrRateMeas(static_cast<int>(iv)));

            // Range-rate Sagnac correction - Groves ch. 8.5.3, eq. 8.46, p. 342
            double dpsr_dot_ie = InsConst<>::omega_ie / InsConst<>::C
                                 * (calc.e_satVel.y() * e_position.x() + calc.e_satPos.y() * e_velocity.x()
                                    - calc.e_satVel.x() * e_position.y() - calc.e_satPos.x() * e_velocity.y());
            LOG_DATA("{}:     dpsr_dot_ie {}", nameId(), dpsr_dot_ie);

            LOG_DATA("{}: _kalmanFilter.x(16, 0) = {} m/s", nameId(), _kalmanFilter.x(16, 0));
            LOG_DATA("{}: _recvClk.drift {} s/s", nameId(), _recvClk.drift.value);

            _recvClk.drift.value += _kalmanFilter.x(16, 0) / InsConst<>::C;

            LOG_DATA("{}: _recvClk.drift {} s/s", nameId(), _recvClk.drift.value);

            // Pseudorange-rate estimate [m/s] - Groves ch. 9.4.1, eq. 9.142, p. 412 (Sagnac correction different sign)
            psrRateEst(static_cast<int>(iv)) = calc.e_lineOfSightUnitVector.transpose() * (calc.e_satVel - e_velocity)
                                               + _recvClk.drift.value * InsConst<>::C
                                               - calc.satClkDrift * InsConst<>::C
                                               - dpsr_dot_ie;
            LOG_DATA("{}:     psrRateEst({}) {}", nameId(), iv, psrRateEst(static_cast<int>(iv)));

            iv++;
        }

        ix++;
    }
    LOG_DATA("{}: _recvClk.bias {} s", nameId(), _recvClk.bias.value);
    LOG_DATA("{}: _recvClk.drift {} s/s", nameId(), _recvClk.drift.value);

    // ---------------------------------------------- Update -----------------------------------------------------

    if (_frame == Frame::NED)
    {
        // Prime vertical radius of curvature (East/West) [m]
        double R_E = calcEarthRadius_E(lla_position(0));
        LOG_DATA("{}:     R_E = {} [m]", nameId(), R_E);
        // Meridian radius of curvature in [m]
        double R_N = calcEarthRadius_N(lla_position(0));
        LOG_DATA("{}:     R_N = {} [m]", nameId(), R_N);

        std::vector<Eigen::Vector3d> n_lineOfSightUnitVectors;
        n_lineOfSightUnitVectors.resize(ix);
        std::vector<double> satElevation;
        satElevation.resize(ix);
        // std::vector<double> CN0; // TODO: get this from GnssObs
        // std::vector<double> rangeAccel; // TODO: get this from GnssObs

        std::vector<double> pseudoRangeObservations;
        pseudoRangeObservations.resize(ix);
        std::vector<double> pseudoRangeRateObservations;
        pseudoRangeRateObservations.resize(ix);

        size_t ix = 0;

        for (auto& calc : calcData)
        {
            LOG_DATA("calc.n_lineOfSightUnitVector.transpose() = {}, calc.skipped = {}", calc.n_lineOfSightUnitVector.transpose(), calc.skipped);
            if (calc.skipped) { continue; }
            n_lineOfSightUnitVectors[ix] = calc.n_lineOfSightUnitVector;
            LOG_DATA("n_lineOfSightUnitVectors[{}] = {}", ix, n_lineOfSightUnitVectors[ix].transpose());
            satElevation[ix] = calc.satElevation;
            LOG_DATA("satElevation[{}] = {}", ix, satElevation[ix]);
            // CN0[i] = calc // TODO: get CN0 from data
            // rangeAccel[i] = calc // TODO: get rangeAccel from data
            pseudoRangeObservations[ix] = psrMeas(static_cast<Eigen::Index>(ix));
            pseudoRangeRateObservations[ix] = calc.pseudorangeRate;

            ix++;
        }

        // 5. Calculate the measurement matrix H_k
        if (_showKalmanFilterOutputPins)
        {
            auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_H);
            _kalmanFilter.H = n_measurementMatrix_H(R_N, R_E, lla_position, n_lineOfSightUnitVectors, pseudoRangeRateObservations);
            LOG_DATA("{}: kalmanFilter.H =\n{}", nameId(), _kalmanFilter.H);
        }
        else
        {
            _kalmanFilter.H = n_measurementMatrix_H(R_N, R_E, lla_position, n_lineOfSightUnitVectors, pseudoRangeRateObservations);
            LOG_DATA("{}: kalmanFilter.H =\n{}", nameId(), _kalmanFilter.H);
        }

        // 6. Calculate the measurement noise covariance matrix R_k
        if (_showKalmanFilterOutputPins)
        {
            auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_R);
            _kalmanFilter.R = measurementNoiseCovariance_R(sigma_rhoZ, sigma_rZ, satElevation);
            LOG_DATA("{}: kalmanFilter.R =\n{}", nameId(), _kalmanFilter.R);
        }
        else
        {
            _kalmanFilter.R = measurementNoiseCovariance_R(sigma_rhoZ, sigma_rZ, satElevation);
            LOG_DATA("{}: kalmanFilter.R =\n{}", nameId(), _kalmanFilter.R);
        }

        std::vector<double> pseudoRangeEstimates;
        pseudoRangeEstimates.resize(ix);
        std::vector<double> pseudoRangeRateEstimates;
        pseudoRangeRateEstimates.resize(ix);
        for (size_t obsIdx = 0; obsIdx < n_lineOfSightUnitVectors.size(); obsIdx++)
        {
            pseudoRangeEstimates[obsIdx] = psrEst(static_cast<Eigen::Index>(obsIdx));
            if (psrRateEst.size() != 0)
            {
                pseudoRangeRateEstimates[obsIdx] = psrRateEst(static_cast<Eigen::Index>(obsIdx));
            }
        }

        // 8. Formulate the measurement z_k
        if (_showKalmanFilterOutputPins)
        {
            auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_z);
            _kalmanFilter.z = measurementInnovation_dz(pseudoRangeObservations, pseudoRangeEstimates, pseudoRangeRateObservations, pseudoRangeRateEstimates);
            LOG_DATA("{}: _kalmanFilter.z =\n{}", nameId(), _kalmanFilter.z);
        }
        else
        {
            _kalmanFilter.z = measurementInnovation_dz(pseudoRangeObservations, pseudoRangeEstimates, pseudoRangeRateObservations, pseudoRangeRateEstimates);
            LOG_DATA("{}: _kalmanFilter.z =\n{}", nameId(), _kalmanFilter.z);
        }
    }
    else // if (_frame == Frame::ECEF)
    {
        LOG_ERROR("{}: Update in ECEF-frame not implemented, yet.", nameId()); // TODO: implement update in e-Sys.
    }

    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_H, gnssObs->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_R, gnssObs->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_z, gnssObs->insTime);
    }
    LOG_DATA("{}:     KF.H =\n{}", nameId(), _kalmanFilter.H);
    LOG_DATA("{}:     KF.R =\n{}", nameId(), _kalmanFilter.R);
    LOG_DATA("{}:     KF.z =\n{}", nameId(), _kalmanFilter.z);

    if (_checkKalmanMatricesRanks && _kalmanFilter.H.rows() > 0) // Number of rows of H is 0, if there is no pseudorange in one epoch. Better skip this than crashing.
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R);
        auto rank = lu.rank();
        if (rank != _kalmanFilter.H.rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank);
        }
    }

    // 7. Calculate the Kalman gain matrix K_k
    // 9. Update the state vector estimate from x(-) to x(+)
    // 10. Update the error covariance matrix from P(-) to P(+)
    if (_showKalmanFilterOutputPins)
    {
        auto guard1 = requestOutputValueLock(OUTPUT_PORT_INDEX_K);
        auto guard2 = requestOutputValueLock(OUTPUT_PORT_INDEX_x);
        auto guard3 = requestOutputValueLock(OUTPUT_PORT_INDEX_P);
        _kalmanFilter.correctWithMeasurementInnovation();
    }
    else
    {
        _kalmanFilter.correctWithMeasurementInnovation();
    }

    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_K, gnssObs->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_x, gnssObs->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_P, gnssObs->insTime);
    }
    LOG_DATA("{}:     KF.K =\n{}", nameId(), _kalmanFilter.K);
    LOG_DATA("{}:     KF.x =\n{}", nameId(), _kalmanFilter.x);
    LOG_DATA("{}:     KF.P =\n{}", nameId(), _kalmanFilter.P);

    if (_checkKalmanMatricesRanks && _kalmanFilter.H.rows() > 0) // Number of rows of H is 0, if there is no pseudorange in one epoch. Better skip this than crashing.
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_kalmanFilter.H * _kalmanFilter.P * _kalmanFilter.H.transpose() + _kalmanFilter.R);
        auto rank = lu.rank();
        if (rank != _kalmanFilter.H.rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank);
        }

        Eigen::FullPivLU<Eigen::MatrixXd> luP(_kalmanFilter.P);
        rank = luP.rank();
        if (rank != _kalmanFilter.P.rows())
        {
            LOG_WARN("{}: P.rank = {}", nameId(), rank);
        }
    }

    _accumulatedAccelBiases += _kalmanFilter.x.block<3, 1>(9, 0) * (1. / SCALE_FACTOR_ACCELERATION);
    _accumulatedGyroBiases += _kalmanFilter.x.block<3, 1>(12, 0) * (1. / SCALE_FACTOR_ANGULAR_RATE);
    _recvClk.bias.value += _kalmanFilter.x(15, 0) / InsConst<>::C;
    _recvClk.drift.value += _kalmanFilter.x(16, 0) / InsConst<>::C;

    // Push out the new data
    auto tcKfInsGnssErrors = std::make_shared<TcKfInsGnssErrors>();
    tcKfInsGnssErrors->insTime = gnssObs->insTime;
    tcKfInsGnssErrors->positionError = _kalmanFilter.x.block<3, 1>(6, 0);
    tcKfInsGnssErrors->velocityError = _kalmanFilter.x.block<3, 1>(3, 0);
    tcKfInsGnssErrors->attitudeError = _kalmanFilter.x.block<3, 1>(0, 0) * (1. / SCALE_FACTOR_ATTITUDE);
    tcKfInsGnssErrors->b_biasAccel = _accumulatedAccelBiases;
    tcKfInsGnssErrors->b_biasGyro = _accumulatedGyroBiases;
    tcKfInsGnssErrors->recvClkOffset = _recvClk.bias.value * InsConst<>::C;
    tcKfInsGnssErrors->recvClkDrift = _recvClk.drift.value * InsConst<>::C;

    if (_frame == Frame::NED)
    {
        tcKfInsGnssErrors->positionError = tcKfInsGnssErrors->positionError.array() * Eigen::Array3d(1. / SCALE_FACTOR_LAT_LON, 1. / SCALE_FACTOR_LAT_LON, 1);
        tcKfInsGnssErrors->frame = TcKfInsGnssErrors::Frame::NED;
    }
    LOG_DATA("tcKfInsGnssErrors->positionError = {}", tcKfInsGnssErrors->positionError.transpose());
    LOG_DATA("tcKfInsGnssErrors->velocityError = {}", tcKfInsGnssErrors->velocityError.transpose());
    LOG_DATA("tcKfInsGnssErrors->attitudeError = {}", tcKfInsGnssErrors->attitudeError.transpose());
    LOG_DATA("tcKfInsGnssErrors->b_biasAccel = {}", tcKfInsGnssErrors->b_biasAccel.transpose());
    LOG_DATA("tcKfInsGnssErrors->b_biasGyro = {}", tcKfInsGnssErrors->b_biasGyro.transpose());
    LOG_DATA("tcKfInsGnssErrors->recvClkOffset = {}", tcKfInsGnssErrors->recvClkOffset);
    LOG_DATA("tcKfInsGnssErrors->recvClkDrift = {}", tcKfInsGnssErrors->recvClkDrift);

    // Closed loop
    if (_showKalmanFilterOutputPins)
    {
        auto guard = requestOutputValueLock(OUTPUT_PORT_INDEX_x);
        _kalmanFilter.x.setZero();
    }
    else
    {
        _kalmanFilter.x.setZero();
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_ERROR, tcKfInsGnssErrors);
}

// ###########################################################################################################
//                                             System matrix 𝐅
// ###########################################################################################################

Eigen::Matrix<double, 17, 17> NAV::TightlyCoupledKF::n_systemMatrix_F(const Eigen::Quaterniond& n_Quat_b,
                                                                      const Eigen::Vector3d& b_specForce_ib,
                                                                      const Eigen::Vector3d& n_omega_in,
                                                                      const Eigen::Vector3d& n_velocity,
                                                                      const Eigen::Vector3d& lla_position,
                                                                      double R_N,
                                                                      double R_E,
                                                                      double g_0,
                                                                      double r_eS_e,
                                                                      const Eigen::Vector3d& tau_bad,
                                                                      const Eigen::Vector3d& tau_bgd) const
{
    double latitude = lla_position(0); // Geodetic latitude of the body in [rad]
    double altitude = lla_position(2); // Geodetic height of the body in [m]

    Eigen::Vector3d beta_bad = 1. / tau_bad.array(); // Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    Eigen::Vector3d beta_bgd = 1. / tau_bgd.array(); // Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)

    // System matrix 𝐅
    // Math: \mathbf{F}^n = \begin{pmatrix} \mathbf{F}_{\dot{\psi},\psi}^n & \mathbf{F}_{\dot{\psi},\delta v}^n & \mathbf{F}_{\dot{\psi},\delta r}^n & \mathbf{0}_3 & \mathbf{C}_b^n & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{F}_{\delta \dot{v},\psi}^n & \mathbf{F}_{\delta \dot{v},\delta v}^n & \mathbf{F}_{\delta \dot{v},\delta r}^n & \mathbf{C}_b^n & \mathbf{0}_3 & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_3 & \mathbf{F}_{\delta \dot{r},\delta v}^n & \mathbf{F}_{\delta \dot{r},\delta r}^n & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} & \mathbf{0}_3 & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & 0 & 1 \\ \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & 0 & 0 \end{pmatrix}
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(17, 17);

    F.block<3, 3>(0, 0) = n_F_dpsi_dpsi(n_omega_in);
    F.block<3, 3>(0, 3) = n_F_dpsi_dv(latitude, altitude, R_N, R_E);
    F.block<3, 3>(0, 6) = n_F_dpsi_dr(latitude, altitude, n_velocity, R_N, R_E);
    F.block<3, 3>(0, 12) = n_F_dpsi_dw(n_Quat_b.toRotationMatrix());
    F.block<3, 3>(3, 0) = n_F_dv_dpsi(n_Quat_b * b_specForce_ib);
    F.block<3, 3>(3, 3) = n_F_dv_dv(n_velocity, latitude, altitude, R_N, R_E);
    F.block<3, 3>(3, 6) = n_F_dv_dr(n_velocity, latitude, altitude, R_N, R_E, g_0, r_eS_e);
    F.block<3, 3>(3, 9) = n_F_dv_df(n_Quat_b.toRotationMatrix());
    F.block<3, 3>(6, 3) = n_F_dr_dv(latitude, altitude, R_N, R_E);
    F.block<3, 3>(6, 6) = n_F_dr_dr(n_velocity, latitude, altitude, R_N, R_E);
    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        F.block<3, 3>(9, 9) = n_F_df_df(_randomProcessAccel == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bad);
        F.block<3, 3>(12, 12) = n_F_dw_dw(_randomProcessGyro == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bgd);
    }

    F.middleRows<3>(0) *= SCALE_FACTOR_ATTITUDE; // 𝜓' [deg / s] = 180/π * ... [rad / s]
    F.middleCols<3>(0) *= 1. / SCALE_FACTOR_ATTITUDE;

    // F.middleRows<3>(3) *= 1.; // 𝛿v' [m / s^2] = 1 * [m / s^2]
    // F.middleCols<3>(3) *= 1. / 1.;

    F.middleRows<2>(6) *= SCALE_FACTOR_LAT_LON; // 𝛿ϕ' [pseudometre / s] = R0 * [rad / s]
    F.middleCols<2>(6) *= 1. / SCALE_FACTOR_LAT_LON;
    // F.middleRows<1>(8) *= 1.; // 𝛿h' [m / s] = 1 * [m / s]
    // F.middleCols<1>(8) *= 1. / 1.;

    F.middleRows<3>(9) *= SCALE_FACTOR_ACCELERATION; // 𝛿f' [mg / s] = 1e3 / g * [m / s^3]
    F.middleCols<3>(9) *= 1. / SCALE_FACTOR_ACCELERATION;

    F.middleRows<3>(12) *= SCALE_FACTOR_ANGULAR_RATE; // 𝛿ω' [mrad / s^2] = 1e3 * [rad / s^2]
    F.middleCols<3>(12) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    // Change in clock offset = clock drift
    F(15, 16) = 1;

    return F;
}

Eigen::Matrix<double, 17, 17> NAV::TightlyCoupledKF::e_systemMatrix_F(const Eigen::Quaterniond& e_Quat_b,
                                                                      const Eigen::Vector3d& b_specForce_ib,
                                                                      const Eigen::Vector3d& e_position,
                                                                      const Eigen::Vector3d& e_gravitation,
                                                                      double r_eS_e,
                                                                      const Eigen::Vector3d& e_omega_ie,
                                                                      const Eigen::Vector3d& tau_bad,
                                                                      const Eigen::Vector3d& tau_bgd) const
{
    Eigen::Vector3d beta_bad = 1. / tau_bad.array(); // Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    Eigen::Vector3d beta_bgd = 1. / tau_bgd.array(); // Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)

    // System matrix 𝐅
    // Math: \mathbf{F}^e = \begin{pmatrix} \mathbf{F}_{\dot{\psi},\psi}^e & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{C}_b^e & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{F}_{\delta \dot{v},\psi}^e & \mathbf{F}_{\delta \dot{v},\delta v}^e & \mathbf{F}_{\delta \dot{v},\delta r}^e & \mathbf{C}_b^e & \mathbf{0}_3 & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_3 & \mathbf{F}_{\delta \dot{r},\delta v}^e & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} & \mathbf{0}_3 & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} & \mathbf{0}_{3,1} & \mathbf{0}_{3,1} \\ \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & 0 & 1 \\ \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & \mathbf{0}_{1,3} & 0 & 0 \end{pmatrix}
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(17, 17);

    F.block<3, 3>(0, 0) = e_F_dpsi_dpsi(e_omega_ie.z());
    F.block<3, 3>(0, 12) = e_F_dpsi_dw(e_Quat_b.toRotationMatrix());
    F.block<3, 3>(3, 0) = e_F_dv_dpsi(e_Quat_b * b_specForce_ib);
    F.block<3, 3>(3, 3) = e_F_dv_dv(e_omega_ie.z());
    F.block<3, 3>(3, 6) = e_F_dv_dr(e_position, e_gravitation, r_eS_e, e_omega_ie);
    F.block<3, 3>(3, 9) = e_F_dv_df(e_Quat_b.toRotationMatrix());
    F.block<3, 3>(6, 3) = e_F_dr_dv();
    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        F.block<3, 3>(9, 9) = e_F_df_df(_randomProcessAccel == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bad);
        F.block<3, 3>(12, 12) = e_F_dw_dw(_randomProcessGyro == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bgd);
    }

    F.middleRows<3>(0) *= SCALE_FACTOR_ATTITUDE; // 𝜓' [deg / s] = 180/π * ... [rad / s]
    F.middleCols<3>(0) *= 1. / SCALE_FACTOR_ATTITUDE;

    // F.middleRows<3>(3) *= 1.; // 𝛿v' [m / s^2] = 1 * [m / s^2]
    // F.middleCols<3>(3) *= 1. / 1.;

    // F.middleRows<3>(6) *= 1.; // 𝛿r' [m / s] = 1 * [m / s]
    // F.middleCols<3>(6) *= 1. / 1.;

    F.middleRows<3>(9) *= SCALE_FACTOR_ACCELERATION; // 𝛿f' [mg / s] = 1e3 / g * [m / s^3]
    F.middleCols<3>(9) *= 1. / SCALE_FACTOR_ACCELERATION;

    F.middleRows<3>(12) *= SCALE_FACTOR_ANGULAR_RATE; // 𝛿ω' [mrad / s^2] = 1e3 * [rad / s^2]
    F.middleCols<3>(12) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    // Change in clock offset = clock drift
    F(15, 16) = 1;

    return F;
}

// ###########################################################################################################
//                                    Noise input matrix 𝐆 & Noise scale matrix 𝐖
//                                     System noise covariance matrix 𝐐
// ###########################################################################################################

Eigen::Matrix<double, 17, 14> NAV::TightlyCoupledKF::noiseInputMatrix_G(const Eigen::Quaterniond& ien_Quat_b)
{
    // DCM matrix from body to navigation frame
    Eigen::Matrix3d ien_Dcm_b = ien_Quat_b.toRotationMatrix();

    // Math: \mathbf{G}_{a} = \begin{bmatrix} \mathbf{G}_{INS} & 0 \\ 0 & \mathbf{G}_{GNSS} \end{bmatrix} = \begin{bmatrix} -\mathbf{C}_b^{i,e,n} & 0 & 0 & 0 & 0 & 0 \\ 0 & \mathbf{C}_b^{i,e,n} & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & \mathbf{I}_3 & 0 & 0 & 0 \\ 0 & 0 & 0 & \mathbf{I}_3 & 0 & 0 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \end{bmatrix}
    Eigen::Matrix<double, 17, 14> G = Eigen::Matrix<double, 17, 14>::Zero();

    // G_INS
    G.block<3, 3>(0, 0) = SCALE_FACTOR_ATTITUDE * -ien_Dcm_b;
    G.block<3, 3>(3, 3) = ien_Dcm_b;
    G.block<3, 3>(9, 6) = SCALE_FACTOR_ACCELERATION * Eigen::Matrix3d::Identity();
    G.block<3, 3>(12, 9) = SCALE_FACTOR_ANGULAR_RATE * Eigen::Matrix3d::Identity();

    // G_GNSS
    G.block<2, 2>(15, 12) = Eigen::Matrix2d::Identity();

    return G;
}

Eigen::Matrix<double, 14, 14> NAV::TightlyCoupledKF::noiseScaleMatrix_W(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                        const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                        const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                        const double& sigma2_cPhi, const double& sigma2_cf)
{
    // Math: \mathbf{W} = \begin{bmatrix} \mathbf{W}_{INS} & 0 \\ 0 & \mathbf{W}_{GNSS} \end{bmatrix}
    Eigen::Matrix<double, 14, 14> W = Eigen::Matrix<double, 14, 14>::Zero();

    // W_INS
    W.block<3, 3>(0, 0).diagonal() = sigma2_rg;                                                                                               // S_rg
    W.block<3, 3>(3, 3).diagonal() = sigma2_ra;                                                                                               // S_ra
    W.block<3, 3>(6, 6).diagonal() = _randomProcessAccel == RandomProcess::RandomWalk ? sigma2_bad : psdBiasGaussMarkov(sigma2_bad, tau_bad); // S_bad
    W.block<3, 3>(9, 9).diagonal() = _randomProcessGyro == RandomProcess::RandomWalk ? sigma2_bgd : psdBiasGaussMarkov(sigma2_bgd, tau_bgd);  // S_bgd

    // W_GNSS
    W(12, 12) = sigma2_cPhi; // S_cPhi
    W(13, 13) = sigma2_cf;   // S_cf

    return W;
}

Eigen::Matrix<double, 17, 17> NAV::TightlyCoupledKF::n_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                     const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                     const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                                     const double& sigma2_cPhi, const double& sigma2_cf,
                                                                                     const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p,
                                                                                     const Eigen::Matrix3d& n_Dcm_b, const double& tau_s)
{
    // Math: \mathbf{Q}^n = \begin{pmatrix} \mathbf{Q}_{INS}^n & 0 \\ 0 & \mathbf{Q}_{GNSS} \end{pmatrix} \ \mathrm{with} \ \mathbf{Q}_{INS}^n = \begin{pmatrix} \mathbf{Q}_{11} & {\mathbf{Q}_{21}^n}^T & {\mathbf{Q}_{31}^n}^T & \mathbf{0}_3 & {\mathbf{Q}_{51}^n}^T \\ \mathbf{Q}_{21}^n & \mathbf{Q}_{22}^n & {\mathbf{Q}_{32}^n}^T & {\mathbf{Q}_{42}^n}^T & \mathbf{Q}_{25}^n \\ \mathbf{Q}_{31}^n & \mathbf{Q}_{32}^n & \mathbf{Q}_{33}^n & \mathbf{Q}_{34}^n & \mathbf{Q}_{35}^n \\ \mathbf{0}_3 & \mathbf{Q}_{42}^n & {\mathbf{Q}_{34}^n}^T & S_{bad}\tau_s\mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{Q}_{51}^n & \mathbf{Q}_{52}^n & {\mathbf{Q}_{35}^n}^T & \mathbf{0}_3 & S_{bgd}\tau_s\mathbf{I}_3 \end{pmatrix} \ \text{P. Groves}\,(14.80) \ \mathrm{and} \ \mathbf{Q}_{GNSS} = \begin{pmatrix} S_{c\phi}\tau_s + \frac{1}{3}S_{cf}\tau_s^3 & \frac{1}{2}S_{cf}\tau_s^2 \\ \frac{1}{2}S_{cf}\tau_s^2 & S_{cf}\tau_s \end{pmatrix} \ \text{P. Groves}\,(14.88)
    Eigen::Vector3d S_ra = sigma2_ra * tau_s;
    Eigen::Vector3d S_rg = sigma2_rg * tau_s;
    Eigen::Vector3d S_bad = sigma2_bad.array() / tau_bad.array();
    Eigen::Vector3d S_bgd = sigma2_bgd.array() / tau_bgd.array();

    // double S_cPhi = psdClockPhaseDrift(sigma2_cPhi, tau_s);
    // double S_cf = psdClockFreqDrift(sigma2_cf, tau_s);

    Eigen::Matrix3d b_Dcm_n = n_Dcm_b.transpose();

    Eigen::Matrix<double, 17, 17> Q = Eigen::Matrix<double, 17, 17>::Zero();
    Q.block<3, 3>(0, 0) = Q_psi_psi(S_rg, S_bgd, tau_s);                              // Q_11
    Q.block<3, 3>(3, 0) = ien_Q_dv_psi(S_rg, S_bgd, n_F_21, tau_s);                   // Q_21
    Q.block<3, 3>(3, 3) = ien_Q_dv_dv(S_ra, S_bad, S_rg, S_bgd, n_F_21, tau_s);       // Q_22
    Q.block<3, 3>(3, 12) = ien_Q_dv_domega(S_bgd, n_F_21, n_Dcm_b, tau_s);            // Q_25
    Q.block<3, 3>(6, 0) = n_Q_dr_psi(S_rg, S_bgd, n_F_21, T_rn_p, tau_s);             // Q_31
    Q.block<3, 3>(6, 3) = n_Q_dr_dv(S_ra, S_bad, S_rg, S_bgd, n_F_21, T_rn_p, tau_s); // Q_32
    Q.block<3, 3>(6, 6) = n_Q_dr_dr(S_ra, S_bad, S_rg, S_bgd, n_F_21, T_rn_p, tau_s); // Q_33
    Q.block<3, 3>(6, 9) = n_Q_dr_df(S_bgd, T_rn_p, n_Dcm_b, tau_s);                   // Q_34
    Q.block<3, 3>(6, 12) = n_Q_dr_domega(S_bgd, n_F_21, T_rn_p, n_Dcm_b, tau_s);      // Q_35
    Q.block<3, 3>(9, 3) = Q_df_dv(S_bad, b_Dcm_n, tau_s);                             // Q_42
    Q.block<3, 3>(9, 9) = Q_df_df(S_bad, tau_s);                                      // Q_44
    Q.block<3, 3>(12, 0) = Q_domega_psi(S_bgd, b_Dcm_n, tau_s);                       // Q_51
    Q.block<3, 3>(12, 12) = Q_domega_domega(S_bgd, tau_s);                            // Q_55

    Q.block<3, 3>(0, 3) = Q.block<3, 3>(3, 0).transpose();   // Q_21^T
    Q.block<3, 3>(0, 6) = Q.block<3, 3>(6, 0).transpose();   // Q_31^T
    Q.block<3, 3>(3, 6) = Q.block<3, 3>(6, 3).transpose();   // Q_32^T
    Q.block<3, 3>(9, 6) = Q.block<3, 3>(6, 9).transpose();   // Q_34^T
    Q.block<3, 3>(12, 3) = Q.block<3, 3>(3, 12).transpose(); // Q_25^T
    Q.block<3, 3>(12, 6) = Q.block<3, 3>(6, 12).transpose(); // Q_35^T
    Q.block<3, 3>(3, 9) = Q.block<3, 3>(9, 3).transpose();   // Q_42^T
    Q.block<3, 3>(0, 12) = Q.block<3, 3>(12, 0).transpose(); // Q_51^T

    Q.middleRows<3>(0) *= SCALE_FACTOR_ATTITUDE;
    Q.middleRows<2>(6) *= SCALE_FACTOR_LAT_LON;
    Q.middleRows<3>(9) *= SCALE_FACTOR_ACCELERATION;
    Q.middleRows<3>(12) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.middleCols<3>(0) *= SCALE_FACTOR_ATTITUDE;
    Q.middleCols<2>(6) *= SCALE_FACTOR_LAT_LON;
    Q.middleCols<3>(9) *= SCALE_FACTOR_ACCELERATION;
    Q.middleCols<3>(12) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.block<2, 2>(15, 15) = Q_gnss(sigma2_cPhi, sigma2_cf, tau_s);

    return Q;
}

Eigen::Matrix<double, 17, 17> NAV::TightlyCoupledKF::e_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                                                     const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                                                     const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                                                     const double& sigma2_cPhi, const double& sigma2_cf,
                                                                                     const Eigen::Matrix3d& e_F_21,
                                                                                     const Eigen::Matrix3d& e_Dcm_b, const double& tau_s)
{
    // Math: \mathbf{Q}^e = \begin{pmatrix} \mathbf{Q}_{INS}^e & 0 \\ 0 & \mathbf{Q}_{GNSS} \end{pmatrix} \ \mathrm{with} \ \mathbf{Q}_{INS}^e = \begin{pmatrix} \mathbf{Q}_{11} & {\mathbf{Q}_{21}^e}^T & {\mathbf{Q}_{31}^e}^T & \mathbf{0}_3 & {\mathbf{Q}_{51}^e}^T \\ \mathbf{Q}_{21}^e & \mathbf{Q}_{22}^e & {\mathbf{Q}_{32}^e}^T & {\mathbf{Q}_{42}^e}^T & \mathbf{Q}_{25}^e \\ \mathbf{Q}_{31}^e & \mathbf{Q}_{32}^e & \mathbf{Q}_{33}^e & \mathbf{Q}_{34}^e & \mathbf{Q}_{35}^e \\ \mathbf{0}_3 & \mathbf{Q}_{42}^e & {\mathbf{Q}_{34}^e}^T & S_{bad}\tau_s\mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{Q}_{51}^e & \mathbf{Q}_{52}^e & {\mathbf{Q}_{35}^e}^T & \mathbf{0}_3 & S_{bgd}\tau_s\mathbf{I}_3 \end{pmatrix} \ \text{P. Groves}\,(14.80) \ \mathrm{and} \ \mathbf{Q}_{GNSS} = \begin{pmatrix} S_{c\phi}\tau_s + \frac{1}{3}S_{cf}\tau_s^3 & \frac{1}{2}S_{cf}\tau_s^2 \\ \frac{1}{2}S_{cf}\tau_s^2 & S_{cf}\tau_s \end{pmatrix} \ \text{P. Groves}\,(14.88)

    Eigen::Vector3d S_ra = sigma2_ra * tau_s;
    Eigen::Vector3d S_rg = sigma2_rg * tau_s;
    Eigen::Vector3d S_bad = sigma2_bad.array() / tau_bad.array();
    Eigen::Vector3d S_bgd = sigma2_bgd.array() / tau_bgd.array();

    // double S_cPhi = psdClockPhaseDrift(sigma2_cPhi, tau_s);
    // double S_cf = psdClockFreqDrift(sigma2_cf, tau_s);

    Eigen::Matrix3d b_Dcm_e = e_Dcm_b.transpose();

    Eigen::Matrix<double, 17, 17> Q = Eigen::Matrix<double, 17, 17>::Zero();
    Q.block<3, 3>(0, 0) = Q_psi_psi(S_rg, S_bgd, tau_s);                        // Q_11
    Q.block<3, 3>(3, 0) = ien_Q_dv_psi(S_rg, S_bgd, e_F_21, tau_s);             // Q_21
    Q.block<3, 3>(3, 3) = ien_Q_dv_dv(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s); // Q_22
    Q.block<3, 3>(3, 12) = ien_Q_dv_domega(S_bgd, e_F_21, e_Dcm_b, tau_s);      // Q_25
    Q.block<3, 3>(6, 0) = ie_Q_dr_psi(S_rg, S_bgd, e_F_21, tau_s);              // Q_31
    Q.block<3, 3>(6, 3) = ie_Q_dr_dv(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s);  // Q_32
    Q.block<3, 3>(6, 6) = ie_Q_dr_dr(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s);  // Q_33
    Q.block<3, 3>(6, 9) = ie_Q_dr_df(S_bgd, e_Dcm_b, tau_s);                    // Q_34
    Q.block<3, 3>(6, 12) = ie_Q_dr_domega(S_bgd, e_F_21, e_Dcm_b, tau_s);       // Q_35
    Q.block<3, 3>(9, 3) = Q_df_dv(S_bad, b_Dcm_e, tau_s);                       // Q_42
    Q.block<3, 3>(9, 9) = Q_df_df(S_bad, tau_s);                                // Q_44
    Q.block<3, 3>(12, 0) = Q_domega_psi(S_bgd, b_Dcm_e, tau_s);                 // Q_51
    Q.block<3, 3>(12, 12) = Q_domega_domega(S_bgd, tau_s);                      // Q_55

    Q.block<3, 3>(0, 3) = Q.block<3, 3>(3, 0).transpose();   // Q_21^T
    Q.block<3, 3>(0, 6) = Q.block<3, 3>(6, 0).transpose();   // Q_31^T
    Q.block<3, 3>(3, 6) = Q.block<3, 3>(6, 3).transpose();   // Q_32^T
    Q.block<3, 3>(9, 6) = Q.block<3, 3>(6, 9).transpose();   // Q_34^T
    Q.block<3, 3>(12, 3) = Q.block<3, 3>(3, 12).transpose(); // Q_25^T
    Q.block<3, 3>(12, 6) = Q.block<3, 3>(6, 12).transpose(); // Q_35^T
    Q.block<3, 3>(3, 9) = Q.block<3, 3>(9, 3).transpose();   // Q_42^T
    Q.block<3, 3>(0, 12) = Q.block<3, 3>(12, 0).transpose(); // Q_51^T

    Q.middleRows<3>(0) *= SCALE_FACTOR_ATTITUDE;
    Q.middleRows<3>(9) *= SCALE_FACTOR_ACCELERATION;
    Q.middleRows<3>(12) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.middleCols<3>(0) *= SCALE_FACTOR_ATTITUDE;
    Q.middleCols<3>(9) *= SCALE_FACTOR_ACCELERATION;
    Q.middleCols<3>(12) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.block<2, 2>(15, 15) = Q_gnss(sigma2_cPhi, sigma2_cf, tau_s);

    return Q;
}

// ###########################################################################################################
//                                         Error covariance matrix P
// ###########################################################################################################

Eigen::Matrix<double, 17, 17> NAV::TightlyCoupledKF::initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& variance_angles,
                                                                                     const Eigen::Vector3d& variance_vel,
                                                                                     const Eigen::Vector3d& variance_pos,
                                                                                     const Eigen::Vector3d& variance_accelBias,
                                                                                     const Eigen::Vector3d& variance_gyroBias,
                                                                                     const double& variance_clkPhase,
                                                                                     const double& variance_clkFreq) const
{
    double scaleFactorPosition = _frame == Frame::NED ? SCALE_FACTOR_LAT_LON : 1.0;

    // 𝐏 Error covariance matrix
    Eigen::Matrix<double, 17, 17> P = Eigen::Matrix<double, 17, 17>::Zero();

    P.diagonal() << std::pow(SCALE_FACTOR_ATTITUDE, 2) * variance_angles, // Flight Angles covariance
        variance_vel,                                                     // Velocity covariance
        std::pow(scaleFactorPosition, 2) * variance_pos(0),               // Latitude/Pos X covariance
        std::pow(scaleFactorPosition, 2) * variance_pos(1),               // Longitude/Pos Y covariance
        variance_pos(2),                                                  // Altitude/Pos Z covariance
        std::pow(SCALE_FACTOR_ACCELERATION, 2) * variance_accelBias,      // Accelerometer Bias covariance
        std::pow(SCALE_FACTOR_ANGULAR_RATE, 2) * variance_gyroBias,       // Gyroscope Bias covariance
        variance_clkPhase,                                                // Receiver clock phase drift covariance
        variance_clkFreq;                                                 // Receiver clock frequency drift covariance

    return P;
}

// ###########################################################################################################
//                                                  Update
// ###########################################################################################################

Eigen::MatrixXd NAV::TightlyCoupledKF::n_measurementMatrix_H(const double& R_N, const double& R_E, const Eigen::Vector3d& lla_position, const std::vector<Eigen::Vector3d>& n_lineOfSightUnitVectors, std::vector<double>& pseudoRangeRateObservations)
{
    // Math: \mathbf{H}_{G,k}^n \approx \begin{pmatrix} 0_{1,3} & 0_{1,3} & {\mathbf{h}_{\rho p}^1}^\text{T} & 0_{1,3} & 0_{1,3} & 1 & 0 \\ 0_{1,3} & 0_{1,3} & {\mathbf{h}_{\rho p}^2}^\text{T} & 0_{1,3} & 0_{1,3} & 1 & 0 \\ \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\ 0_{1,3} & 0_{1,3} & {\mathbf{h}_{\rho p}^m}^\text{T} & 0_{1,3} & 0_{1,3} & 1 & 0 \\ - & - & - & - & - & - & - \\ 0_{1,3} & {\mathbf{u}_{a1}^n}^\text{T} & 0_{1,3} & 0_{1,3} & 0_{1,3} & 0 & 1 \\ 0_{1,3} & {\mathbf{u}_{a2}^n}^\text{T} & 0_{1,3} & 0_{1,3} & 0_{1,3} & 0 & 1 \\ \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\ 0_{1,3} & {\mathbf{u}_{am}^n}^\text{T} & 0_{1,3} & 0_{1,3} & 0_{1,3} & 0 & 1 \end{pmatrix}_{\mathbf{x} = \hat{\mathbf{x}}_k^-} \qquad \text{P. Groves}\,(14.127)

    auto numSats = static_cast<uint8_t>(n_lineOfSightUnitVectors.size());

    auto numMeasurements = static_cast<Eigen::Index>(2 * numSats);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(numMeasurements, 17);

    Eigen::Vector3d h_rhoP;

    for (size_t j = 0; j < numSats; j++)
    {
        h_rhoP << (R_N + lla_position(2)) * n_lineOfSightUnitVectors[j](0),
            (R_E + lla_position(2)) * std::cos(lla_position(0)) * n_lineOfSightUnitVectors[j](1),
            -n_lineOfSightUnitVectors[j](2);

        auto i = static_cast<uint8_t>(j);

        H.block<1, 3>(i, 6) = h_rhoP.transpose();
        H(i, 15) = 1;

        // Take pseudorange-rate observation only into account, if available (otherwise, this ruins the stochastics)
        if (!std::isnan(pseudoRangeRateObservations[j]))
        {
            H.block<1, 3>(numSats + i, 3) = n_lineOfSightUnitVectors[j].transpose();
            H(numSats + i, 16) = 1;
        }
    }

    // H.middleCols<3>(0) *= 1. / SCALE_FACTOR_ATTITUDE; // Only zero elements
    H.middleCols<2>(6) *= 1. / SCALE_FACTOR_LAT_LON;
    // H.middleCols<3>(9) *= 1. / SCALE_FACTOR_ACCELERATION; // Only zero elements
    // H.middleCols<3>(12) *= 1. / SCALE_FACTOR_ANGULAR_RATE; // Only zero elements

    return H;
}

Eigen::MatrixXd NAV::TightlyCoupledKF::measurementNoiseCovariance_R(const double& sigma_rhoZ, const double& sigma_rZ, const std::vector<double>& satElevation)
{
    // Math: \mathbf{R}_G = \begin{pmatrix} \sigma_{\rho1}^2 & 0 & \dots & 0 & | & 0 & 0 & \dots & 0 \\ 0 & \sigma_{\rho2}^2 & \dots & 0 & | & 0 & 0 & \dots & 0 \\ \vdots & \vdots & \ddots & \vdots & | & \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dots & \sigma_{\rho m}^2 & | & 0 & 0 & \dots & 0 \\ - & - & - & - & - & - & - & - & - \\ 0 & 0 & \dots & 0 & | & \sigma_{r1}^2 & 0 & \dots & 0 \\ 0 & 0 & \dots & 0 & | & 0 & \sigma_{r 2}^2 & \dots & 0 \\ \vdots & \vdots & \ddots & \vdots & | & \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dots & 0 & | & 0 & 0 & \dots & \sigma_{rm}^2 \end{pmatrix} \qquad \text{P. Groves}\,(9.168)

    auto numSats = static_cast<uint8_t>(satElevation.size());

    auto numMeasurements = static_cast<Eigen::Index>(2 * numSats);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(numMeasurements, numMeasurements);

    for (size_t j = 0; j < numSats; j++)
    {
        auto sigma_rho2 = sigma2(satElevation[j], sigma_rhoZ, 0., 0., 0., 0.);
        auto sigma_r2 = sigma2(satElevation[j], sigma_rZ, 0., 0., 0., 0.);

        auto i = static_cast<uint8_t>(j);

        R(i, i) = sigma_rho2;
        R(numSats + i, numSats + i) = sigma_r2;
    }

    return R;
}

double NAV::TightlyCoupledKF::sigma2(const double& satElevation, const double& sigma_Z, const double& sigma_C, const double& sigma_A, const double& CN0, const double& rangeAccel)
{
    // Math: \sigma_{\rho j}^2 = \frac{1}{\sin^2{\theta_{nu}^{aj}}}\left(\sigma_{\rho Z}^2 + \frac{\sigma_{\rho c}^2}{(c/n_0)_j} + \sigma_{\rho a}^2 \ddot{r}_{aj}^2 \right) \qquad \text{P. Groves}\,(9.168)\,\left(\text{extension of}\,(9.137)\right)

    auto sigma2 = 1. / std::pow(std::sin(satElevation), 2) * std::pow(sigma_Z, 2);
    if (CN0 != 0.)
    {
        sigma2 += 1. / std::pow(std::sin(satElevation), 2) * std::pow(sigma_C, 2) / CN0;
    }
    if (rangeAccel != 0.)
    {
        sigma2 += 1. / std::pow(std::sin(satElevation), 2) * std::pow(sigma_A, 2) * std::pow(rangeAccel, 2);
    }

    return sigma2;
}

Eigen::MatrixXd NAV::TightlyCoupledKF::measurementInnovation_dz(const std::vector<double>& pseudoRangeObservations, const std::vector<double>& pseudoRangeEstimates, const std::vector<double>& pseudoRangeRateObservations, const std::vector<double>& pseudoRangeRateEstimates)
{
    // Math: \delta \mathbf{z}^-_{\mathbf{G},k} = \begin{pmatrix} \delta \mathbf{z}^-_{\rho,k} \\ \delta \mathbf{z}^-_{r,k} \end{pmatrix} = \begin{pmatrix} \rho^1_{a,C} - \hat{\rho}^{1-}_{a,C}, \rho^2_{a,C} - \hat{\rho}^{2-}_{a,C}, \cdots, \rho^m_{a,C} - \hat{\rho}^{m-}_{a,C} \\ \dot{\rho}^1_{a,C} - \hat{\dot{\rho}}^{1-}_{a,C}, \dot{\rho}^2_{a,C} - \hat{\dot{\rho}}^{2-}_{a,C}, \cdots, \dot{\rho}^m_{a,C} - \hat{\dot{\rho}}^{m-}_{a,C} \end{pmatrix}_k \qquad \text{P. Groves}\,(14.119)

    auto numSats = static_cast<uint8_t>(pseudoRangeObservations.size());

    auto numMeasurements = static_cast<Eigen::Index>(2 * numSats);
    Eigen::MatrixXd deltaZ = Eigen::MatrixXd::Zero(numMeasurements, 1);

    for (size_t j = 0; j < numSats; j++)
    {
        deltaZ(static_cast<Eigen::Index>(j), 0) = pseudoRangeObservations[j] - pseudoRangeEstimates[j];
        if (!std::isnan(pseudoRangeRateObservations[j]))
        {
            deltaZ(static_cast<Eigen::Index>(numSats + j), 0) = pseudoRangeRateObservations[j] - pseudoRangeRateEstimates[j];
        }
    }

    return deltaZ;
}