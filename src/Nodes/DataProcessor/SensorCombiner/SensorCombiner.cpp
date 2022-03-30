#include "SensorCombiner.hpp"

#include "util/Logger.hpp"

#include "Navigation/Math/Math.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include <imgui_internal.h>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::SensorCombiner::SensorCombiner()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 483, 350 }; //TODO: adapt

    updateNumberOfInputPins();

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
}

NAV::SensorCombiner::~SensorCombiner()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SensorCombiner::typeStatic()
{
    return "SensorCombiner";
}

std::string NAV::SensorCombiner::type() const
{
    return typeStatic();
}

std::string NAV::SensorCombiner::category()
{
    return "Data Processor";
}

void NAV::SensorCombiner::guiConfig()
{
    // TODO: adapt _maxSizeImuObservations to number of sensors?

    if (ImGui::BeginTable(fmt::format("Pin Settings##{}", size_t(id)).c_str(), inputPins.size() > 1 ? 2 : 1,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Pin");
        if (inputPins.size() > 1)
        {
            ImGui::TableSetupColumn("");
        }
        ImGui::TableHeadersRow();

        for (size_t pinIndex = 0; pinIndex < _pinData.size(); ++pinIndex)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); // Pin

            // if (pinIndex == 0 && _dragAndDropPinIndex > 0)
            // {
            //     showDragDropTargetPin(0);
            // }

            bool selectablePinDummy = false;
            ImGui::Selectable(fmt::format("{}##{}", inputPins.at(pinIndex).name, size_t(id)).c_str(), &selectablePinDummy);
            // if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
            // {
            //     dragAndDropPinStillInProgress = true;
            //     _dragAndDropPinIndex = static_cast<int>(pinIndex);
            //     // Data is copied into heap inside the drag and drop
            //     ImGui::SetDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str(), &pinIndex, sizeof(pinIndex));
            //     ImGui::TextUnformatted(inputPins.at(pinIndex).name.c_str());
            //     ImGui::EndDragDropSource();
            // }
            // if (_dragAndDropPinIndex >= 0
            //     && pinIndex != static_cast<size_t>(_dragAndDropPinIndex - 1)
            //     && pinIndex != static_cast<size_t>(_dragAndDropPinIndex))
            // {
            //     showDragDropTargetPin(pinIndex + 1);
            // }
            // if (ImGui::IsItemHovered())
            // {
            //     ImGui::SetTooltip("This item can be dragged to reorder the pins");
            // }
            if (inputPins.size() > 1)
            {
                ImGui::TableNextColumn(); // Delete
                if (ImGui::Button(fmt::format("x##{} - {}", size_t(id), pinIndex).c_str()))
                {
                    nm::DeleteInputPin(inputPins.at(pinIndex).id);
                    _pinData.erase(_pinData.begin() + static_cast<int64_t>(pinIndex));
                    --_nInputPins;
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Delete the pin");
                }
            }
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); // Pin
        if (ImGui::Button(fmt::format("Add Pin##{}", size_t(id)).c_str()))
        {
            ++_nInputPins;
            LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nInputPins);
            flow::ApplyChanges();
            updateNumberOfInputPins();
        }

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::SensorCombiner::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    // TODO

    return j;
}

void NAV::SensorCombiner::restore([[maybe_unused]] json const& j) //TODO: remove [[maybe_unused]]
{
    LOG_TRACE("{}: called", nameId());

    // TODO
}

bool NAV::SensorCombiner::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _imuObservations.clear();

    LOG_DEBUG("SensorCombiner initialized");

    return true;
}

void NAV::SensorCombiner::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::SensorCombiner::updateNumberOfInputPins()
{
    while (inputPins.size() < _nInputPins)
    {
        nm::CreateInputPin(this, fmt::format("Pin {}", inputPins.size() + 1).c_str(), Pin::Type::Flow,
                           { NAV::ImuObs::type() }, &SensorCombiner::recvSignal);
        _pinData.emplace_back();
    }
    while (inputPins.size() > _nInputPins)
    {
        nm::DeleteInputPin(inputPins.back().id);
        _pinData.pop_back();
    }
}

void NAV::SensorCombiner::recvSignal(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto imuObs = std::static_pointer_cast<const ImuObs>(nodeData);

    if (!imuObs->insTime.has_value() && !imuObs->timeSinceStartup.has_value())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime/timeSinceStartup)", nameId());
        return;
    }

    // Add imuObs tₖ to the start of the list
    _imuObservations.push_front(imuObs);

    // Remove observations at the end of the list till the max size is reached
    while (_imuObservations.size() > _maxSizeImuObservations)
    {
        LOG_WARN("{}: Receive new Imu observation, but list is full --> discarding oldest observation", nameId());
        _imuObservations.pop_back();
    }

    // First ImuObs
    if (_imuObservations.size() == 1)
    {
        // Push out a message with the initial state and a matching imu Observation
        auto ImuObsOut = std::make_shared<ImuObs>(_imuPos);

        ImuObsOut->insTime = imuObs->insTime;
        // ImuObsOut->imuObs = imuObs;

        invokeCallbacks(OUTPUT_PORT_INDEX_COMBINED_SIGNAL, ImuObsOut);
        return;
    }

    // If enough imu observations, combine the signals
    if (_imuObservations.size() == _maxSizeImuObservations)
    {
        combineSignals();
    }
}

void NAV::SensorCombiner::combineSignals()
{
    // -------------------------------------------------- Construct the message to send out ----------------------------------------------------

    // for (size_t pinIndex = 0; pinIndex < _pinData.size(); ++pinIndex)
    // {
    //     auto& pinData = _pinData.at(pinIndex);    // IMU Observation of sensor #1
    //     const std::shared_ptr<const ImuObs>& imuObs__s0 = _imuObservations.at(0);
    //     // IMU Observation at the time tₖ₋₁
    //     const std::shared_ptr<const ImuObs>& imuObs__t1 = _imuObservations.at(1);
    // }

    auto obs = _imuObservations.front(); // TODO: this should be the combined IMU observation
    invokeCallbacks(OUTPUT_PORT_INDEX_COMBINED_SIGNAL, obs);
}

Eigen::Matrix<double, 9, 9> stateTransitionMatrix_Phi([[maybe_unused]] double dt, [[maybe_unused]] uint8_t M)
{
    Eigen::Matrix<double, 9, 9> nullMatrix{};
    return nullMatrix;
}

Eigen::Matrix<double, 9, 9> processNoiseMatrix_Q([[maybe_unused]] double dt, [[maybe_unused]] double sigma_a, [[maybe_unused]] double sigma_f, [[maybe_unused]] double sigma_biasw, [[maybe_unused]] double sigma_biasf, [[maybe_unused]] uint8_t M)
{
    Eigen::Matrix<double, 9, 9> nullMatrix{};
    return nullMatrix;
}

Eigen::Matrix<double, 12, 9> designMatrix_H([[maybe_unused]] double omega, [[maybe_unused]] double omegadot, [[maybe_unused]] Eigen::Matrix<double, 12, 12>& R, [[maybe_unused]] Eigen::Matrix<double, 3, 3>& DCM, [[maybe_unused]] uint8_t M)
{
    Eigen::Matrix<double, 12, 9> nullMatrix{};
    return nullMatrix;
}