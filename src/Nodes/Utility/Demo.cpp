// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Demo.hpp"
#include <cstddef>
#include <imgui.h>
#include <optional>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/Matrix.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include <chrono>
#include <thread>
#include <random>

namespace NAV
{
namespace
{
InsTime getCurrentInsTime()
{
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto* t = std::localtime(&now); // NOLINT(concurrency-mt-unsafe)

    return { static_cast<uint16_t>(t->tm_year + 1900),
             static_cast<uint16_t>(t->tm_mon),
             static_cast<uint16_t>(t->tm_mday),
             static_cast<uint16_t>(t->tm_hour),
             static_cast<uint16_t>(t->tm_min),
             static_cast<long double>(t->tm_sec) };
}
} // namespace

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
static void to_json(json& j, const Demo::DemoData& data) // NOLINT(misc-use-anonymous-namespace)
{
    j = json{
        { "boolean", data.boolean },
        { "integer", data.integer },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
static void from_json(const json& j, Demo::DemoData& data) // NOLINT(misc-use-anonymous-namespace)
{
    if (j.contains("boolean"))
    {
        j.at("boolean").get_to(data.boolean);
    }
    if (j.contains("integer"))
    {
        j.at("integer").get_to(data.integer);
    }
}

} // namespace NAV

NAV::Demo::Demo()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _onlyRealTime = false; // Set this to true if you have a sensor, network stream, ...
    _hasConfig = true;
    _lockConfigDuringRun = false;
    _guiConfigDefaultWindowSize = { 630, 410 };

    // Pins are usually created by calling the following functions in the constructor
    nm::CreateInputPin(this, "Flow", Pin::Type::Flow, { NAV::NodeData::type() }, &Demo::receiveData);
    nm::CreateOutputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::ImuObs::type() });

    // To create or delete pins depending on GUI options we use a function as it needs to be called from multiple places
    updatePins();
}

NAV::Demo::~Demo()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Demo::typeStatic()
{
    return "Demo";
}

std::string NAV::Demo::type() const
{
    return typeStatic();
}

std::string NAV::Demo::category()
{
    return "Utility";
}

void NAV::Demo::guiConfig()
{
    if (ImGui::BeginTable("##DemoValues", 3, ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Enable");
        ImGui::TableSetupColumn("Input");
        ImGui::TableSetupColumn("Output");
        ImGui::TableHeadersRow();

        /* ----------------------------------------------- Delegate ----------------------------------------------- */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Delegate##enable {}", size_t(id)).c_str(), &_enableDelegate))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableDelegate)
        {
            ImGui::TableSetColumnIndex(1);
            {
                // The returned type automatically blocks editing on the other side of the link. Like a scoped_lock for mutexes
                auto connectedNode = getInputValue<Demo>(getPinIdx(DemoPins::Delegate).value());
                ImGui::Text("Delegate: %s", connectedNode ? connectedNode->v->nameId().c_str() : "N/A");
            }
        }
        /* ------------------------------------------------ Flow ------------------------------------------------ */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Flow##enable {}", size_t(id)).c_str(), &_enableFlow))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableFlow)
        {
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("Flow Data Count: %d", _receivedDataCnt);
            ImGui::TableSetColumnIndex(2);
            if (ImGui::Checkbox(fmt::format("Simulate File Reader##{}", size_t(id)).c_str(), &_fileReaderInsteadSensor))
            {
                if (_fileReaderInsteadSensor)
                {
                    if (_timer.is_running()) { _timer.stop(); }
                }
                else
                {
                    if (isInitialized() && !_timer.is_running())
                    {
                        int outputInterval = static_cast<int>(1.0 / static_cast<double>(_outputFrequency) * 1000.0);
                        _timer.start(outputInterval, readSensorDataThread, this);
                    }
                }
                updateOutputFlowPin();
                flow::ApplyChanges();
            }
            if (_fileReaderInsteadSensor)
            {
                ImGui::SetNextItemWidth(100.0F);
                if (ImGui::InputInt(fmt::format("FileReader Obs Count##{}", size_t(id)).c_str(), &_nPollData))
                {
                    flow::ApplyChanges();
                }
            }
            else
            {
                if (ImGui::SliderInt(fmt::format("Frequency##{}", size_t(id)).c_str(), &_outputFrequency, 1, 10))
                {
                    int outputInterval = static_cast<int>(1.0 / static_cast<double>(_outputFrequency) * 1000.0);
                    _timer.setInterval(outputInterval);
                    flow::ApplyChanges();
                }
            }
        }
        /* ------------------------------------------------- Bool ------------------------------------------------- */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Bool##enable {}", size_t(id)).c_str(), &_enableBool))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableBool)
        {
            ImGui::TableSetColumnIndex(1);
            {
                auto connectedBool = getInputValue<bool>(getPinIdx(DemoPins::Bool).value());
                ImGui::Text("Bool: %s", connectedBool ? (connectedBool->v ? "true" : "false") : "N/A");
            }

            ImGui::TableSetColumnIndex(2);
            {
                auto guard = requestOutputValueLock(getPinIdx(DemoPins::Bool).value());
                if (ImGui::Checkbox(fmt::format("Bool##{}", size_t(id)).c_str(), &_valueBool))
                {
                    flow::ApplyChanges();
                }
            }
        }
        /* -------------------------------------------------- Int ------------------------------------------------- */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Int##enable {}", size_t(id)).c_str(), &_enableInt))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableInt)
        {
            ImGui::TableSetColumnIndex(1);
            if (auto connectedInt = getInputValue<int>(getPinIdx(DemoPins::Int).value()))
            {
                ImGui::Text("Int: %d", *connectedInt->v);
            }
            else
            {
                ImGui::TextUnformatted("Int: N/A");
            }

            ImGui::TableSetColumnIndex(2);
            {
                auto guard = requestOutputValueLock(getPinIdx(DemoPins::Int).value());
                if (ImGui::InputInt(fmt::format("Int##{}", size_t(id)).c_str(), &_valueInt)) // Returns true if a change was made
                {
                    // Limit the values to [-2,5]
                    _valueInt = std::max(_valueInt, -2);
                    _valueInt = std::min(_valueInt, 5);

                    flow::ApplyChanges();
                }
            }
        }
        /* ------------------------------------------------- Float ------------------------------------------------ */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Float##enable {}", size_t(id)).c_str(), &_enableFloat))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableFloat)
        {
            ImGui::TableSetColumnIndex(1);
            if (auto connectedFloat = getInputValue<float>(getPinIdx(DemoPins::Float).value()))
            {
                ImGui::Text("Float: %.3f", *connectedFloat->v);
            }
            else
            {
                ImGui::TextUnformatted("Float: N/A");
            }

            ImGui::TableSetColumnIndex(2);
            {
                auto guard = requestOutputValueLock(getPinIdx(DemoPins::Float).value());
                if (ImGui::DragFloat(fmt::format("Float##{}", size_t(id)).c_str(), &_valueFloat))
                {
                    flow::ApplyChanges();
                }
            }
        }
        /* ------------------------------------------------ Double ------------------------------------------------ */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Double##enable {}", size_t(id)).c_str(), &_enableDouble))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableDouble)
        {
            ImGui::TableSetColumnIndex(1);
            if (auto connectedDouble = getInputValue<double>(getPinIdx(DemoPins::Double).value()))
            {
                ImGui::Text("Double : %.3f", *connectedDouble->v);
            }
            else
            {
                ImGui::TextUnformatted("Double: N/A");
            }

            ImGui::TableSetColumnIndex(2);
            {
                auto guard = requestOutputValueLock(getPinIdx(DemoPins::Double).value());
                if (ImGui::DragDouble(fmt::format("Double##{}", size_t(id)).c_str(), &_valueDouble))
                {
                    flow::ApplyChanges();
                }
            }
        }
        /* ------------------------------------------------ String ------------------------------------------------ */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("String##enable {}", size_t(id)).c_str(), &_enableString))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableString)
        {
            ImGui::TableSetColumnIndex(1);
            if (auto connectedString = getInputValue<std::string>(getPinIdx(DemoPins::String).value()))
            {
                ImGui::Text("String: %s", connectedString->v->c_str());
            }
            else
            {
                ImGui::TextUnformatted("String: N/A");
            }
            ImGui::Text("The String was updated %lu time%s", _stringUpdateCounter, _stringUpdateCounter > 1 || _stringUpdateCounter == 0 ? "s" : "");

            ImGui::TableSetColumnIndex(2);
            {
                // Before accessing and changing the value. A lock has to be requested to ensure it is not changed before all linked nodes received the value.
                auto guard = requestOutputValueLock(getPinIdx(DemoPins::String).value());
                if (ImGui::InputText(fmt::format("String##{}", size_t(id)).c_str(), &_valueString))
                {
                    flow::ApplyChanges();
                    const auto& outputPin = outputPins[getPinIdx(DemoPins::String).value()];
                    if (outputPin.isPinLinked())
                    {
                        if (!isInitialized()) { LOG_WARN("{}: Notifying connected nodes requires this node to be initialized.", nameId()); }
                        else if (!callbacksEnabled) { LOG_WARN("{}: Notifying connected nodes requires enabled callbacks on this node. Do this by running the flow.", nameId()); }
                        else if (std::ranges::none_of(outputPin.links, [](const OutputPin::OutgoingLink& link) { return link.connectedNode->isInitialized(); }))
                        {
                            LOG_WARN("{}: Notifying connected nodes requires at least one connected node to be initialized.", nameId());
                        }
                    }
                    notifyOutputValueChanged(getPinIdx(DemoPins::String).value(), getCurrentInsTime(), std::move(guard));
                }
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("The string notifies about changes.\nInitialize both nodes for this to work.");
        }
        /* ------------------------------------------------ Object ------------------------------------------------ */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Object##enable {}", size_t(id)).c_str(), &_enableObject))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableObject)
        {
            ImGui::TableSetColumnIndex(1);
            if (auto connectedObject = getInputValue<DemoData>(getPinIdx(DemoPins::Object).value()))
            {
                ImGui::Text("Object: [%d, %d, %d], %s", connectedObject->v->integer.at(0), connectedObject->v->integer.at(1), connectedObject->v->integer.at(2),
                            connectedObject->v->boolean ? "true" : "false");
            }
            else
            {
                ImGui::TextUnformatted("Object: N/A");
            }

            ImGui::TableSetColumnIndex(2);
            {
                auto guard = requestOutputValueLock(getPinIdx(DemoPins::Object).value());
                if (ImGui::InputInt3(fmt::format("##object.integer {}", size_t(id)).c_str(), _valueObject.integer.data()))
                {
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                if (ImGui::Checkbox(fmt::format("Object##{}", size_t(id)).c_str(), &_valueObject.boolean))
                {
                    flow::ApplyChanges();
                }
            }
        }
        /* ------------------------------------------------ Matrix ------------------------------------------------ */
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        if (ImGui::Checkbox(fmt::format("Matrix##enable {}", size_t(id)).c_str(), &_enableMatrix))
        {
            updatePins();
            flow::ApplyChanges();
        }
        if (_enableMatrix)
        {
            ImGui::TableSetColumnIndex(1);
            if (auto connectedMatrix = getInputValue<Eigen::MatrixXd>(getPinIdx(DemoPins::Matrix).value()))
            {
                gui::widgets::MatrixView("Current Matrix", connectedMatrix->v, GuiMatrixViewFlags_Header, ImGuiTableFlags_Borders | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit, "%.1f");
            }
            else
            {
                ImGui::TextUnformatted("Matrix: N/A");
            }

            ImGui::TableSetColumnIndex(2);
            {
                auto guard = requestOutputValueLock(getPinIdx(DemoPins::Matrix).value());
                if (gui::widgets::InputMatrix(fmt::format("Init Matrix##{}", size_t(id)).c_str(), &_valueMatrix, GuiMatrixViewFlags_Header, ImGuiTableFlags_Borders | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit, 30.0F, 0.0, 0.0, "%.1f"))
                {
                    flow::ApplyChanges();
                }
            }
        }

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::Demo::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    return {
        { "outputFrequency", _outputFrequency },
        { "nPollData", _nPollData },
        { "enableDelegate", _enableDelegate },
        { "enableFlow", _enableFlow },
        { "enableBool", _enableBool },
        { "enableInt", _enableInt },
        { "enableFloat", _enableFloat },
        { "enableDouble", _enableDouble },
        { "enableString", _enableString },
        { "enableObject", _enableObject },
        { "enableMatrix", _enableMatrix },
        { "valueBool", _valueBool },
        { "valueInt", _valueInt },
        { "valueFloat", _valueFloat },
        { "valueDouble", _valueDouble },
        { "valueString", _valueString },
        { "valueObject", _valueObject },
        { "valueMatrix", _valueMatrix },
        { "fileReaderInsteadSensor", _fileReaderInsteadSensor },
    };
}

void NAV::Demo::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputFrequency")) { j.at("outputFrequency").get_to(_outputFrequency); }
    if (j.contains("nPollData")) { j.at("nPollData").get_to(_nPollData); }
    if (j.contains("enableDelegate")) { j.at("enableDelegate").get_to(_enableDelegate); }
    if (j.contains("enableFlow")) { j.at("enableFlow").get_to(_enableFlow); }
    if (j.contains("enableBool")) { j.at("enableBool").get_to(_enableBool); }
    if (j.contains("enableInt")) { j.at("enableInt").get_to(_enableInt); }
    if (j.contains("enableFloat")) { j.at("enableFloat").get_to(_enableFloat); }
    if (j.contains("enableDouble")) { j.at("enableDouble").get_to(_enableDouble); }
    if (j.contains("enableString")) { j.at("enableString").get_to(_enableString); }
    if (j.contains("enableObject")) { j.at("enableObject").get_to(_enableObject); }
    if (j.contains("enableMatrix")) { j.at("enableMatrix").get_to(_enableMatrix); }
    if (j.contains("valueBool")) { j.at("valueBool").get_to(_valueBool); }
    if (j.contains("valueInt")) { j.at("valueInt").get_to(_valueInt); }
    if (j.contains("valueFloat")) { j.at("valueFloat").get_to(_valueFloat); }
    if (j.contains("valueDouble")) { j.at("valueDouble").get_to(_valueDouble); }
    if (j.contains("valueString")) { j.at("valueString").get_to(_valueString); }
    if (j.contains("valueObject")) { j.at("valueObject").get_to(_valueObject); }
    if (j.contains("valueMatrix")) { j.at("valueMatrix").get_to(_valueMatrix); }
    if (j.contains("fileReaderInsteadSensor"))
    {
        j.at("fileReaderInsteadSensor").get_to(_fileReaderInsteadSensor);
        updateOutputFlowPin();
    }
    updatePins();
}

bool NAV::Demo::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // To Show the Initialization in the GUI
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    _receivedDataCnt = 0;

    _stringUpdateCounter = 0;

    if (_enableFlow && !_fileReaderInsteadSensor)
    {
        // The Timer is used to simulate a sensor reader
        int outputInterval = static_cast<int>(1.0 / static_cast<double>(_outputFrequency) * 1000.0);
        _timer.start(outputInterval, readSensorDataThread, this);
    }

    return true;
}

void NAV::Demo::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (_timer.is_running())
    {
        _timer.stop();
    }

    // To Show the Deinitialization in the GUI
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void NAV::Demo::updatePins()
{
    size_t pinIdx = 0;

    auto updatePin = [&](bool pinExists, bool enabled,
                         const char* pinName, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {}, // BUG: Bug in clang-format, so we need to disable formatting
                         OutputPin::PinData data = static_cast<void*>(nullptr), // clang-format off
                         void(NAV::Demo::*notifyFunc)(const InsTime&, size_t) = nullptr) {                                 // clang-format on
        if (!pinExists && enabled)
        {
            nm::CreateInputPin(this, pinName, pinType, dataIdentifier, notifyFunc, static_cast<int>(pinIdx));
            nm::CreateOutputPin(this, pinName, pinType, dataIdentifier, data, static_cast<int>(pinIdx));
        }
        else if (pinExists && !enabled)
        {
            nm::DeleteInputPin(inputPins.at(pinIdx));
            nm::DeleteOutputPin(outputPins.at(pinIdx));
        }
        if (enabled) { pinIdx++; }
    };

    updatePin(getPinIdx(DemoPins::Delegate) && inputPins.at(*getPinIdx(DemoPins::Delegate)).type == Pin::Type::Delegate, _enableDelegate,
              "Demo Node", Pin::Type::Delegate, { typeStatic() }, this);
    // nm::CreateInputPin(this, "Demo Node", Pin::Type::Delegate, { typeStatic() });
    // nm::CreateOutputPin(this, "", Pin::Type::Delegate, { typeStatic() }, this, 0);

    {
        bool pinExists = getPinIdx(DemoPins::Flow) && inputPins.at(*getPinIdx(DemoPins::Flow)).type == Pin::Type::Flow;
        if (!pinExists && _enableFlow)
        {
            nm::CreateInputPin(this, "Flow", Pin::Type::Flow, { NAV::NodeData::type() }, &Demo::receiveData);
            nm::CreateOutputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::ImuObs::type() });
        }
        else if (pinExists && !_enableFlow)
        {
            nm::DeleteInputPin(inputPins.at(pinIdx));
            nm::DeleteOutputPin(outputPins.at(pinIdx));
        }
        if (_enableFlow) { pinIdx++; }
    }

    updatePin(getPinIdx(DemoPins::Bool) && inputPins.at(*getPinIdx(DemoPins::Bool)).type == Pin::Type::Bool, _enableBool,
              "Bool", Pin::Type::Bool, { "" }, &_valueBool);
    // nm::CreateInputPin(this, "Bool", Pin::Type::Bool);
    // nm::CreateOutputPin(this, "Bool", Pin::Type::Bool, { "" }, &_valueBool);

    updatePin(getPinIdx(DemoPins::Int) && inputPins.at(*getPinIdx(DemoPins::Int)).type == Pin::Type::Int, _enableInt,
              "Int", Pin::Type::Int, { "" }, &_valueInt);
    // nm::CreateInputPin(this, "Int", Pin::Type::Int);
    // nm::CreateOutputPin(this, "Int", Pin::Type::Int, { "" }, &_valueInt);

    updatePin(getPinIdx(DemoPins::Float) && inputPins.at(*getPinIdx(DemoPins::Float)).type == Pin::Type::Float, _enableFloat,
              "Float", Pin::Type::Float, { "Float" }, &_valueFloat);
    // nm::CreateInputPin(this, "Float", Pin::Type::Float);
    // nm::CreateOutputPin(this, "Float", Pin::Type::Float, { "" }, &_valueFloat);

    updatePin(getPinIdx(DemoPins::Double) && inputPins.at(*getPinIdx(DemoPins::Double)).type == Pin::Type::Float, _enableDouble,
              "Double", Pin::Type::Float, { "Double" }, &_valueDouble);
    // nm::CreateInputPin(this, "Double", Pin::Type::Float);
    // nm::CreateOutputPin(this, "Double", Pin::Type::Float, { "" }, &_valueDouble);

    updatePin(getPinIdx(DemoPins::String) && inputPins.at(*getPinIdx(DemoPins::String)).type == Pin::Type::String, _enableString,
              "String", Pin::Type::String, { "" }, &_valueString, &Demo::stringUpdatedNotifyFunction);
    // nm::CreateInputPin(this, "String", Pin::Type::String, {}, &Demo::stringUpdatedNotifyFunction);
    // nm::CreateOutputPin(this, "String", Pin::Type::String, { "" }, &_valueString);

    updatePin(getPinIdx(DemoPins::Object) && inputPins.at(*getPinIdx(DemoPins::Object)).type == Pin::Type::Object, _enableObject,
              "Object", Pin::Type::Object, { "Demo::DemoData" }, &_valueObject);
    // nm::CreateInputPin(this, "Object", Pin::Type::Object, { "Demo::DemoData" });
    // nm::CreateOutputPin(this, "Object", Pin::Type::Object, { "Demo::DemoData" }, &_valueObject);

    updatePin(getPinIdx(DemoPins::Matrix) && inputPins.at(*getPinIdx(DemoPins::Matrix)).type == Pin::Type::Matrix, _enableMatrix,
              "Matrix", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_valueMatrix);
    // nm::CreateInputPin(this, "Matrix", Pin::Type::Matrix, { "Eigen::MatrixXd" });
    // nm::CreateOutputPin(this, "Matrix", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_valueMatrix);
}

std::optional<size_t> NAV::Demo::getPinIdx(DemoPins pinType) const
{
    for (size_t i = 0; i < inputPins.size(); i++)
    {
        switch (pinType)
        {
        case DemoPins::Delegate:
            if (inputPins.at(i).type == Pin::Type::Delegate) { return i; }
            break;
        case DemoPins::Flow:
            if (inputPins.at(i).type == Pin::Type::Flow) { return i; }
            break;
        case DemoPins::Bool:
            if (inputPins.at(i).type == Pin::Type::Bool) { return i; }
            break;
        case DemoPins::Int:
            if (inputPins.at(i).type == Pin::Type::Int) { return i; }
            break;
        case DemoPins::Float:
            if (inputPins.at(i).type == Pin::Type::Float && inputPins.at(i).dataIdentifier.front() == "Float") { return i; }
            break;
        case DemoPins::Double:
            if (inputPins.at(i).type == Pin::Type::Float && inputPins.at(i).dataIdentifier.front() == "Double") { return i; }
            break;
        case DemoPins::String:
            if (inputPins.at(i).type == Pin::Type::String) { return i; }
            break;
        case DemoPins::Object:
            if (inputPins.at(i).type == Pin::Type::Object) { return i; }
            break;
        case DemoPins::Matrix:
            if (inputPins.at(i).type == Pin::Type::Matrix) { return i; }
            break;
        }
    }

    return std::nullopt;
}

bool NAV::Demo::resetNode()
{
    LOG_TRACE("{}: called", nameId());
    // Here you could reset a FileReader
    _iPollData = 0;
    _receivedDataCnt = 0;

    return true;
}

void NAV::Demo::updateOutputFlowPin()
{
    size_t flowPinIdx = getPinIdx(DemoPins::Flow).value();
    std::vector<ax::NodeEditor::PinId> connectedPins;
    for (const auto& link : outputPins.at(flowPinIdx).links)
    {
        connectedPins.push_back(link.connectedPinId);
    }
    nm::DeleteOutputPin(outputPins.at(flowPinIdx));
    if (_fileReaderInsteadSensor)
    {
        nm::CreateOutputPin(this, "FileReader\n Data", Pin::Type::Flow, { NAV::NodeData::type() }, &Demo::pollData, static_cast<int>(flowPinIdx));
    }
    else
    {
        nm::CreateOutputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::ImuObs::type() }, static_cast<void*>(nullptr), static_cast<int>(flowPinIdx));
    }
    for (const auto& pinId : connectedPins)
    {
        if (auto* targetPin = nm::FindInputPin(pinId))
        {
            if (outputPins.at(flowPinIdx).canCreateLink(*targetPin))
            {
                outputPins.at(flowPinIdx).createLink(*targetPin);
            }
        }
    }
}

bool NAV::Demo::onCreateLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (_enableFlow && outputPins.at(getPinIdx(DemoPins::Flow).value()).isPinLinked())
    {
        _onlyRealTime = !_fileReaderInsteadSensor;
    }

    return true;
}

void NAV::Demo::afterDeleteLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    if (_enableFlow && !outputPins.at(getPinIdx(DemoPins::Flow).value()).isPinLinked())
    {
        _onlyRealTime = false;
    }
}

void NAV::Demo::receiveData(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    std::shared_ptr<const NAV::NodeData> obs = queue.extract_front(); // Either 'extract_front()' or 'pop_front()' needs to be called
    _receivedDataCnt++;

    LOG_DEBUG("{}: received {} data at [{} GPST]", nameId(), _receivedDataCnt, obs->insTime.toYMDHMS(GPST));
}

void NAV::Demo::readSensorDataThread(void* userData)
{
    auto* node = static_cast<Demo*>(userData);

    if (!node->_enableFlow || !node->getPinIdx(DemoPins::Flow).has_value()) { return; }

    if (!node->outputPins.at(node->getPinIdx(DemoPins::Flow).value()).isPinLinked() || !node->callbacksEnabled)
    {
        return;
    }

    if (node->getMode() == Mode::POST_PROCESSING)
    {
        LOG_WARN("{}: Flow contains nodes which can only do post-processing. Sensor output is suppressed.");

        return;
    }

    auto imuPos = ImuPos();
    auto obs = std::make_shared<ImuObs>(imuPos);

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto* t = std::gmtime(&now); // NOLINT(concurrency-mt-unsafe)

    obs->insTime = InsTime(static_cast<uint16_t>(t->tm_year + 1900), static_cast<uint16_t>(t->tm_mon) + 1, static_cast<uint16_t>(t->tm_mday),
                           static_cast<uint16_t>(t->tm_hour), static_cast<uint16_t>(t->tm_min), static_cast<long double>(t->tm_sec), GPST);

    std::random_device rd;
    std::default_random_engine generator(rd());

    std::uniform_real_distribution<double> distribution(-9.0, 9.0);
    obs->p_acceleration = Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));

    distribution = std::uniform_real_distribution<double>(-3.0, 3.0);
    obs->p_angularRate = Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));

    distribution = std::uniform_real_distribution<double>(-1.0, 1.0);
    obs->p_magneticField = Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));

    distribution = std::uniform_real_distribution<double>(15.0, 25.0);
    obs->temperature = distribution(generator);

    LOG_INFO("{}: Sending Sensor data with time [{} GPST]", node->nameId(), obs->insTime.toYMDHMS(GPST));

    node->invokeCallbacks(node->getPinIdx(DemoPins::Flow).value(), obs);
}

std::shared_ptr<const NAV::NodeData> NAV::Demo::peekPollData(bool peek)
{
    // This function is only an example of how to implement peek/poll logic. It is not used in this node.
    if (_iPollData >= _nPollData)
    {
        return nullptr;
    }

    if (peek) // Early return with time to let the Node sort the observations
    {
        auto obs = std::make_shared<NodeData>(); // Construct the real observation (here in example also from type NodeData)
        obs->insTime = InsTime(2000, 1, 1, 0, 0, _iPollData);
        return obs;
    }

    auto obs = std::make_shared<NodeData>(); // Construct the real observation (here in example also from type NodeData)
    obs->insTime = InsTime(2000, 1, 1, 0, 0, _iPollData);

    _iPollData++;
    // Calls all the callbacks
    invokeCallbacks(getPinIdx(DemoPins::Flow).value(), obs);

    return obs;
}

std::shared_ptr<const NAV::NodeData> NAV::Demo::pollData()
{
    if (_iPollData >= _nPollData)
    {
        return nullptr; // Tells the node that the last message was read
    }

    auto obs = std::make_shared<NodeData>(); // Construct the real observation (here in example also from type NodeData)
    obs->insTime = InsTime(2000, 1, 1, 0, 0, _iPollData);

    _iPollData++;

    invokeCallbacks(getPinIdx(DemoPins::Flow).value(), obs); // Calls all the callbacks
    return obs;
}

void NAV::Demo::stringUpdatedNotifyFunction([[maybe_unused]] const InsTime& insTime, size_t pinIdx)
{
    _stringUpdateCounter++;

    if (auto value = getInputValue<std::string>(pinIdx))
    {
        LOG_DEBUG("String value updated to '{}' at time {}", *value->v, insTime);
    }
}