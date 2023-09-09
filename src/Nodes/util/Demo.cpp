// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Demo.hpp"

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
InsTime getCurrentInsTime()
{
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto* t = std::localtime(&now); // NOLINT(concurrency-mt-unsafe) // FIXME: error: function is not thread safe

    return { static_cast<uint16_t>(t->tm_year + 1900),
             static_cast<uint16_t>(t->tm_mon),
             static_cast<uint16_t>(t->tm_mday),
             static_cast<uint16_t>(t->tm_hour),
             static_cast<uint16_t>(t->tm_min),
             static_cast<long double>(t->tm_sec) };
}

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const Demo::DemoData& data)
{
    j = json{
        { "boolean", data.boolean },
        { "integer", data.integer },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, Demo::DemoData& data)
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

    _onlyRealTime = true; // Add this if you have a sensor, network stream, ...
    _hasConfig = true;
    _lockConfigDuringRun = false;
    _guiConfigDefaultWindowSize = { 630, 410 };

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, { typeStatic() }, this);
    nm::CreateOutputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::ImuObs::type() });
    nm::CreateOutputPin(this, "FileReader\n Data", Pin::Type::Flow, { NAV::NodeData::type() }, &Demo::pollData);
    nm::CreateOutputPin(this, "Bool", Pin::Type::Bool, { "" }, &_valueBool);
    nm::CreateOutputPin(this, "Int", Pin::Type::Int, { "" }, &_valueInt);
    nm::CreateOutputPin(this, "Float", Pin::Type::Float, { "" }, &_valueFloat);
    nm::CreateOutputPin(this, "Double", Pin::Type::Float, { "" }, &_valueDouble);
    nm::CreateOutputPin(this, "String", Pin::Type::String, { "" }, &_valueString);
    nm::CreateOutputPin(this, "Object", Pin::Type::Object, { "Demo::DemoData" }, &_valueObject);
    nm::CreateOutputPin(this, "Matrix", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_valueMatrix);

    nm::CreateInputPin(this, "Demo Node", Pin::Type::Delegate, { typeStatic() });
    nm::CreateInputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::ImuObs::type() }, &Demo::receiveSensorData);
    inputPins.back().neededForTemporalQueueCheck = false;
    nm::CreateInputPin(this, "FileReader\n Data", Pin::Type::Flow, { NAV::NodeData::type() }, &Demo::receiveFileReaderData);
    nm::CreateInputPin(this, "Bool", Pin::Type::Bool);
    nm::CreateInputPin(this, "Int", Pin::Type::Int);
    nm::CreateInputPin(this, "Float", Pin::Type::Float);
    nm::CreateInputPin(this, "Double", Pin::Type::Float);
    nm::CreateInputPin(this, "String", Pin::Type::String, {}, &Demo::stringUpdatedNotifyFunction);
    nm::CreateInputPin(this, "Object", Pin::Type::Object, { "Demo::DemoData" });
    nm::CreateInputPin(this, "Matrix", Pin::Type::Matrix, { "Eigen::MatrixXd" });
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
    if (ImGui::BeginTable("##DemoValues", 2, ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Input");
        ImGui::TableSetupColumn("Output");
        ImGui::TableHeadersRow();

        /* ----------------------------------------------- Delegate ----------------------------------------------- */
        ImGui::TableNextColumn();
        {
            const Demo* connectedNode = getInputValue<const Demo>(INPUT_PORT_INDEX_DEMO_NODE);
            auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_DEMO_NODE);
            if (mutex) { mutex->lock(); }
            ImGui::Text("Delegate: %s", connectedNode ? connectedNode->nameId().c_str() : "N/A");
            if (mutex) { mutex->unlock(); }
        }
        ImGui::TableNextColumn();
        /* ------------------------------------------------ Sensor ------------------------------------------------ */
        ImGui::TableNextColumn();
        ImGui::Text("Sensor Data Count: %d", _receivedDataFromSensorCnt);
        ImGui::TableNextColumn();
        if (ImGui::SliderInt("Frequency", &_outputFrequency, 1, 10))
        {
            int outputInterval = static_cast<int>(1.0 / static_cast<double>(_outputFrequency) * 1000.0);
            _timer.setInterval(outputInterval);
            flow::ApplyChanges();
        }
        /* ---------------------------------------------- FileReader ---------------------------------------------- */
        ImGui::TableNextColumn();
        ImGui::Text("FileReader Data Count: %d", _receivedDataFromFileReaderCnt);
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x / 3);
        if (ImGui::InputInt("FileReader Obs Count", &_nPollData))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------- Bool ------------------------------------------------- */
        ImGui::TableNextColumn();
        {
            const auto* connectedBool = getInputValue<const bool>(INPUT_PORT_INDEX_BOOL);
            auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_BOOL);
            if (mutex) { mutex->lock(); }
            ImGui::Text("Bool: %s", connectedBool ? (*connectedBool ? "true" : "false") : "N/A");
            if (mutex) { mutex->unlock(); }
        }
        ImGui::TableNextColumn();
        if (ImGui::Checkbox("Bool", &_valueBool))
        {
            flow::ApplyChanges();
        }

        /* -------------------------------------------------- Int ------------------------------------------------- */
        ImGui::TableNextColumn();
        if (const auto* connectedInt = getInputValue<const int>(INPUT_PORT_INDEX_INT))
        {
            auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_INT);
            if (mutex) { mutex->lock(); }
            ImGui::Text("Int: %d", *connectedInt);
            if (mutex) { mutex->unlock(); }
        }
        else
        {
            ImGui::TextUnformatted("Int: N/A");
        }
        ImGui::TableNextColumn();
        if (ImGui::InputInt("Int", &_valueInt)) // Returns true if a change was made
        {
            // Limit the values to [-2,5]
            if (_valueInt < -2)
            {
                _valueInt = -2;
            }
            if (_valueInt > 5)
            {
                _valueInt = 5;
            }

            flow::ApplyChanges();
        }
        /* ------------------------------------------------- Float ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (const auto* connectedFloat = getInputValue<const float>(INPUT_PORT_INDEX_FLOAT))
        {
            auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_FLOAT);
            if (mutex) { mutex->lock(); }
            ImGui::Text("Float: %.3f", *connectedFloat);
            if (mutex) { mutex->unlock(); }
        }
        else
        {
            ImGui::TextUnformatted("Float: N/A");
        }
        ImGui::TableNextColumn();
        if (ImGui::DragFloat("Float", &_valueFloat))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------ Double ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (const auto* connectedDouble = getInputValue<const double>(INPUT_PORT_INDEX_DOUBLE))
        {
            auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_DOUBLE);
            if (mutex) { mutex->lock(); }
            ImGui::Text("Double : %.3f", *connectedDouble);
            if (mutex) { mutex->unlock(); }
        }
        else
        {
            ImGui::TextUnformatted("Double: N/A");
        }
        ImGui::TableNextColumn();

        if (ImGui::DragDouble("Double", &_valueDouble))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------ String ------------------------------------------------ */
        ImGui::TableNextColumn();
        ImGui::Text("String: %s", _connectedString.c_str());
        ImGui::Text("The String was updated %lu time%s", _stringUpdateCounter, _stringUpdateCounter > 1 || _stringUpdateCounter == 0 ? "s" : "");
        ImGui::TableNextColumn();
        requestOutputValueLock(OUTPUT_PORT_INDEX_STRING); // Before accessing and changing the value. A lock has to be requested to ensure it is not changed before all linked nodes received the value.
        if (ImGui::InputText("String", &_valueString))
        {
            flow::ApplyChanges();
            const auto& outputPin = outputPins[OUTPUT_PORT_INDEX_STRING];
            if (outputPin.isPinLinked())
            {
                if (!isInitialized()) { LOG_WARN("{}: Notifying connected nodes requires this node to be initialized.", nameId()); }
                else if (!callbacksEnabled) { LOG_WARN("{}: Notifying connected nodes requires enabled callbacks on this node.", nameId()); }
                else if (std::none_of(outputPin.links.begin(), outputPin.links.end(), [](const OutputPin::OutgoingLink& link) { return link.connectedNode->isInitialized(); }))
                {
                    LOG_WARN("{}: Notifying connected nodes requires at least one connected node to be initialized.", nameId());
                }
            }
            notifyOutputValueChanged(OUTPUT_PORT_INDEX_STRING, getCurrentInsTime());
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("The string notifies about changes.\nEnable this nodes callback & initialize both nodes.");
        /* ------------------------------------------------ Object ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (const auto* connectedObject = getInputValue<const DemoData>(INPUT_PORT_INDEX_DEMO_DATA))
        {
            auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_DEMO_DATA);
            if (mutex) { mutex->lock(); }
            ImGui::Text("Object: [%d, %d, %d], %s", connectedObject->integer.at(0), connectedObject->integer.at(1), connectedObject->integer.at(2),
                        connectedObject->boolean ? "true" : "false");
            if (mutex) { mutex->unlock(); }
        }
        else
        {
            ImGui::TextUnformatted("Object: N/A");
        }
        ImGui::TableNextColumn();
        if (ImGui::InputInt3("", _valueObject.integer.data()))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Object", &_valueObject.boolean))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------ Matrix ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (const auto* connectedMatrix = getInputValue<const Eigen::MatrixXd>(INPUT_PORT_INDEX_MATRIX))
        {
            auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_MATRIX);
            if (mutex) { mutex->lock(); }
            gui::widgets::MatrixView("Current Matrix", connectedMatrix, GuiMatrixViewFlags_Header, ImGuiTableFlags_Borders | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit, "%.1f");
            if (mutex) { mutex->unlock(); }
        }
        else
        {
            ImGui::TextUnformatted("Matrix: N/A");
        }
        ImGui::TableNextColumn();

        if (gui::widgets::InputMatrix("Init Matrix", &_valueMatrix, GuiMatrixViewFlags_Header, ImGuiTableFlags_Borders | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedFit, 30.0F, 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::Demo::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["outputFrequency"] = _outputFrequency;
    j["nPollData"] = _nPollData;
    j["valueBool"] = _valueBool;
    j["valueInt"] = _valueInt;
    j["valueFloat"] = _valueFloat;
    j["valueDouble"] = _valueDouble;
    j["valueString"] = _valueString;
    j["valueObject"] = _valueObject;
    j["valueMatrix"] = _valueMatrix;

    return j;
}

void NAV::Demo::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputFrequency"))
    {
        j.at("outputFrequency").get_to(_outputFrequency);
    }
    if (j.contains("nPollData"))
    {
        j.at("nPollData").get_to(_nPollData);
    }
    if (j.contains("valueBool"))
    {
        j.at("valueBool").get_to(_valueBool);
    }
    if (j.contains("valueInt"))
    {
        j.at("valueInt").get_to(_valueInt);
    }
    if (j.contains("valueFloat"))
    {
        j.at("valueFloat").get_to(_valueFloat);
    }
    if (j.contains("valueDouble"))
    {
        j.at("valueDouble").get_to(_valueDouble);
    }
    if (j.contains("valueString"))
    {
        j.at("valueString").get_to(_valueString);
    }
    if (j.contains("valueObject"))
    {
        j.at("valueObject").get_to(_valueObject);
    }
    if (j.contains("valueMatrix"))
    {
        j.at("valueMatrix").get_to(_valueMatrix);
    }
}

bool NAV::Demo::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // To Show the Initialization in the GUI
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    _receivedDataFromSensorCnt = 0;
    _receivedDataFromFileReaderCnt = 0;

    _stringUpdateCounter = 0;

    int outputInterval = static_cast<int>(1.0 / static_cast<double>(_outputFrequency) * 1000.0);
    _timer.start(outputInterval, readSensorDataThread, this);

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

bool NAV::Demo::resetNode()
{
    LOG_TRACE("{}: called", nameId());
    // Here you could reset a FileReader
    _iPollData = 0;

    return true;
}

bool NAV::Demo::onCreateLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));

    return true;
}

void NAV::Demo::onDeleteLink([[maybe_unused]] OutputPin& startPin, [[maybe_unused]] InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));
}

void NAV::Demo::receiveSensorData(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    _receivedDataFromSensorCnt++;
    LOG_DEBUG("{}: received {} Sensor Data", nameId(), _receivedDataFromSensorCnt);

    queue.pop_front(); // Here we did not read the data, so lets just pop it from the queue
}

void NAV::Demo::receiveFileReaderData(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Here we are reading the data, so extracts gets the value and automatically pops it from the queue
    auto obs = queue.extract_front();
    _receivedDataFromFileReaderCnt++;

    LOG_DEBUG("{}: received {} FileReader Data: {}", nameId(), _receivedDataFromFileReaderCnt, obs->insTime.toYMDHMS());
}

void NAV::Demo::readSensorDataThread(void* userData)
{
    auto* node = static_cast<Demo*>(userData);

    if (node->getMode() == Mode::POST_PROCESSING) { return; }

    auto imuPos = ImuPos();
    auto obs = std::make_shared<ImuObs>(imuPos);

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto* t = std::localtime(&now); // NOLINT(concurrency-mt-unsafe) // FIXME: error: function is not thread safe

    obs->insTime = InsTime(static_cast<uint16_t>(t->tm_year + 1900),
                           static_cast<uint16_t>(t->tm_mon),
                           static_cast<uint16_t>(t->tm_mday),
                           static_cast<uint16_t>(t->tm_hour),
                           static_cast<uint16_t>(t->tm_min),
                           static_cast<long double>(t->tm_sec));

    std::random_device rd;
    std::default_random_engine generator(rd());

    std::uniform_real_distribution<double> distribution(-9.0, 9.0);
    obs->accelUncompXYZ = Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));

    distribution = std::uniform_real_distribution<double>(-3.0, 3.0);
    obs->gyroUncompXYZ = Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));

    distribution = std::uniform_real_distribution<double>(-1.0, 1.0);
    obs->magUncompXYZ = Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));

    distribution = std::uniform_real_distribution<double>(15.0, 25.0);
    obs->temperature = distribution(generator);

    node->invokeCallbacks(OUTPUT_PORT_INDEX_FLOW_SENSOR, obs);
}

std::shared_ptr<const NAV::NodeData> NAV::Demo::peekPollData(bool peek)
{
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
    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW_FILE, obs);

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

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW_FILE, obs); // Calls all the callbacks
    return obs;
}

void NAV::Demo::stringUpdatedNotifyFunction([[maybe_unused]] const InsTime& insTime, size_t pinIdx) // TODO: This does not work
{
    _stringUpdateCounter++;

    _connectedString = *getInputValue<const std::string>(pinIdx);
    LOG_DEBUG("String value updated to {} at time {}", _connectedString, insTime);

    releaseInputValue(pinIdx); // Do not forget to release the mutex after you are done reading it
}