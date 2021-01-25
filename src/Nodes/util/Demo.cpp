#include "Demo.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/InsObs.hpp"

#include <chrono>
#include <thread>

namespace NAV
{
void to_json(json& j, const Demo::DemoData& data)
{
    j = json{
        { "boolean", data.boolean },
        { "integer", data.integer },
    };
}
void from_json(const json& j, Demo::DemoData& data)
{
    j.at("boolean").get_to(data.boolean);
    j.at("integer").get_to(data.integer);
}

} // namespace NAV

NAV::Demo::Demo()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "Sensor\nData", Pin::Type::Flow, NAV::NodeData::type());
    nm::CreateOutputPin(this, "FileReader\n Data", Pin::Type::Flow, NAV::InsObs::type(), &Demo::pollData);
    nm::CreateOutputPin(this, "Bool", Pin::Type::Bool, "", &valueBool);
    nm::CreateOutputPin(this, "Int", Pin::Type::Int, "", &valueInt);
    nm::CreateOutputPin(this, "Float", Pin::Type::Float, "", &valueFloat);
    nm::CreateOutputPin(this, "Double", Pin::Type::Float, "", &valueDouble);
    nm::CreateOutputPin(this, "String", Pin::Type::String, "", &valueString);
    nm::CreateOutputPin(this, "Object", Pin::Type::Object, "Demo::DemoData", &valueObject);
    nm::CreateOutputPin(this, "Matrix", Pin::Type::Matrix, "Eigen::MatrixXd", &valueMatrix);
    nm::CreateOutputPin(this, "Function", Pin::Type::Function, "std::string (*)(int, bool)", &Demo::callbackFunction);

    nm::CreateInputPin(this, "Demo Node", Pin::Type::Delegate, { typeStatic() });
    nm::CreateInputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::NodeData::type() }, &Demo::receiveSensorData);
    nm::CreateInputPin(this, "FileReader\n Data", Pin::Type::Flow, { NAV::InsObs::type() }, &Demo::receiveFileReaderData);
    nm::CreateInputPin(this, "Bool", Pin::Type::Bool);
    nm::CreateInputPin(this, "Int", Pin::Type::Int);
    nm::CreateInputPin(this, "Float", Pin::Type::Float);
    nm::CreateInputPin(this, "Double", Pin::Type::Float);
    nm::CreateInputPin(this, "String", Pin::Type::String);
    nm::CreateInputPin(this, "Object", Pin::Type::Object, { "Demo::DemoData" });
    nm::CreateInputPin(this, "Matrix", Pin::Type::Matrix, { "Eigen::MatrixXd" });
    nm::CreateInputPin(this, "Function", Pin::Type::Function, { "std::string (*)(int, bool)" });
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
    return "Demo";
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
        auto* connectedNode = getInputValue<Demo>(InputPortIndex_DemoNode);
        ImGui::Text("Delegate: %s", connectedNode ? connectedNode->nameId().c_str() : "N/A");
        ImGui::TableNextColumn();
        /* ------------------------------------------------ Sensor ------------------------------------------------ */
        ImGui::TableNextColumn();
        ImGui::Text("Sensor Data Count: %d", receivedDataFromSensorCnt);
        ImGui::TableNextColumn();
        if (ImGui::SliderInt("Frequency", &outputFrequency, 1, 10))
        {
            int outputInterval = static_cast<int>(1.0 / static_cast<double>(outputFrequency) * 1000.0);
            timer.setInterval(outputInterval);
            flow::ApplyChanges();
        }
        /* ---------------------------------------------- FileReader ---------------------------------------------- */
        ImGui::TableNextColumn();
        ImGui::Text("FileReader Data Count: %d", receivedDataFromFileReaderCnt);
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x / 3);
        if (ImGui::InputInt("FileReader Obs Count", &nPollData))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------- Bool ------------------------------------------------- */
        ImGui::TableNextColumn();
        auto* connectedBool = getInputValue<bool>(InputPortIndex_Bool);
        ImGui::Text("Bool: %s", connectedBool ? (*connectedBool ? "true" : "false") : "N/A");
        ImGui::TableNextColumn();
        if (ImGui::Checkbox("Bool", &valueBool))
        {
            flow::ApplyChanges();
        }
        /* -------------------------------------------------- Int ------------------------------------------------- */
        ImGui::TableNextColumn();
        if (auto* connectedInt = getInputValue<int>(InputPortIndex_Int))
        {
            ImGui::Text("Int: %d", *connectedInt);
        }
        else
        {
            ImGui::TextUnformatted("Int: N/A");
        }
        ImGui::TableNextColumn();
        if (ImGui::InputInt("Int", &valueInt)) // Returns true if a change was made
        {
            // Limit the values to [-2,5]
            if (valueInt < -2)
            {
                valueInt = -2;
            }
            if (valueInt > 5)
            {
                valueInt = 5;
            }

            flow::ApplyChanges();
        }
        /* ------------------------------------------------- Float ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (auto* connectedFloat = getInputValue<float>(InputPortIndex_Float))
        {
            ImGui::Text("Float: %.3f", *connectedFloat);
        }
        else
        {
            ImGui::TextUnformatted("Float: N/A");
        }
        ImGui::TableNextColumn();
        if (ImGui::DragFloat("Float", &valueFloat))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------ Double ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (auto* connectedDouble = getInputValue<double>(InputPortIndex_Double))
        {
            ImGui::Text("Double: %.6f", *connectedDouble);
        }
        else
        {
            ImGui::TextUnformatted("Double: N/A");
        }
        ImGui::TableNextColumn();
        if (ImGui::InputDouble("Double", &valueDouble))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------ String ------------------------------------------------ */
        ImGui::TableNextColumn();
        auto* connectedString = getInputValue<std::string>(InputPortIndex_String);
        ImGui::Text("String: %s", connectedString ? connectedString->c_str() : "N/A");
        ImGui::TableNextColumn();
        if (ImGui::InputText("String", &valueString))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------ Object ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (auto* connectedObject = getInputValue<DemoData>(InputPortIndex_DemoData))
        {
            ImGui::Text("Object: [%d, %d, %d], %s", connectedObject->integer.at(0), connectedObject->integer.at(1), connectedObject->integer.at(2),
                        connectedObject->boolean ? "true" : "false");
        }
        else
        {
            ImGui::TextUnformatted("Object: N/A");
        }
        ImGui::TableNextColumn();
        if (ImGui::InputInt3("", valueObject.integer.data()))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Object", &valueObject.boolean))
        {
            flow::ApplyChanges();
        }
        /* ------------------------------------------------ Matrix ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (auto* connectedMatrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Matrix))
        {
            if (connectedMatrix->rows() == 3 && connectedMatrix->cols() == 3)
            {
                ImGui::Text("Matrix: [%.1f, %.1f, %.1f]\n"
                            "               [%.1f, %.1f, %.1f]\n"
                            "               [%.1f, %.1f, %.1f]",
                            (*connectedMatrix)(0, 0), (*connectedMatrix)(0, 1), (*connectedMatrix)(0, 2),
                            (*connectedMatrix)(1, 0), (*connectedMatrix)(1, 1), (*connectedMatrix)(1, 2),
                            (*connectedMatrix)(2, 0), (*connectedMatrix)(2, 1), (*connectedMatrix)(2, 2));
            }
            else
            {
                ImGui::TextUnformatted("Matrix: Not 3x3");
            }
        }
        else
        {
            ImGui::TextUnformatted("Matrix: N/A");
        }
        ImGui::TableNextColumn();
        float itemWidth = ImGui::GetContentRegionAvail().x / 3.0F - 2 * ImGui::GetStyle().ItemInnerSpacing.x;
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##0,0", &valueMatrix(0, 0), 0.0, 0.0, "%.1f")) // We don't want a label,
        {                                                                      // but the label has to be an unique identifier.
            flow::ApplyChanges();                                              // So use ##, which hides everything after it
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##0,1", &valueMatrix(0, 1), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##0,2", &valueMatrix(0, 2), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##1,0", &valueMatrix(1, 0), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##1,1", &valueMatrix(1, 1), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##1,2", &valueMatrix(1, 2), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }

        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##2,0", &valueMatrix(2, 0), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##2,1", &valueMatrix(2, 1), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::InputDouble("##2,2", &valueMatrix(2, 2), 0.0, 0.0, "%.1f"))
        {
            flow::ApplyChanges();
        }
        /* ----------------------------------------------- Function ----------------------------------------------- */
        ImGui::TableNextColumn();
        if (ImGui::Button("Call Function"))
        {
            receivedDataFromCallback = callInputFunction<std::string>(InputPortIndex_Function, 20, callbackInt, callbackBool);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(itemWidth);
        if (ImGui::SliderInt("##CallbackInt", &callbackInt, -10, 10))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("##CallbackBool", &callbackBool))
        {
            flow::ApplyChanges();
        }

        ImGui::Text("%s", receivedDataFromCallback.c_str());

        ImGui::TableNextColumn();

        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::Demo::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["outputFrequency"] = outputFrequency;
    j["nPollData"] = nPollData;
    j["valueBool"] = valueBool;
    j["valueInt"] = valueInt;
    j["valueFloat"] = valueFloat;
    j["valueDouble"] = valueDouble;
    j["valueString"] = valueString;
    j["valueObject"] = valueObject;
    j["valueMatrix"] = valueMatrix;
    j["callbackInt"] = callbackInt;
    j["callbackBool"] = callbackBool;

    return j;
}

void NAV::Demo::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputFrequency"))
    {
        j.at("outputFrequency").get_to(outputFrequency);
    }
    if (j.contains("nPollData"))
    {
        j.at("nPollData").get_to(nPollData);
    }
    if (j.contains("valueBool"))
    {
        j.at("valueBool").get_to(valueBool);
    }
    if (j.contains("valueInt"))
    {
        j.at("valueInt").get_to(valueInt);
    }
    if (j.contains("valueFloat"))
    {
        j.at("valueFloat").get_to(valueFloat);
    }
    if (j.contains("valueDouble"))
    {
        j.at("valueDouble").get_to(valueDouble);
    }
    if (j.contains("valueString"))
    {
        j.at("valueString").get_to(valueString);
    }
    if (j.contains("valueObject"))
    {
        j.at("valueObject").get_to(valueObject);
    }
    if (j.contains("valueMatrix"))
    {
        j.at("valueMatrix").get_to(valueMatrix);
    }
    if (j.contains("callbackInt"))
    {
        j.at("callbackInt").get_to(callbackInt);
    }
    if (j.contains("callbackBool"))
    {
        j.at("callbackBool").get_to(callbackBool);
    }
}

bool NAV::Demo::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // To Show the Initialization in the GUI
    std::this_thread::sleep_until(std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(3000));

    // Currently crashes clang-tidy (Stack dump: #0 Calling std::chrono::operator<=>), so use sleep_until
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    receivedDataFromSensorCnt = 0;
    receivedDataFromFileReaderCnt = 0;

    callbackCounter = 0;
    receivedDataFromCallback = "";

    int outputInterval = static_cast<int>(1.0 / static_cast<double>(outputFrequency) * 1000.0);
    timer.start(outputInterval, readSensorDataThread, this);

    return true;
}

void NAV::Demo::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (timer.is_running())
    {
        timer.stop();
    }
}

bool NAV::Demo::resetNode()
{
    LOG_TRACE("{}: called", nameId());
    // Here you could reset a FileReader
    iPollData = 0;

    return true;
}

bool NAV::Demo::onCreateLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    return true;
}

void NAV::Demo::onDeleteLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));
}

void NAV::Demo::receiveSensorData(const std::shared_ptr<NodeData>& /*nodeData*/, ax::NodeEditor::LinkId /*linkId*/)
{
    LOG_INFO("{}: received Sensor Data", nameId());

    receivedDataFromSensorCnt++;
}

void NAV::Demo::receiveFileReaderData(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<InsObs>(nodeData);
    LOG_INFO("{}: received FileReader Data: {}", nameId(), obs->insTime->GetStringOfDate());

    receivedDataFromFileReaderCnt++;
}

void NAV::Demo::readSensorDataThread(void* userData)
{
    auto* node = static_cast<Demo*>(userData);

    auto obs = std::make_shared<NodeData>();
    node->invokeCallbacks(OutputPortIndex_NodeData, obs);
}

std::shared_ptr<NAV::NodeData> NAV::Demo::pollData(bool peek)
{
    if (iPollData >= nPollData)
    {
        return nullptr;
    }
    if (!peek)
    {
        iPollData++;
    }

    auto obs = std::make_shared<InsObs>();

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto* t = std::localtime(&now);

    obs->insTime = InsTime(static_cast<uint16_t>(t->tm_year),
                           static_cast<uint16_t>(t->tm_mon),
                           static_cast<uint16_t>(t->tm_mday),
                           static_cast<uint16_t>(t->tm_hour),
                           static_cast<uint16_t>(t->tm_min),
                           static_cast<long double>(t->tm_sec));

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(OutputPortIndex_InsObs, obs);
    }

    return obs;
}

std::string NAV::Demo::callbackFunction(int integer1, int integer2, bool boolean)
{
    callbackCounter++;
    return fmt::format("{} called {} time{}\nwith parameters: {}, {}, {}",
                       nameId(),
                       callbackCounter,
                       callbackCounter > 1 ? "s" : "",
                       integer1, integer2, boolean);
}