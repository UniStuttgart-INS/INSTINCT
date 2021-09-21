#include "Demo.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include <chrono>
#include <thread>
#include <random>

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
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 630, 410 };

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, { typeStatic() }, this);
    nm::CreateOutputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::ImuObs::type() });
    nm::CreateOutputPin(this, "FileReader\n Data", Pin::Type::Flow, { NAV::InsObs::type() }, &Demo::pollData);
    nm::CreateOutputPin(this, "Bool", Pin::Type::Bool, { "" }, &valueBool);
    nm::CreateOutputPin(this, "Int", Pin::Type::Int, { "" }, &valueInt);
    nm::CreateOutputPin(this, "Float", Pin::Type::Float, { "" }, &valueFloat);
    nm::CreateOutputPin(this, "Double", Pin::Type::Float, { "" }, &valueDouble);
    nm::CreateOutputPin(this, "String", Pin::Type::String, { "" }, &valueString);
    nm::CreateOutputPin(this, "Object", Pin::Type::Object, { "Demo::DemoData" }, &valueObject);
    nm::CreateOutputPin(this, "Matrix", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &valueMatrix);

    nm::CreateInputPin(this, "Demo Node", Pin::Type::Delegate, { typeStatic() });
    nm::CreateInputPin(this, "Sensor\nData", Pin::Type::Flow, { NAV::ImuObs::type() }, &Demo::receiveSensorData);
    nm::CreateInputPin(this, "FileReader\n Data", Pin::Type::Flow, { NAV::InsObs::type() }, &Demo::receiveFileReaderData);
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
        const auto* connectedNode = getInputValue<Demo>(InputPortIndex_DemoNode);
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
        const auto* connectedBool = getInputValue<bool>(InputPortIndex_Bool);
        ImGui::Text("Bool: %s", connectedBool ? (*connectedBool ? "true" : "false") : "N/A");
        ImGui::TableNextColumn();
        if (ImGui::Checkbox("Bool", &valueBool))
        {
            flow::ApplyChanges();
            notifyOutputValueChanged(OutputPortIndex_Bool);
        }
        /* -------------------------------------------------- Int ------------------------------------------------- */
        ImGui::TableNextColumn();
        if (const auto* connectedInt = getInputValue<int>(InputPortIndex_Int))
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
            notifyOutputValueChanged(OutputPortIndex_Int);
        }
        /* ------------------------------------------------- Float ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (const auto* connectedFloat = getInputValue<float>(InputPortIndex_Float))
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
            notifyOutputValueChanged(OutputPortIndex_Float);
        }
        /* ------------------------------------------------ Double ------------------------------------------------ */
        ImGui::TableNextColumn();

        if (auto* connectedDouble = getInputValue<double>(InputPortIndex_Double);
            connectedDouble == nullptr)
        {
            ImGui::TextUnformatted("Double: N/A");
        }
        else if (auto floatFromDouble = static_cast<float>(*connectedDouble);
                 ImGui::DragFloat("Double", &floatFromDouble, 1.0F, 0.0F, 0.0F, "%.6f"))
        {
            *connectedDouble = floatFromDouble;
            flow::ApplyChanges();
            notifyInputValueChanged(InputPortIndex_Double);
        }
        ImGui::TableNextColumn();
        ImGui::Text("Double: %.6f", valueDouble);
        /* ------------------------------------------------ String ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (auto* connectedString = getInputValue<std::string>(InputPortIndex_String);
            connectedString == nullptr)
        {
            ImGui::Text("String: N/A");
        }
        else if (ImGui::InputText("String", connectedString))
        {
            flow::ApplyChanges();
            notifyInputValueChanged(InputPortIndex_String);
        }
        ImGui::Text("The String was updated %lu time%s", stringUpdateCounter, stringUpdateCounter > 1 || stringUpdateCounter == 0 ? "s" : "");
        ImGui::TableNextColumn();
        ImGui::Text("String: %s", valueString.c_str());
        /* ------------------------------------------------ Object ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (const auto* connectedObject = getInputValue<DemoData>(InputPortIndex_DemoData))
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
            notifyOutputValueChanged(OutputPortIndex_DemoData);
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Object", &valueObject.boolean))
        {
            flow::ApplyChanges();
            notifyOutputValueChanged(OutputPortIndex_DemoData);
        }
        /* ------------------------------------------------ Matrix ------------------------------------------------ */
        ImGui::TableNextColumn();
        if (auto* connectedMatrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Matrix);
            connectedMatrix == nullptr)
        {
            ImGui::TextUnformatted("Matrix: N/A");
        }
        else
        {
            if (ImGui::BeginTable("Init Matrix", static_cast<int>(connectedMatrix->cols() + 1),
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
            {
                ImGui::TableSetupColumn("");
                for (int64_t col = 0; col < connectedMatrix->cols(); col++)
                {
                    ImGui::TableSetupColumn(std::to_string(col).c_str());
                }
                ImGui::TableHeadersRow();
                for (int64_t row = 0; row < connectedMatrix->rows(); row++)
                {
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(std::to_string(row).c_str());
                    ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                    for (int64_t col = 0; col < connectedMatrix->cols(); col++)
                    {
                        ImGui::TableNextColumn();
                        ImGui::SetNextItemWidth(50);
                        if (ImGui::InputDouble(("##initMatrix(" + std::to_string(row) + ", " + std::to_string(col) + ")").c_str(),
                                               &(*connectedMatrix)(row, col), 0.0, 0.0, "%.1f"))
                        {
                            flow::ApplyChanges();
                            notifyInputValueChanged(InputPortIndex_Matrix);
                        }
                    }
                }
                ImGui::EndTable();
            }
        }
        ImGui::TableNextColumn();
        if (ImGui::BeginTable("Current Matrix", static_cast<int>(valueMatrix.cols() + 1),
                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
        {
            ImGui::TableSetupColumn("");
            for (int64_t col = 0; col < valueMatrix.cols(); col++)
            {
                ImGui::TableSetupColumn(std::to_string(col).c_str());
            }
            ImGui::TableHeadersRow();
            for (int64_t row = 0; row < valueMatrix.rows(); row++)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(std::to_string(row).c_str());
                ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                for (int64_t col = 0; col < valueMatrix.cols(); col++)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.1f", valueMatrix(row, col));
                }
            }

            ImGui::EndTable();
        }

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
}

bool NAV::Demo::initialize()
{
    LOG_TRACE("{}: called", nameId());

    // To Show the Initialization in the GUI
    std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(3000));

    // Currently crashes clang-tidy (Stack dump: #0 Calling std::chrono::operator<=>), so use sleep_until
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    receivedDataFromSensorCnt = 0;
    receivedDataFromFileReaderCnt = 0;

    stringUpdateCounter = 0;

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

void NAV::Demo::receiveSensorData(const std::shared_ptr<const NodeData>& /*nodeData*/, ax::NodeEditor::LinkId /*linkId*/)
{
    LOG_INFO("{}: received Sensor Data", nameId());

    receivedDataFromSensorCnt++;
}

void NAV::Demo::receiveFileReaderData(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::dynamic_pointer_cast<const InsObs>(nodeData);
    LOG_INFO("{}: received FileReader Data: {}", nameId(), obs->insTime->GetStringOfDate());

    receivedDataFromFileReaderCnt++;
}

void NAV::Demo::readSensorDataThread(void* userData)
{
    auto* node = static_cast<Demo*>(userData);

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

    node->invokeCallbacks(OutputPortIndex_NodeData, obs);
}

std::shared_ptr<const NAV::NodeData> NAV::Demo::pollData(bool peek)
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
    auto* t = std::localtime(&now); // NOLINT(concurrency-mt-unsafe) // FIXME: error: function is not thread safe

    obs->insTime = InsTime(static_cast<uint16_t>(t->tm_year + 1900),
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

void NAV::Demo::stringUpdatedNotifyFunction(ax::NodeEditor::LinkId /*linkId*/)
{
    stringUpdateCounter++;
}