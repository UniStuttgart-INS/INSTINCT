#include "ImuDataLogger.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "util/Logger.hpp"

#include "gui/widgets/FileDialog.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::ImuDataLogger::ImuDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::ASCII;

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuDataLogger::writeObservation);
    //nm::CreateInputPin(this, "writeObservation", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" }, &ImuDataLogger::writeObservation);
}

NAV::ImuDataLogger::~ImuDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuDataLogger::typeStatic()
{
    return "ImuDataLogger";
}

std::string NAV::ImuDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::ImuDataLogger::category()
{
    return "Data Logger";
}

void NAV::ImuDataLogger::guiConfig()
{
    if (gui::widgets::FileDialogSave(path, "Save File", ".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::ImuDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::ImuDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

bool NAV::ImuDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    filestream << "GpsCycle,GpsWeek,GpsToW,TimeStartup,"
               << "UnCompMagX,UnCompMagY,UnCompMagZ,UnCompAccX,UnCompAccY,UnCompAccZ,UnCompGyroX,UnCompGyroY,UnCompGyroZ,"
               << "Temperature" << std::endl;

    return true;
}

void NAV::ImuDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}
void NAV::ImuDataLogger::writeObservation(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
//void NAV::ImuDataLogger::writeObservation(ax::NodeEditor::LinkId linkId)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 9;

    if (obs->insTime.has_value())
    {
        filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.value().toGPSweekTow().gpsCycle;
    }
    filestream << ",";
    if (obs->insTime.has_value())
    {
        filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().gpsWeek;
    }
    filestream << ",";
    if (obs->insTime.has_value())
    {
        filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().tow;
    }
    filestream << ",";
    if (obs->timeSinceStartup.has_value())
    {
        filestream << std::setprecision(valuePrecision) << obs->timeSinceStartup.value();
    }
    filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        filestream << obs->magUncompXYZ.value().x();
    }
    filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        filestream << obs->magUncompXYZ.value().y();
    }
    filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        filestream << obs->magCompXYZ.value().z();
    }
    filestream << ",";
    if (obs->accelCompXYZ.has_value())
    {
        filestream << obs->accelCompXYZ.value().x();
    }
    filestream << ",";
    if (obs->accelCompXYZ.has_value())
    {
        filestream << obs->accelCompXYZ.value().y();
    }
    filestream << ",";
    if (obs->accelCompXYZ.has_value())
    {
        filestream << obs->accelCompXYZ.value().z();
    }
    filestream << ",";
    if (obs->gyroCompXYZ.has_value())
    {
        filestream << obs->gyroCompXYZ.value().x();
    }
    filestream << ",";
    if (obs->gyroCompXYZ.has_value())
    {
        filestream << obs->gyroCompXYZ.value().y();
    }
    filestream << ",";
    if (obs->gyroCompXYZ.has_value())
    {
        filestream << obs->gyroCompXYZ.value().z();
    }
    filestream << ",";
    if (obs->temperature.has_value())
    {
        filestream << obs->temperature.value();
        /*if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            size_t pinIndex = pinIndexFromId(link->endPinId);

            LOG_DATA("{}: called on pin {}", nameId(), pinIndex);

            if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
            {
                auto* value = getInputValue<Eigen::MatrixXd>(pinIndex);
                // Matrix
                //for (int row = 0; row < value->rows(); row++)
                //{
                for (int col = 0; col < value->cols(); col++)
                {
                    //addData(pinIndex, i++, (*value)(row, col));
                    filestream << (*value)(0, col);
                }
                //}
            }
            else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
            {
                auto* value = getInputValue<BlockMatrix>(pinIndex);

                auto matrix = (*value)();

                // Matrix
                //for (int row = 0; row < matrix.rows(); row++)
                //{
                for (int col = 0; col < matrix.cols(); col++)
                {
                    filestream << matrix(0, col);
                }
                //}
            }
        }*/
        filestream << '\n';
    }
}