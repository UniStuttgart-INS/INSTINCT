#include "MatrixLogger.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision
#include <Eigen/Core>

#include "util/Time/TimeBase.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::MatrixLogger::MatrixLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _fileType = FileType::CSV;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "write", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    inputPins.back().notifyFunc.emplace_back(this, static_cast<void (Node::*)(ax::NodeEditor::LinkId)>(&MatrixLogger::writeMatrix), 0);
}

NAV::MatrixLogger::~MatrixLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::MatrixLogger::typeStatic()
{
    return "MatrixLogger";
}

std::string NAV::MatrixLogger::type() const
{
    return typeStatic();
}

std::string NAV::MatrixLogger::category()
{
    return "Data Logger";
}

void NAV::MatrixLogger::guiConfig()
{
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::MatrixLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::MatrixLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::MatrixLogger::flush()
{
    _filestream.flush();
}

bool NAV::MatrixLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _headerWritten = false;

    return true;
}

void NAV::MatrixLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::MatrixLogger::writeMatrix(ax::NodeEditor::LinkId linkId)
{
    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 12;

    if (!_headerWritten)
    {
        _filestream << "Time [s],GpsCycle,GpsWeek,GpsTow [s]";
    }

    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            size_t pinIndex = pinIndexFromId(link->endPinId);

            InsTime currentTime = util::time::GetCurrentInsTime();
            // Matrix
            if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
            {
                auto* value = getInputValue<Eigen::MatrixXd>(pinIndex);

                if (value != nullptr && !currentTime.empty())
                {
                    if (!_headerWritten)
                    {
                        for (int row = 0; row < value->rows(); row++)
                        {
                            for (int col = 0; col < value->cols(); col++)
                            {
                                _filestream << ",[" << row << ";" << col << "]";
                            }
                        }
                        _filestream << std::endl;
                        _headerWritten = true;
                    }

                    _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(currentTime) * 1e9) / 1e9;
                    _filestream << "," << std::fixed << std::setprecision(gpsCyclePrecision) << currentTime.toGPSweekTow().gpsCycle;
                    _filestream << "," << std::defaultfloat << std::setprecision(gpsTimePrecision) << currentTime.toGPSweekTow().gpsWeek;
                    _filestream << "," << std::defaultfloat << std::setprecision(gpsTimePrecision) << currentTime.toGPSweekTow().tow;
                    _filestream << std::setprecision(valuePrecision);

                    for (int row = 0; row < value->rows(); row++)
                    {
                        for (int col = 0; col < value->cols(); col++)
                        {
                            _filestream << "," << (*value)(row, col);
                        }
                    }
                    _filestream << "\n";
                }
            }
            else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
            {
                auto* value = getInputValue<BlockMatrix>(pinIndex);

                if (value != nullptr && !currentTime.empty())
                {
                    auto matrix = (*value)();
                    if (!_headerWritten)
                    {
                        for (int row = 0; row < matrix.rows(); row++)
                        {
                            for (int col = 0; col < matrix.cols(); col++)
                            {
                                _filestream << ",[" << row << ";" << col << "]";
                            }
                        }
                        _filestream << std::endl;
                        _headerWritten = true;
                    }

                    _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(currentTime) * 1e9) / 1e9;
                    _filestream << "," << std::fixed << std::setprecision(gpsCyclePrecision) << currentTime.toGPSweekTow().gpsCycle;
                    _filestream << "," << std::defaultfloat << std::setprecision(gpsTimePrecision) << currentTime.toGPSweekTow().gpsWeek;
                    _filestream << "," << std::defaultfloat << std::setprecision(gpsTimePrecision) << currentTime.toGPSweekTow().tow;
                    _filestream << std::setprecision(valuePrecision);

                    for (int row = 0; row < matrix.rows(); row++)
                    {
                        for (int col = 0; col < matrix.cols(); col++)
                        {
                            _filestream << "," << matrix(row, col);
                        }
                    }
                    _filestream << "\n";
                }
            }
        }
    }
}