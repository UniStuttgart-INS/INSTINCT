// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MatrixLogger.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision
#include "util/Eigen.hpp"

#include "util/Time/TimeBase.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::MatrixLogger::MatrixLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "write", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &MatrixLogger::writeMatrix);
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
        doDeinitialize();
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

void NAV::MatrixLogger::writeMatrix(const InsTime& insTime, size_t pinIdx)
{
    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 12;

    if (!_headerWritten)
    {
        _filestream << "Time [s],GpsCycle,GpsWeek,GpsTow [s]";
    }

    if (auto* sourcePin = inputPins[pinIdx].link.getConnectedPin())
    {
        // Matrix
        if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            const auto* value = getInputValue<const Eigen::MatrixXd>(INPUT_PORT_INDEX_MATRIX);

            if (value != nullptr && !insTime.empty())
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

                _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(insTime) * 1e9) / 1e9;
                _filestream << "," << std::fixed << std::setprecision(gpsCyclePrecision) << insTime.toGPSweekTow().gpsCycle;
                _filestream << "," << std::defaultfloat << std::setprecision(gpsTimePrecision) << insTime.toGPSweekTow().gpsWeek;
                _filestream << "," << std::defaultfloat << std::setprecision(gpsTimePrecision) << insTime.toGPSweekTow().tow;
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
    }
    releaseInputValue(pinIdx);
}