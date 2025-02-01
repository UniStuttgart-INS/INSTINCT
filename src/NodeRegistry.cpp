// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "NodeRegistry.hpp"

#include "util/Logger.hpp"

#include "internal/Node/Node.hpp"
#include "NodeData/NodeData.hpp"

#include <string>

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::NodeRegistry
{
/// List of all registered nodes.
/// Key: category, Value: Nodes
std::map<std::string, std::vector<NodeInfo>> _registeredNodes;

/// List of all registered node data types.
/// Key: NodeData.type(), Value: parentTypes()
std::map<std::string, std::vector<std::string>> _registeredNodeDataTypes;

} // namespace NAV::NodeRegistry
/* -------------------------------------------------------------------------------------------------------- */
/*                                       Private Function Declarations                                      */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::NodeRegistry
{
/// @brief Registers a Node with the NodeManager
/// @tparam T Node Class to register
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
void registerNodeType()
{
    NodeInfo info;
    info.constructor = []() { return new T(); }; // NOLINT(cppcoreguidelines-owning-memory)
    info.type = T::typeStatic();

    T obj;
    for (const InputPin& pin : obj.inputPins)
    {
        info.pinInfoList.emplace_back(pin.kind, pin.type, pin.dataIdentifier);
    }
    for (const OutputPin& pin : obj.outputPins)
    {
        info.pinInfoList.emplace_back(pin.kind, pin.type, pin.dataIdentifier);
    }

    _registeredNodes[T::category()].push_back(info);
}

/// @brief Register a NodeData with the NodeManager
/// @tparam T NodeData Class to register
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<NodeData, T>>>
void registerNodeDataType()
{
    _registeredNodeDataTypes[T::type()] = T::parentTypes();
}

} // namespace NAV::NodeRegistry

/* -------------------------------------------------------------------------------------------------------- */
/*                                           Function Definitions                                           */
/* -------------------------------------------------------------------------------------------------------- */

bool NAV::NodeRegistry::NodeInfo::hasCompatiblePin(const Pin* pin) const
{
    if (pin == nullptr)
    {
        return true;
    }

    Pin::Kind searchPinKind = pin->kind == Pin::Kind::Input ? Pin::Kind::Output : Pin::Kind::Input;
    for (const auto& pinInfo : this->pinInfoList)
    {
        const std::vector<std::string>& startPinDataIdentifier = pin->kind == Pin::Kind::Input ? pinInfo.dataIdentifier : pin->dataIdentifier;
        const std::vector<std::string>& endPinDataIdentifier = pin->kind == Pin::Kind::Input ? pin->dataIdentifier : pinInfo.dataIdentifier;
        const std::string& startPinParentNodeType = pin->kind == Pin::Kind::Input ? this->type : pin->parentNode->type();

        if (pinInfo.kind == searchPinKind && pinInfo.type == pin->type)
        {
            if ((pinInfo.type == Pin::Type::Flow
                 && NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(startPinDataIdentifier, endPinDataIdentifier))
                || (pinInfo.type == Pin::Type::Delegate
                    && std::find(endPinDataIdentifier.begin(), endPinDataIdentifier.end(), startPinParentNodeType) != endPinDataIdentifier.end())
                || ((pinInfo.type == Pin::Type::Object || pinInfo.type == Pin::Type::Matrix) // NOLINT(misc-redundant-expression) - false positive warning
                    && Pin::dataIdentifierHaveCommon(startPinDataIdentifier, endPinDataIdentifier))
                || pinInfo.type == Pin::Type::Bool || pinInfo.type == Pin::Type::Int || pinInfo.type == Pin::Type::Float || pinInfo.type == Pin::Type::String)
            {
                return true;
            }
        }
    }

    return false;
}

const std::map<std::string, std::vector<NAV::NodeRegistry::NodeInfo>>& NAV::NodeRegistry::RegisteredNodes()
{
    return _registeredNodes;
}

bool NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(const std::vector<std::string>& childTypes, const std::vector<std::string>& parentTypes)
{
    for (const auto& childType : childTypes)
    {
        for (const auto& parentType : parentTypes)
        {
            if (childType == parentType)
            {
                return true;
            }
            for (const auto& [dataType, parentTypes] : _registeredNodeDataTypes)
            {
                if (dataType == childType)
                {
                    for (const auto& parentTypeOfChild : parentTypes)
                    {
                        if (NodeDataTypeAnyIsChildOf({ parentTypeOfChild }, { parentType }))
                        {
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}

std::vector<std::string> NAV::NodeRegistry::GetParentNodeDataTypes(const std::string& type)
{
    std::vector<std::string> returnTypes;
    if (_registeredNodeDataTypes.contains(type))
    {
        const auto& parentTypes = _registeredNodeDataTypes.at(type);

        // Add all the immediate parents
        if (!parentTypes.empty())
        {
            returnTypes.insert(returnTypes.end(), parentTypes.begin(), parentTypes.end());
        }

        // Add parents of parents
        for (const auto& parentType : parentTypes)
        {
            auto parentParentTypes = GetParentNodeDataTypes(parentType);
            if (!parentParentTypes.empty())
            {
                returnTypes.insert(returnTypes.end(), parentParentTypes.begin(), parentParentTypes.end());
            }
        }
    }

    return returnTypes;
}

// Utility
#include "Nodes/Utility/Combiner.hpp"
#include "Nodes/Utility/Demo.hpp"
#include "Nodes/Utility/GroupBox.hpp"
#include "Nodes/Utility/Merger.hpp"
#include "Nodes/Utility/Terminator.hpp"
#include "Nodes/Utility/TimeWindow.hpp"
// Converter
#include "Nodes/Converter/GNSS/RtklibPosConverter.hpp"
#include "Nodes/Converter/GNSS/UartPacketConverter.hpp"
#include "Nodes/Converter/GNSS/UbloxGnssObsConverter.hpp"
#include "Nodes/Converter/GNSS/UbloxGnssOrbitCollector.hpp"
#include "Nodes/Converter/IMU/VectorNavBinaryConverter.hpp"
// Data Link
#if __linux__ || __APPLE__
    #include "Nodes/DataLink/mavlinkSend.hpp"
#endif
#include "Nodes/DataLink/udpSend.hpp"
#include "Nodes/DataLink/udpRecv.hpp"
// Data Logger
#include "Nodes/DataLogger/General/CsvLogger.hpp"
#include "Nodes/DataLogger/General/KmlLogger.hpp"
#include "Nodes/DataLogger/General/MatrixLogger.hpp"
#include "Nodes/DataLogger/GNSS/RinexObsLogger.hpp"
#include "Nodes/DataLogger/GNSS/UartDataLogger.hpp"
#include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"
#include "Nodes/DataLogger/WiFi/WiFiObsLogger.hpp"
// Data Processor
#include "Nodes/DataProcessor/ErrorModel/AllanDeviation.hpp"
#include "Nodes/DataProcessor/ErrorModel/ErrorModel.hpp"
#include "Nodes/DataProcessor/GNSS/GnssAnalyzer.hpp"
#include "Nodes/DataProcessor/GNSS/SinglePointPositioning.hpp"
#include "Nodes/DataProcessor/Integrator/ImuIntegrator.hpp"
#include "Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF.hpp"
#include "Nodes/DataProcessor/KalmanFilter/TightlyCoupledKF.hpp"
#include "Nodes/DataProcessor/SensorCombiner/ImuFusion.hpp"
#include "Nodes/DataProcessor/WiFi/WiFiPositioning.hpp"
// Data Provider
#include "Nodes/DataProvider/CSV/CsvFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RinexNavFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RinexObsFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/EmlidFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RtklibPosFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/NmeaFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/UbloxFile.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/EmlidSensor.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"
#include "Nodes/DataProvider/IMU/FileReader/ImuFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/KvhFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/DataProvider/IMU/Sensors/KvhSensor.hpp"
#include "Nodes/DataProvider/IMU/Sensors/Navio2Sensor.hpp"
#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"
#include "Nodes/DataProvider/IMU/FileReader/UlogFile.hpp"
#include "Nodes/DataProvider/State/PosVelAttFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/MultiImuFile.hpp"
#include "Nodes/DataProvider/WiFi/Sensors/EspressifSensor.hpp"
#include "Nodes/DataProvider/WiFi/Sensors/ArubaSensor.hpp"
#include "Nodes/DataProvider/WiFi/FileReader/WiFiObsFile.hpp"
// Data Simulator
#include "Nodes/DataProvider/IMU/Simulators/ImuSimulator.hpp"
// Plotting
#include "Nodes/Plotting/Plot.hpp"
// State
#include "Nodes/State/PosVelAttInitializer.hpp"
// Experimental
#include "Nodes/Experimental/DataProcessor/ARMA.hpp"
#include "Nodes/Experimental/DataProvider/IMU/NetworkStream/SkydelNetworkStream.hpp"
#include "Nodes/Experimental/Simple/Delay.hpp"

void NAV::NodeRegistry::RegisterNodeTypes()
{
    LOG_TRACE("called");

    Node::_autostartWorker = false;

    // Utility
    registerNodeType<Combiner>();
    registerNodeType<Demo>();
    registerNodeType<GroupBox>();
    registerNodeType<Merger>();
    registerNodeType<Terminator>();
    registerNodeType<TimeWindow>();
    // Converter
    registerNodeType<RtklibPosConverter>();
    registerNodeType<UartPacketConverter>();
    registerNodeType<UbloxGnssObsConverter>();
    registerNodeType<UbloxGnssOrbitCollector>();
    registerNodeType<VectorNavBinaryConverter>();
    // Data Link
#if __linux__ || __APPLE__
    registerNodeType<MavlinkSend>();
#endif
    registerNodeType<UdpSend>();
    registerNodeType<UdpRecv>();
    // Data Logger
    registerNodeType<CsvLogger>();
    registerNodeType<KmlLogger>();
    registerNodeType<MatrixLogger>();
    registerNodeType<RinexObsLogger>();
    registerNodeType<UartDataLogger>();
    registerNodeType<VectorNavDataLogger>();
    registerNodeType<WiFiObsLogger>();
    // Data Processor
    registerNodeType<AllanDeviation>();
    registerNodeType<ErrorModel>();
    registerNodeType<GnssAnalyzer>();
    registerNodeType<SinglePointPositioning>();
    registerNodeType<ImuIntegrator>();
    registerNodeType<LooselyCoupledKF>();
    // registerNodeType<TightlyCoupledKF>();
    registerNodeType<ImuFusion>();
    registerNodeType<WiFiPositioning>();
    // Data Provider
    registerNodeType<CsvFile>();
    registerNodeType<RinexNavFile>();
    registerNodeType<RinexObsFile>();
    registerNodeType<EmlidFile>();
    registerNodeType<RtklibPosFile>();
    registerNodeType<NmeaFile>();
    registerNodeType<UbloxFile>();
    registerNodeType<EmlidSensor>();
    registerNodeType<UbloxSensor>();
    registerNodeType<ImuFile>();
    registerNodeType<KvhFile>();
    registerNodeType<VectorNavFile>();
    registerNodeType<KvhSensor>();
    registerNodeType<Navio2Sensor>();
    registerNodeType<VectorNavSensor>();
    registerNodeType<UlogFile>();
    registerNodeType<PosVelAttFile>();
    registerNodeType<MultiImuFile>();
    registerNodeType<EspressifSensor>();
    registerNodeType<ArubaSensor>();
    registerNodeType<WiFiObsFile>();
    // Data Simulator
    registerNodeType<ImuSimulator>();
    // Experimental
    // registerNodeType<NAV::experimental::ARMA>();
    // registerNodeType<NAV::experimental::SkydelNetworkStream>();
    // registerNodeType<NAV::experimental::Delay>();
    // Plotting
    registerNodeType<Plot>();
    // State
    registerNodeType<PosVelAttInitializer>();

    Node::_autostartWorker = true;
}

#include "NodeData/General/DynamicData.hpp"
#include "NodeData/General/StringObs.hpp"
#include "NodeData/GNSS/EmlidObs.hpp"
#include "NodeData/GNSS/GnssCombination.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/SppSolution.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsSimulated.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/VectorNavBinaryOutput.hpp"
#include "NodeData/State/InsGnssLCKFSolution.hpp"
#include "NodeData/State/InsGnssTCKFSolution.hpp"
#include "NodeData/State/Pos.hpp"
#include "NodeData/State/PosVel.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/WiFi/WiFiObs.hpp"
#include "NodeData/WiFi/WiFiPositioningSolution.hpp"

void NAV::NodeRegistry::RegisterNodeDataTypes()
{
    registerNodeDataType<NodeData>();
    // General
    registerNodeDataType<DynamicData>();
    registerNodeDataType<StringObs>();
    // GNSS
    registerNodeDataType<EmlidObs>();
    registerNodeDataType<GnssCombination>();
    registerNodeDataType<GnssObs>();
    registerNodeDataType<RtklibPosObs>();
    registerNodeDataType<SppSolution>();
    registerNodeDataType<UbloxObs>();
    // IMU
    registerNodeDataType<ImuObs>();
    registerNodeDataType<ImuObsSimulated>();
    registerNodeDataType<ImuObsWDelta>();
    registerNodeDataType<KvhObs>();
    registerNodeDataType<VectorNavBinaryOutput>();
    // State
    registerNodeDataType<InsGnssLCKFSolution>();
    registerNodeDataType<InsGnssTCKFSolution>();
    registerNodeDataType<Pos>();
    registerNodeDataType<PosVel>();
    registerNodeDataType<PosVelAtt>();
    // WiFi
    registerNodeDataType<WiFiObs>();
    registerNodeDataType<WiFiPositioningSolution>();
}