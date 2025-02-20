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
namespace
{

/// List of all registered nodes.
/// Key: category, Value: Nodes
std::map<std::string, std::vector<NodeInfo>> _registeredNodes;

/// List of all registered node data types.
/// Key: NodeData.type(), Value: parentTypes()
std::map<std::string, std::vector<std::string>> _registeredNodeDataTypes;

/* -------------------------------------------------------------------------------------------------------- */
/*                                       Private Function Declarations                                      */
/* -------------------------------------------------------------------------------------------------------- */

/// @brief Registers a Node with the NodeManager
/// @tparam T Node Class to register
template<std::derived_from<Node> T>
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
template<std::derived_from<NodeData> T>
void registerNodeDataType()
{
    _registeredNodeDataTypes[T::type()] = T::parentTypes();
}

} // namespace
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
                    && std::ranges::find(endPinDataIdentifier, startPinParentNodeType) != endPinDataIdentifier.end())
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
#include "Nodes/DataProcessor/SensorCombiner/ImuFusion.hpp"
#include "Nodes/DataProcessor/WiFi/WiFiPositioning.hpp"
#include "Nodes/DataProcessor/Filter/LowPassFilter.hpp"
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
    registerNodeType<LowPassFilter>();
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
    registerNodeDataType<Pos>();
    registerNodeDataType<PosVel>();
    registerNodeDataType<PosVelAtt>();
    // WiFi
    registerNodeDataType<WiFiObs>();
    registerNodeDataType<WiFiPositioningSolution>();
}

std::vector<std::string> NAV::NodeRegistry::GetStaticDataDescriptors(const std::vector<std::string>& dataIdentifier)
{
    // ATTENTION: Entries need to be in correct inheritance order (deepest inheritance first)

    // NodeData
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { DynamicData::type() })) { return DynamicData::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { StringObs::type() })) { return StringObs::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { GnssCombination::type() })) { return GnssCombination::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { GnssObs::type() })) { return GnssObs::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { VectorNavBinaryOutput::type() })) { return VectorNavBinaryOutput::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { EmlidObs::type() })) { return EmlidObs::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { UbloxObs::type() })) { return UbloxObs::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { WiFiObs::type() })) { return WiFiObs::GetStaticDataDescriptors(); }
    // Pos
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { InsGnssTCKFSolution::type() })) { return InsGnssTCKFSolution::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { InsGnssLCKFSolution::type() })) { return InsGnssLCKFSolution::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { PosVelAtt::type() })) { return PosVelAtt::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { SppSolution::type() })) { return SppSolution::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { RtklibPosObs::type() })) { return RtklibPosObs::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { WiFiPositioningSolution::type() })) { return WiFiPositioningSolution::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { PosVel::type() })) { return PosVel::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { Pos::type() })) { return Pos::GetStaticDataDescriptors(); }
    // ImuObs
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { ImuObsSimulated::type() })) { return ImuObsSimulated::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { ImuObsWDelta::type() })) { return ImuObsWDelta::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { KvhObs::type() })) { return KvhObs::GetStaticDataDescriptors(); }
    if (NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(dataIdentifier, { ImuObs::type() })) { return ImuObs::GetStaticDataDescriptors(); }

    return {};
}

bool NAV::NodeRegistry::TypeHasDynamicData(const std::string& type)
{
    return type == DynamicData::type()
           || type == GnssCombination::type()
           || type == GnssObs::type()
           || type == SppSolution::type();
}

std::shared_ptr<NAV::NodeData> NAV::NodeRegistry::CopyNodeData(const std::shared_ptr<const NodeData>& nodeData)
{
    if (nodeData == nullptr) { return nullptr; }

    // General
    if (nodeData->getType() == DynamicData::type()) { return std::make_shared<DynamicData>(*std::static_pointer_cast<const DynamicData>(nodeData)); }
    if (nodeData->getType() == StringObs::type()) { return std::make_shared<StringObs>(*std::static_pointer_cast<const StringObs>(nodeData)); }
    // GNSS
    if (nodeData->getType() == EmlidObs::type()) { return std::make_shared<EmlidObs>(*std::static_pointer_cast<const EmlidObs>(nodeData)); }
    if (nodeData->getType() == GnssCombination::type()) { return std::make_shared<GnssCombination>(*std::static_pointer_cast<const GnssCombination>(nodeData)); }
    if (nodeData->getType() == GnssObs::type()) { return std::make_shared<GnssObs>(*std::static_pointer_cast<const GnssObs>(nodeData)); }
    if (nodeData->getType() == RtklibPosObs::type()) { return std::make_shared<RtklibPosObs>(*std::static_pointer_cast<const RtklibPosObs>(nodeData)); }
    if (nodeData->getType() == SppSolution::type()) { return std::make_shared<SppSolution>(*std::static_pointer_cast<const SppSolution>(nodeData)); }
    if (nodeData->getType() == UbloxObs::type()) { return std::make_shared<UbloxObs>(*std::static_pointer_cast<const UbloxObs>(nodeData)); }
    // IMU
    if (nodeData->getType() == ImuObs::type()) { return std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(nodeData)); }
    if (nodeData->getType() == ImuObsSimulated::type()) { return std::make_shared<ImuObsSimulated>(*std::static_pointer_cast<const ImuObsSimulated>(nodeData)); }
    if (nodeData->getType() == ImuObsWDelta::type()) { return std::make_shared<ImuObsWDelta>(*std::static_pointer_cast<const ImuObsWDelta>(nodeData)); }
    if (nodeData->getType() == KvhObs::type()) { return std::make_shared<KvhObs>(*std::static_pointer_cast<const KvhObs>(nodeData)); }
    if (nodeData->getType() == VectorNavBinaryOutput::type()) { return std::make_shared<VectorNavBinaryOutput>(*std::static_pointer_cast<const VectorNavBinaryOutput>(nodeData)); }
    // State
    if (nodeData->getType() == InsGnssLCKFSolution::type()) { return std::make_shared<InsGnssLCKFSolution>(*std::static_pointer_cast<const InsGnssLCKFSolution>(nodeData)); }
    if (nodeData->getType() == InsGnssTCKFSolution::type()) { return std::make_shared<InsGnssTCKFSolution>(*std::static_pointer_cast<const InsGnssTCKFSolution>(nodeData)); }
    if (nodeData->getType() == Pos::type()) { return std::make_shared<Pos>(*std::static_pointer_cast<const Pos>(nodeData)); }
    if (nodeData->getType() == PosVel::type()) { return std::make_shared<PosVel>(*std::static_pointer_cast<const PosVel>(nodeData)); }
    if (nodeData->getType() == PosVelAtt::type()) { return std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(nodeData)); }
    // WiFi
    if (nodeData->getType() == WiFiObs::type()) { return std::make_shared<WiFiObs>(*std::static_pointer_cast<const WiFiObs>(nodeData)); }
    if (nodeData->getType() == WiFiPositioningSolution::type()) { return std::make_shared<WiFiPositioningSolution>(*std::static_pointer_cast<const WiFiPositioningSolution>(nodeData)); }

    return nullptr;
}