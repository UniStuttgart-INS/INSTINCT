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
/// @tparam std::enable_if_t<std::is_base_of_v<Node, T>> Makes sure template only exists for classes with base class 'Nodes'
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<Node, T>>>
void registerNodeType()
{
    NodeInfo info;
    info.constructor = []() { return new T(); }; // NOLINT(cppcoreguidelines-owning-memory)
    info.type = T::typeStatic();

    T obj;
    for (const Pin& pin : obj.inputPins)
    {
        info.pinInfoList.emplace_back(pin.kind, pin.type, pin.dataIdentifier);
    }
    for (const Pin& pin : obj.outputPins)
    {
        info.pinInfoList.emplace_back(pin.kind, pin.type, pin.dataIdentifier);
    }

    _registeredNodes[T::category()].push_back(info);
}

/// @brief Register a NodeData with the NodeManager
/// @tparam T NodeData Class to register
/// @tparam std::enable_if_t<std::is_base_of_v<NodeData, T>> Makes sure template only exists for classes with base class 'NodeData'
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
                 && NAV::NodeRegistry::NodeDataTypeIsChildOf(startPinDataIdentifier, endPinDataIdentifier))
                || (pinInfo.type == Pin::Type::Delegate
                    && std::find(endPinDataIdentifier.begin(), endPinDataIdentifier.end(), startPinParentNodeType) != endPinDataIdentifier.end())
                || ((pinInfo.type == Pin::Type::Object || pinInfo.type == Pin::Type::Matrix) // NOLINT(misc-redundant-expression) // FIXME: error: equivalent expression on both sides of logical operator
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

bool NAV::NodeRegistry::NodeDataTypeIsChildOf(const std::vector<std::string>& childTypes, const std::vector<std::string>& parentTypes)
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
                        if (NodeDataTypeIsChildOf({ parentTypeOfChild }, { parentType }))
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
        returnTypes.insert(returnTypes.end(), parentTypes.begin(), parentTypes.end());

        // Add parents of parents
        for (const auto& parentType : parentTypes)
        {
            const auto& parentParentTypes = GetParentNodeDataTypes(parentType);
            returnTypes.insert(returnTypes.end(), parentParentTypes.begin(), parentParentTypes.end());
        }
    }

    return returnTypes;
}

// Utility
#include "Nodes/util/Demo.hpp"
#include "Nodes/util/GroupBox.hpp"
// Simple
#include "Nodes/Simple/Combiner.hpp"
#include "Nodes/Simple/Delay.hpp"
#include "Nodes/Simple/Transformation.hpp"
// Converter
#include "Nodes/Converter/IMU/VectorNavBinaryConverter.hpp"
// Data Logger
#include "Nodes/DataLogger/GNSS/EmlidDataLogger.hpp"
#include "Nodes/DataLogger/GNSS/UbloxDataLogger.hpp"
#include "Nodes/DataLogger/IMU/ImuDataLogger.hpp"
#include "Nodes/DataLogger/IMU/KvhDataLogger.hpp"
#include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"
// Data Processor
#include "Nodes/DataProcessor/Integrator/ImuIntegrator.hpp"
#include "Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF.hpp"
#include "Nodes/DataProcessor/KalmanFilter/AddImuBias.hpp"
#include "Nodes/DataProcessor/KalmanFilter/AddPVAError.hpp"
// Data Provider
#include "Nodes/DataProvider/GNSS/FileReader/EmlidFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RtklibPosFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/UbloxFile.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/EmlidSensor.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"
#include "Nodes/DataProvider/IMU/FileReader/ImuFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/KvhFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/DataProvider/IMU/NetworkStream/SkydelNetworkStream.hpp"
#include "Nodes/DataProvider/IMU/Sensors/KvhSensor.hpp"
#include "Nodes/DataProvider/IMU/Sensors/Navio2Sensor.hpp"
#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"
// Data Simulator
#include "Nodes/DataProvider/IMU/Simulators/ImuSimulator.hpp"
// Plotting
#include "Nodes/Plotting/Plot.hpp"
// State
#include "Nodes/State/PosVelAttInitializer.hpp"
// Experimental
#include "Nodes/Experimental/Simple/Matrix.hpp"
#include "Nodes/Experimental/DataProcessor/ARMA.hpp"

void NAV::NodeRegistry::RegisterNodeTypes()
{
    LOG_TRACE("called");

    // Utility
    registerNodeType<Demo>();
    registerNodeType<GroupBox>();
    // Simple
    registerNodeType<Combiner>();
    registerNodeType<Delay>();
    registerNodeType<NAV::experimental::Matrix>();
    registerNodeType<Transformation>();
    // Converter
    registerNodeType<VectorNavBinaryConverter>();
    // Data Logger
    registerNodeType<EmlidDataLogger>();
    registerNodeType<UbloxDataLogger>();
    registerNodeType<ImuDataLogger>();
    registerNodeType<KvhDataLogger>();
    registerNodeType<VectorNavDataLogger>();
    // Data Processor
    registerNodeType<ImuIntegrator>();
    registerNodeType<LooselyCoupledKF>();
    registerNodeType<AddImuBias>();
    registerNodeType<AddPVAError>();
    // Data Provider
    registerNodeType<EmlidFile>();
    registerNodeType<RtklibPosFile>();
    registerNodeType<UbloxFile>();
    registerNodeType<EmlidSensor>();
    registerNodeType<UbloxSensor>();
    registerNodeType<ImuFile>();
    registerNodeType<SkydelNetworkStream>();
    registerNodeType<KvhFile>();
    registerNodeType<VectorNavFile>();
    registerNodeType<KvhSensor>();
    registerNodeType<Navio2Sensor>();
    registerNodeType<VectorNavSensor>();
    // Data Simulator
    registerNodeType<ImuSimulator>();
    // Experimental
    registerNodeType<NAV::experimental::ARMA>();
    // Plotting
    registerNodeType<Plot>();
    // State
    registerNodeType<PosVelAttInitializer>();
}

#include "NodeData/NodeData.hpp"
#include "NodeData/InsObs.hpp"
#include "NodeData/IMU/VectorNavBinaryOutput.hpp"
#include "NodeData/General/StringObs.hpp"
#include "NodeData/GNSS/EmlidObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/GNSS/SkydelObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InertialNavSol.hpp"

void NAV::NodeRegistry::RegisterNodeDataTypes()
{
    registerNodeDataType<NodeData>();
    registerNodeDataType<InsObs>();
    // General
    registerNodeDataType<StringObs>();
    // GNSS
    registerNodeDataType<EmlidObs>();
    registerNodeDataType<RtklibPosObs>();
    registerNodeDataType<UbloxObs>();
    registerNodeDataType<SkydelObs>();
    // IMU
    registerNodeDataType<ImuObs>();
    registerNodeDataType<KvhObs>();
    registerNodeDataType<ImuObsWDelta>();
    registerNodeDataType<VectorNavBinaryOutput>();
    // State
    registerNodeDataType<PosVelAtt>();
    registerNodeDataType<InertialNavSol>();
}