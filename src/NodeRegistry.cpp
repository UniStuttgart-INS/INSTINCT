#include "NodeRegistry.hpp"

#include "util/Logger.hpp"

#include "Nodes/Node.hpp"
#include "NodeData/NodeData.hpp"

#include <string>

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::NodeRegistry
{
/// List of all registered nodes.
/// Key: category, Value: Nodes
std::map<std::string, std::vector<NodeInfo>> registeredNodes_;

/// List of all registered node data types.
/// Key: NodeData.type(), Value: parentTypes()
std::map<std::string, std::vector<std::string>> registeredNodeDataTypes_;

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

    registeredNodes_[T::category()].push_back(info);
}

/// @brief Register a NodeData with the NodeManager
/// @tparam T NodeData Class to register
/// @tparam std::enable_if_t<std::is_base_of_v<NodeData, T>> Makes sure template only exists for classes with base class 'NodeData'
template<typename T,
         typename = std::enable_if_t<std::is_base_of_v<NodeData, T>>>
void registerNodeDataType()
{
    registeredNodeDataTypes_[T::type()] = T::parentTypes();
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
                || ((pinInfo.type == Pin::Type::Object || pinInfo.type == Pin::Type::Matrix || pinInfo.type == Pin::Type::Function)
                    && (startPinDataIdentifier.empty()
                        || endPinDataIdentifier.empty()
                        || std::find(endPinDataIdentifier.begin(), endPinDataIdentifier.end(), startPinDataIdentifier.front()) != endPinDataIdentifier.end()))
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
    return registeredNodes_;
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
            for (const auto& [dataType, parentTypes] : registeredNodeDataTypes_)
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

// Utility
#include "Nodes/util/Demo.hpp"
#include "Nodes/util/GroupBox.hpp"
// Simple
#include "Nodes/Simple/Matrix.hpp"
// Data Logger
#include "Nodes/DataLogger/GNSS/EmlidDataLogger.hpp"
#include "Nodes/DataLogger/GNSS/UbloxDataLogger.hpp"
#include "Nodes/DataLogger/IMU/ImuDataLogger.hpp"
#include "Nodes/DataLogger/IMU/KvhDataLogger.hpp"
#include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"
// Data Processor
#include "Nodes/DataProcessor/Integrator/ImuIntegrator.hpp"
// Data Provider
#include "Nodes/DataProvider/GNSS/FileReader/EmlidFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RtklibPosFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/UbloxFile.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/EmlidSensor.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"
#include "Nodes/DataProvider/IMU/FileReader/ImuFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/KvhFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/DataProvider/IMU/Sensors/KvhSensor.hpp"
#include "Nodes/DataProvider/IMU/Sensors/Navio2Sensor.hpp"
#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"
// Data Simulator
#include "Nodes/DataProvider/IMU/Simulators/ImuSimulator.hpp"
// Plotting
#include "Nodes/Plotting/Plot.hpp"
// State
#include "Nodes/State/State.hpp"
// Time
#include "Nodes/Time/TimeSynchronizer.hpp"

void NAV::NodeRegistry::RegisterNodeTypes()
{
    LOG_TRACE("called");

    // Utility
    registerNodeType<Demo>();
    registerNodeType<GroupBox>();
    // Simple
    registerNodeType<Matrix>();
    // Data Logger
    registerNodeType<EmlidDataLogger>();
    registerNodeType<UbloxDataLogger>();
    registerNodeType<ImuDataLogger>();
    registerNodeType<KvhDataLogger>();
    registerNodeType<VectorNavDataLogger>();
    // Data Processor
    registerNodeType<ImuIntegrator>();
    // Data Provider
    registerNodeType<EmlidFile>();
    registerNodeType<RtklibPosFile>();
    registerNodeType<UbloxFile>();
    registerNodeType<EmlidSensor>();
    registerNodeType<UbloxSensor>();
    registerNodeType<ImuFile>();
    registerNodeType<KvhFile>();
    registerNodeType<VectorNavFile>();
    registerNodeType<KvhSensor>();
    registerNodeType<Navio2Sensor>();
    registerNodeType<VectorNavSensor>();
    // Data Simulator
    registerNodeType<ImuSimulator>();
    // Data Provider
    registerNodeType<Plot>();
    // State
    registerNodeType<State>();
    // Time
    registerNodeType<TimeSynchronizer>();
}

#include "NodeData/NodeData.hpp"
#include "NodeData/InsObs.hpp"
#include "NodeData/GNSS/EmlidObs.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"

void NAV::NodeRegistry::RegisterNodeDataTypes()
{
    registerNodeDataType<NodeData>();
    registerNodeDataType<InsObs>();
    // GNSS
    registerNodeDataType<EmlidObs>();
    registerNodeDataType<GnssObs>();
    registerNodeDataType<RtklibPosObs>();
    registerNodeDataType<UbloxObs>();
    // IMU
    registerNodeDataType<ImuObs>();
    registerNodeDataType<KvhObs>();
    registerNodeDataType<VectorNavObs>();
}