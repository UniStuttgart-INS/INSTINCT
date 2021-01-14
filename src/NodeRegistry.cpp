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
/// List of all registered nodes
std::vector<NodeInfo> registeredNodes_;

/// List of all registered node data types
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
    info.category = T::category();
    registeredNodes_.push_back(info);
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

const std::vector<NAV::NodeRegistry::NodeInfo>& NAV::NodeRegistry::registeredNodes()
{
    return registeredNodes_;
}

bool NAV::NodeRegistry::NodeDataTypeIsChildOf(const std::string& childType, const std::string& parentType)
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
                if (NodeDataTypeIsChildOf(parentTypeOfChild, parentType))
                {
                    return true;
                }
            }
        }
    }

    return false;
}

// Utility
#include "Nodes/util/Demo.hpp"
#include "Nodes/util/GroupBox.hpp"
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

void NAV::NodeRegistry::registerNodeTypes()
{
    LOG_TRACE("called");

    // Utility
    registerNodeType<Demo>();
    registerNodeType<GroupBox>();
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

void NAV::NodeRegistry::registerNodeDataTypes()
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