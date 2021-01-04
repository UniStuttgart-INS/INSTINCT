#include "NodeRegistry.hpp"

#include "util/Logger.hpp"

#include "Nodes/Node.hpp"
#include "NodeData/NodeData.hpp"

#include <string_view>

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::NodeRegistry
{
/// List of all registered nodes
std::vector<NodeInfo> registeredNodes_;

/// List of all registered node data types
std::map<std::string_view, std::vector<std::string_view>> registeredNodeDataTypes_;

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

bool NAV::NodeRegistry::NodeDataTypeIsChildOf(const std::string_view& childType, const std::string_view& parentType)
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
                if (parentTypeOfChild == parentType)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

#include "Nodes/util/GroupBox.hpp"
// Data Logger
#include "Nodes/DataLogger/GNSS/EmlidDataLogger.hpp"
#include "Nodes/DataLogger/GNSS/UbloxDataLogger.hpp"
#include "Nodes/DataLogger/IMU/ImuDataLogger.hpp"
#include "Nodes/DataLogger/IMU/KvhDataLogger.hpp"
#include "Nodes/DataLogger/IMU/VectorNavDataLogger.hpp"
// Data Provider
#include "Nodes/DataProvider/GNSS/FileReader/EmlidFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RtklibPosFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/UbloxFile.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/EmlidSensor.hpp"
#include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"
#include "Nodes/DataProvider/IMU/FileReader/ImuFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/KvhFile.hpp"
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"

void NAV::NodeRegistry::registerNodeTypes()
{
    LOG_TRACE("called");

    // GroupBox
    registerNodeType<GroupBox>();
    // Data Logger
    registerNodeType<EmlidDataLogger>();
    registerNodeType<UbloxDataLogger>();
    registerNodeType<ImuDataLogger>();
    registerNodeType<KvhDataLogger>();
    registerNodeType<VectorNavDataLogger>();
    // Data Provider
    registerNodeType<EmlidFile>();
    registerNodeType<RtklibPosFile>();
    registerNodeType<UbloxFile>();
    registerNodeType<EmlidSensor>();
    registerNodeType<UbloxSensor>();
    registerNodeType<ImuFile>();
    registerNodeType<KvhFile>();
    registerNodeType<VectorNavFile>();
    registerNodeType<VectorNavSensor>();
}

#include "NodeData/NodeData.hpp"
#include "NodeData/InsObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"

void NAV::NodeRegistry::registerNodeDataTypes()
{
    registerNodeDataType<NodeData>();
    registerNodeDataType<InsObs>();
    // IMU
    registerNodeDataType<ImuObs>();
    registerNodeDataType<KvhObs>();
    registerNodeDataType<VectorNavObs>();
}