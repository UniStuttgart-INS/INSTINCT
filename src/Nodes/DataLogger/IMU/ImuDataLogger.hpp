/// @file ImuDataLogger.hpp
/// @brief Data Logger for Imu observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-30

#pragma once

#include "Nodes/Node.hpp"
#include "Nodes/DataLogger/Protocol/FileWriter.hpp"

namespace NAV
{
class NodeData;

/// Data Logger for IMU observations
class ImuDataLogger : public Node, public FileWriter
{
  public:
    /// @brief Default constructor
    ImuDataLogger();
    /// @brief Destructor
    ~ImuDataLogger() override;
    /// @brief Copy constructor
    ImuDataLogger(const ImuDataLogger&) = delete;
    /// @brief Move constructor
    ImuDataLogger(ImuDataLogger&&) = delete;
    /// @brief Copy assignment operator
    ImuDataLogger& operator=(const ImuDataLogger&) = delete;
    /// @brief Move assignment operator
    ImuDataLogger& operator=(ImuDataLogger&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

  private:
    /// @brief Write Observation to the file
    /// @param[in] nodeData The received observation
    /// @param[in] linkId Id of the link over which the data is received
    void writeObservation(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);
};

} // namespace NAV
