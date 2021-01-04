/// @file EmlidDataLogger.hpp
/// @brief Data Logger for Emlid observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-06-23

#pragma once

#include "Nodes/Node.hpp"
#include "Nodes/DataLogger/Protocol/FileWriter.hpp"

namespace NAV
{
class NodeData;

/// Data Logger for Emlid observations
class EmlidDataLogger : public Node, public FileWriter
{
  public:
    /// @brief Default constructor
    EmlidDataLogger();
    /// @brief Destructor
    ~EmlidDataLogger() override;
    /// @brief Copy constructor
    EmlidDataLogger(const EmlidDataLogger&) = delete;
    /// @brief Move constructor
    EmlidDataLogger(EmlidDataLogger&&) = delete;
    /// @brief Copy assignment operator
    EmlidDataLogger& operator=(const EmlidDataLogger&) = delete;
    /// @brief Move assignment operator
    EmlidDataLogger& operator=(EmlidDataLogger&&) = delete;

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
    void writeObservation(std::shared_ptr<NodeData> nodeData);
};

} // namespace NAV
