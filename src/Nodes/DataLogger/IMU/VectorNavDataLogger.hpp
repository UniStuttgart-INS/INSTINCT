/// @file VectorNavDataLogger.hpp
/// @brief Data Logger for VectorNav observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-17

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataLogger/Protocol/FileWriter.hpp"
#include "Nodes/DataLogger/Protocol/CommonLog.hpp"

namespace NAV
{
class NodeData;

/// Data Logger for VectorNav observations
class VectorNavDataLogger : public Node, public FileWriter, public CommonLog
{
  public:
    /// @brief Default constructor
    VectorNavDataLogger();
    /// @brief Destructor
    ~VectorNavDataLogger() override;
    /// @brief Copy constructor
    VectorNavDataLogger(const VectorNavDataLogger&) = delete;
    /// @brief Move constructor
    VectorNavDataLogger(VectorNavDataLogger&&) = delete;
    /// @brief Copy assignment operator
    VectorNavDataLogger& operator=(const VectorNavDataLogger&) = delete;
    /// @brief Move assignment operator
    VectorNavDataLogger& operator=(VectorNavDataLogger&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Function called by the flow executer after finishing to flush out remaining data
    void flush() override;

  private:
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Write Observation to the file
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void writeObservation(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Flag to write the header once
    bool _headerWritten = false;
};

} // namespace NAV
