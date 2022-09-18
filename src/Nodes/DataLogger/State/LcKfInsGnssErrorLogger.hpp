/// @file LcKfInsGnssErrorLogger.hpp
/// @brief Data Logger for INS/GNSS LCKF Errors
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-22

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataLogger/Protocol/FileWriter.hpp"
#include "Nodes/DataLogger/Protocol/CommonLog.hpp"

namespace NAV
{
class NodeData;

/// Data Logger for INS/GNSS LCKF Errors
class LcKfInsGnssErrorLogger : public Node, public FileWriter, public CommonLog
{
  public:
    /// @brief Default constructor
    LcKfInsGnssErrorLogger();
    /// @brief Destructor
    ~LcKfInsGnssErrorLogger() override;
    /// @brief Copy constructor
    LcKfInsGnssErrorLogger(const LcKfInsGnssErrorLogger&) = delete;
    /// @brief Move constructor
    LcKfInsGnssErrorLogger(LcKfInsGnssErrorLogger&&) = delete;
    /// @brief Copy assignment operator
    LcKfInsGnssErrorLogger& operator=(const LcKfInsGnssErrorLogger&) = delete;
    /// @brief Move assignment operator
    LcKfInsGnssErrorLogger& operator=(LcKfInsGnssErrorLogger&&) = delete;

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
};

} // namespace NAV
