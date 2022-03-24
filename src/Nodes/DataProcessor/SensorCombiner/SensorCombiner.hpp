/// @file SensorCombiner.hpp
/// @brief Combines signals of sensors that provide the same signal-type to one signal
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-03-24

#pragma once

#include "internal/Node/Node.hpp"

namespace NAV
{
/// @brief Combines signals of sensors that provide the same signal-type to one signal
class SensorCombiner : public Node
{
  public:
    /// @brief Default constructor
    SensorCombiner();
    /// @brief Destructor
    ~SensorCombiner() override;
    /// @brief Copy constructor
    SensorCombiner(const SensorCombiner&) = delete;
    /// @brief Move constructor
    SensorCombiner(SensorCombiner&&) = delete;
    /// @brief Copy assignment operator
    SensorCombiner& operator=(const SensorCombiner&) = delete;
    /// @brief Move assignment operator
    SensorCombiner& operator=(SensorCombiner&&) = delete;

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

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_COMBINED_SIGNAL = 0; ///< @brief Flow (InertialNavSol)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the signal at the time tₖ
    /// @param[in] nodeData Signal to process
    /// @param[in] linkId Id of the link over which the data is received
    void recvSignal(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Combines the signals
    void combineSignals();
};

} // namespace NAV
