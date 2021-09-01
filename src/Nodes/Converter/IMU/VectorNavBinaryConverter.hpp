/// @file VectorNavBinaryConverter.hpp
/// @brief Converts VectorNavBinaryOutput
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-09

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/State/PosVelAtt.hpp"

#include <array>
#include <memory>

namespace NAV
{
/// Converts VectorNavBinaryOutput
class VectorNavBinaryConverter : public Node
{
  public:
    /// @brief Default constructor
    VectorNavBinaryConverter();
    /// @brief Destructor
    ~VectorNavBinaryConverter() override;
    /// @brief Copy constructor
    VectorNavBinaryConverter(const VectorNavBinaryConverter&) = delete;
    /// @brief Move constructor
    VectorNavBinaryConverter(VectorNavBinaryConverter&&) = delete;
    /// @brief Copy assignment operator
    VectorNavBinaryConverter& operator=(const VectorNavBinaryConverter&) = delete;
    /// @brief Move assignment operator
    VectorNavBinaryConverter& operator=(VectorNavBinaryConverter&&) = delete;

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

  private:
    constexpr static size_t OutputPortIndex_Converted = 0;            ///< @brief Flow
    constexpr static size_t InputPortIndex_VectorNavBinaryOutput = 0; ///< @brief Flow (VectorNavBinaryOutput)

    /// Enum specifying the type of the output message
    enum OutputType
    {
        OutputType_ImuObsWDelta, ///< Extract ImuObsWDelta data
        OutputType_PosVelAtt,    ///< Extract PosVelAtt data
    };

    /// The selected output type in the GUI
    OutputType outputType = OutputType_ImuObsWDelta;

    /// @brief Converts the VectorNavBinaryOutput observation to the selected message type
    /// @param[in] nodeData VectorNavBinaryOutput to process
    /// @param[in] linkId Id of the link over which the data is received
    void receiveObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Converts the VectorNavBinaryOutput to a ImuObsWDelta observation
    /// @param[in] nodeData VectorNavBinaryOutput to process
    /// @return The converted data
    std::shared_ptr<ImuObsWDelta> convert2ImuObsWDelta(const std::shared_ptr<VectorNavBinaryOutput>& vnObs);

    /// @brief Converts the VectorNavBinaryOutput to a PosVelAtt observation
    /// @param[in] nodeData VectorNavBinaryOutput to process
    /// @return The converted data
    std::shared_ptr<PosVelAtt> convert2PosVelAtt(const std::shared_ptr<VectorNavBinaryOutput>& vnObs);
};

} // namespace NAV
