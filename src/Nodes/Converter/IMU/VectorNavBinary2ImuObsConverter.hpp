/// @file VectorNavBinary2ImuObsConverter.hpp
/// @brief Converts VectorNavBinaryOutput to ImuObsWDelta
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-09

#pragma once

#include "internal/Node/Node.hpp"

#include <array>

namespace NAV
{
/// Converts VectorNavBinaryOutput to ImuObsWDelta
class VectorNavBinary2ImuObsConverter : public Node
{
  public:
    /// @brief Default constructor
    VectorNavBinary2ImuObsConverter();
    /// @brief Destructor
    ~VectorNavBinary2ImuObsConverter() override;
    /// @brief Copy constructor
    VectorNavBinary2ImuObsConverter(const VectorNavBinary2ImuObsConverter&) = delete;
    /// @brief Move constructor
    VectorNavBinary2ImuObsConverter(VectorNavBinary2ImuObsConverter&&) = delete;
    /// @brief Copy assignment operator
    VectorNavBinary2ImuObsConverter& operator=(const VectorNavBinary2ImuObsConverter&) = delete;
    /// @brief Move assignment operator
    VectorNavBinary2ImuObsConverter& operator=(VectorNavBinary2ImuObsConverter&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

  private:
    constexpr static size_t OutputPortIndex_ImuObsWDelta = 0;         ///< @brief Flow (ImuObsWDelta)
    constexpr static size_t InputPortIndex_VectorNavBinaryOutput = 0; ///< @brief Flow (VectorNavBinaryOutput)

    /// @brief Converts the VectorNavBinaryOutput observation to the ImuObsWDelta observation
    /// @param[in] nodeData VectorNavBinaryOutput to process
    /// @param[in] linkId Id of the link over which the data is received
    void convertObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);
};

} // namespace NAV
