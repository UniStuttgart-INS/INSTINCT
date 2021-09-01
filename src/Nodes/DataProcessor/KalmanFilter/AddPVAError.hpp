/// @file AddPVAError.hpp
/// @brief Adds the PVA Error to the PosVelAtt state
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-09-01

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Eigen.hpp"

#include "NodeData/State/PVAError.hpp"

namespace NAV
{
class AddPVAError : public Node
{
  public:
    /// @brief Default constructor
    AddPVAError();
    /// @brief Destructor
    ~AddPVAError() override;
    /// @brief Copy constructor
    AddPVAError(const AddPVAError&) = delete;
    /// @brief Move constructor
    AddPVAError(AddPVAError&&) = delete;
    /// @brief Copy assignment operator
    AddPVAError& operator=(const AddPVAError&) = delete;
    /// @brief Move assignment operator
    AddPVAError& operator=(AddPVAError&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

  private:
    constexpr static size_t OutputPortIndex_PosVelAtt = 0; ///< @brief Flow (PosVelAtt)

    /// @brief Receive function for PosVelAtt
    /// @param[in] nodeData PosVelAtt received
    /// @param[in] linkId Id of the link over which the data is received
    void recvPosVelAtt(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive function for PVAError
    /// @param[in] nodeData PVAError received
    /// @param[in] linkId Id of the link over which the data is received
    void recvPVAError(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// Pointer to the most recent PVA error
    std::shared_ptr<PVAError> pvaError = nullptr;
};

} // namespace NAV
