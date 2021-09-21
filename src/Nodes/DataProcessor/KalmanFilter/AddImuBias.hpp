/// @file AddImuBias.hpp
/// @brief Adds the Bias Error to the Imu observation
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-31

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Eigen.hpp"

#include "NodeData/State/ImuBiases.hpp"

namespace NAV
{
class AddImuBias : public Node
{
  public:
    /// @brief Default constructor
    AddImuBias();
    /// @brief Destructor
    ~AddImuBias() override;
    /// @brief Copy constructor
    AddImuBias(const AddImuBias&) = delete;
    /// @brief Move constructor
    AddImuBias(AddImuBias&&) = delete;
    /// @brief Copy assignment operator
    AddImuBias& operator=(const AddImuBias&) = delete;
    /// @brief Move assignment operator
    AddImuBias& operator=(AddImuBias&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

  private:
    constexpr static size_t OutputPortIndex_ImuObs = 0; ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive function for ImuObs
    /// @param[in] nodeData Observation received
    /// @param[in] linkId Id of the link over which the data is received
    void recvImuObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive function for ImuBiases
    /// @param[in] nodeData Observation received
    /// @param[in] linkId Id of the link over which the data is received
    void recvImuBiases(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// Pointer to the most recent imu biases
    ImuBiases imuBiases;
};

} // namespace NAV
