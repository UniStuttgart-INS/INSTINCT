/**
 * @file ImuIntegrator.hpp
 * @brief Imu Data Integrator
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-23
 */

#pragma once

#include "../Integrator.hpp"

namespace NAV
{
/// Plots VectorNav Imu Data
class ImuIntegrator : public Integrator
{
  public:
    /**
     * @brief Construct a new Imu Integrator object
     * 
     * @param[in] name Name of the Node
     * @param[in, out] options Program options string list
     */
    ImuIntegrator(const std::string& name, std::deque<std::string>& options);

    /// Default Destructor
    ~ImuIntegrator() override;

    ImuIntegrator(const ImuIntegrator&) = delete;            ///< Copy constructor
    ImuIntegrator(ImuIntegrator&&) = delete;                 ///< Move constructor
    ImuIntegrator& operator=(const ImuIntegrator&) = delete; ///< Copy assignment operator
    ImuIntegrator& operator=(ImuIntegrator&&) = delete;      ///< Move assignment operator

    /**
     * @brief Integrates ImuObs data
     * 
     * @param[in] observation The received observation
     * @param[in] userData User data specified when registering the callback
     * @retval NavStatus Indicates whether the Integration was successfull
     */
    static NavStatus integrateImuObs(std::shared_ptr<NodeData> observation, std::shared_ptr<Node> userData);

  private:
};

} // namespace NAV
