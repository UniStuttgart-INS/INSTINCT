/**
 * @file Integrator.hpp
 * @brief Imu Integrator class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-24
 */

#pragma once

#include "../Integrator.hpp"

namespace NAV
{
/// Abstract Integrator class
class ImuIntegrator : public Integrator
{
  protected:
    /**
     * @brief Construct a new Integrator object
     * 
     * @param[in] name Name of the Integrator
     */
    ImuIntegrator(std::string name);

    /// Default destructor
    ~ImuIntegrator();

    /**
     * @brief Initialize the Integrator
     * 
     * @retval NavStatus Indicates whether initialization was successfull
     */
    NavStatus initialize() override;

    /**
     * @brief Deinitialize the Integrator
     * 
     * @retval NavStatus Indicates whether deinitialization was successfull
     */
    NavStatus deinitialize() override;

    /**
     * @brief Integrate an Imu Observation
     * 
     * @param[in] observation The received observation
     * @param[in] userData User data specified when registering the callback
     * @retval NavStatus Indicates whether the write was successfull.
     */
    static NavStatus integrateImuObs(std::shared_ptr<void> observation, std::shared_ptr<void> userData);
};

} // namespace NAV
