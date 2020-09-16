#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "util/InsMechanization.hpp"
#include "util/InsConstants.hpp"
#include "util/InsGravity.hpp"

NAV::ImuIntegrator::ImuIntegrator(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : Node(name)
{
}

void NAV::ImuIntegrator::integrateObservation(std::shared_ptr<NAV::ImuObs>& imuObs__t0)
{
    // Get the IMU Position information
    const auto& imuNode = incomingLinks[1].first.lock();
    auto& imuPortIndex = incomingLinks[1].second;
    auto imuPosition = std::static_pointer_cast<ImuPos>(imuNode->requestOutputData(imuPortIndex));

    // Get the current state data
    const auto& stateNode = incomingLinks[2].first.lock();
    auto& statePortIndex = incomingLinks[2].second;
    auto stateData__t1 = std::static_pointer_cast<StateData>(stateNode->requestOutputData(statePortIndex));

    // Fill if empty
    if (prevObs.size() < 2)
    {
        prevObs.push_front(imuObs__t0);
        prevStates.push_front(stateData__t1);
        return;
    }

    // Rotate State Data
    prevStates.pop_back();
    prevStates.push_front(stateData__t1);

    /// tₖ₋₂ Time at prior to previous epoch
    const auto& time__t2 = prevObs.at(1)->insTime.value();
    /// tₖ₋₁ Time at previous epoch
    const auto& time__t1 = prevObs.at(0)->insTime.value();
    /// tₖ Current Time
    const auto& time__t0 = imuObs__t0->insTime.value();

    /// ω_ip_p (tₖ₋₁) Angular velocity in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    const auto& angularVelocity_ip__t1 = prevObs.at(0)->gyroUncompXYZ.value();
    /// ω_ip_p (tₖ) Angular velocity in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    const auto& angularVelocity_ip__t0 = imuObs__t0->gyroUncompXYZ.value();

    // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    auto timeDifferenceSec__t1 = (time__t1 - time__t2).count();
    // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    auto timeDifferenceSec__t0 = (time__t0 - time__t1).count();

    /// q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
    Eigen::Quaterniond quaternion_p2e__t2 = prevStates.at(1)->quaternion_b2e() * imuPosition->quatGyro_p2b;
    /// q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
    Eigen::Quaterniond quaternion_p2e__t1 = prevStates.at(0)->quaternion_b2e() * imuPosition->quatGyro_p2b;

    /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
    const Eigen::Vector3d& angularVelocity_ie__t0 = InsConst::angularVelocity_ie_e;

    /// q (tₖ) Quaternion, from platform to earth coordinates, at the current time tₖ
    const Eigen::Quaterniond quaternion_p2e__t0 = updateQuaternionsRungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                               angularVelocity_ip__t0, angularVelocity_ip__t1,
                                                                               angularVelocity_ie__t0,
                                                                               quaternion_p2e__t1, quaternion_p2e__t2);

    /// a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
    const auto& acceleration_p__t1 = prevObs.at(0)->accelUncompXYZ.value();
    /// a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
    const auto& acceleration_p__t0 = imuObs__t0->accelUncompXYZ.value();

    /// v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
    const auto& velocity_e__t2 = prevStates.at(1)->velocity_e();
    /// v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
    const auto& velocity_e__t1 = prevStates.at(0)->velocity_e();
    /// x_e (tₖ₋₂) Position in [m/s], in earth coordinates, at the time tₖ₋₂
    const auto& position_e__t2 = prevStates.at(1)->positionECEF_WGS84();
    /// x_e (tₖ₋₁) Position in [m/s], in earth coordinates, at the time tₖ₋₁
    const auto& position_e__t1 = prevStates.at(0)->positionECEF_WGS84();

    /// g_e Gravity vector in [m/s^2], in earth coordinates
    const auto& gravity_e__t1 = Eigen::Vector3d(0, 0, gravity::gravityMagnitude_Gleason(prevStates.at(0)->latitude()));

    /// v (tₖ), Velocity in earth coordinates, at the current time tₖ
    const Eigen::Vector3d velocity_e__t0 = updateVelocityRungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                     acceleration_p__t0, acceleration_p__t1,
                                                                     velocity_e__t2,
                                                                     position_e__t2,
                                                                     gravity_e__t1,
                                                                     quaternion_p2e__t0, quaternion_p2e__t1, quaternion_p2e__t2);

    /// x_e (tₖ) Position in [m/s], in earth coordinates, at the time tₖ
    const Eigen::Vector3d position_e__t0 = position_e__t1 + velocity_e__t1 * timeDifferenceSec__t0;

    /// New State Data object to store the results and to invoke callbacks with
    auto stateData__t0 = std::make_shared<StateData>();
    // Use same timestamp as IMU message
    stateData__t0->insTime = time__t0;

    // Store velocity in the state
    stateData__t0->velocity_e() = velocity_e__t0;
    // Store position in the state. Important to do before using the quaternion_n2e.
    stateData__t0->latLonHeight() = trafo::ecef2llh_WGS84(position_e__t0);

    /// IMU platform orientation
    auto quaternion_b2p = imuPosition->quatGyro_p2b.conjugate();
    /// Quaternion for rotation from earth to navigation frame. Depends on position which was updated before
    auto quaternion_e2n = stateData__t0->quaternion_n2e().conjugate();
    // Store body to navigation frame quaternion in the state
    stateData__t0->quat_b2n_coeff() = (quaternion_e2n * quaternion_p2e__t0 * quaternion_b2p).coeffs();

    // Rotate Observation
    prevObs.pop_back();
    prevObs.push_front(imuObs__t0);

    invokeCallbacks(stateData__t0);
}