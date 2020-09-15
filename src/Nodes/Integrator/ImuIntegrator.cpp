#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "util/InsMechanization.hpp"
#include "util/InsConstants.hpp"

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
    if (prevObs.empty())
    {
        prevObs.push_back(imuObs__t0);
        prevObs.push_back(imuObs__t0);
        prevStates.push_back(stateData__t1);
        prevStates.push_back(stateData__t1);
    }

    // Rotate Data
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

    // q (tₖ) Quaternion, from platform to earth coordinates, at the current time tₖ
    Eigen::Quaterniond quaternion_p2e__t0 = updateQuaternionsRungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                         angularVelocity_ip__t0, angularVelocity_ip__t1,
                                                                         angularVelocity_ie__t0,
                                                                         quaternion_p2e__t1, quaternion_p2e__t2);

    auto stateData__t0 = std::make_shared<StateData>();

    auto quaternion_b2p = imuPosition->quatGyro_p2b.conjugate();
    // TODO: update position, and then use stateData__t0
    auto quaternion_e2n = stateData__t1->quaternion_n2e().conjugate();
    stateData__t0->quat_b2n_coeff() = (quaternion_e2n * quaternion_p2e__t0 * quaternion_b2p).coeffs();

    // Rotate Observation
    prevObs.pop_back();
    prevObs.push_front(imuObs__t0);

    invokeCallbacks(stateData__t0);
}