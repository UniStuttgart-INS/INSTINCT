#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "util/InsMechanization.hpp"
#include "util/InsConstants.hpp"
#include "util/InsGravity.hpp"

NAV::ImuIntegrator::ImuIntegrator(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : Node(name)
{
    if (options.count("Integration Frame"))
    {
        if (options.at("Integration Frame") == "ECEF")
        {
            integrationFrame = IntegrationFrame::ECEF;
        }
        else if (options.at("Integration Frame") == "NED")
        {
            integrationFrame = IntegrationFrame::NED;
        }
    }
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
    /// State Data at the time tₖ₋₁
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

    /// IMU Observation at the time tₖ₋₂
    auto& imuObs__t2 = prevObs.at(1);
    /// IMU Observation at the time tₖ₋₁
    auto& imuObs__t1 = prevObs.at(0);

    /// State Data at the time tₖ₋₂
    auto& stateData__t2 = prevStates.at(1);

    /// Result State Data at the time tₖ
    auto stateData__t0 = std::make_shared<StateData>();

    /// tₖ₋₂ Time at prior to previous epoch
    const InsTime& time__t2 = imuObs__t2->insTime.value();
    /// tₖ₋₁ Time at previous epoch
    const InsTime& time__t1 = imuObs__t1->insTime.value();
    /// tₖ Current Time
    const InsTime& time__t0 = imuObs__t0->insTime.value();

    // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    const long double timeDifferenceSec__t1 = (time__t1 - time__t2).count();
    // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    const long double timeDifferenceSec__t0 = (time__t0 - time__t1).count();

    /// ω_ip_p (tₖ₋₁) Angular velocity in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& angularVelocity_ip_p__t1 = imuObs__t1->gyroUncompXYZ.value();
    /// ω_ip_p (tₖ) Angular velocity in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    const Eigen::Vector3d& angularVelocity_ip_p__t0 = imuObs__t0->gyroUncompXYZ.value();

    /// a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
    const Eigen::Vector3d& acceleration_p__t1 = imuObs__t1->accelUncompXYZ.value();
    /// a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
    const Eigen::Vector3d& acceleration_p__t0 = imuObs__t0->accelUncompXYZ.value();

    /// v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const auto& velocity_n__t1 = stateData__t1->velocity_n();
    /// v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
    const auto& velocity_n__t2 = stateData__t2->velocity_n();
    /// v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
    const Eigen::Vector3d velocity_e__t2 = stateData__t2->quaternion_n2e() * velocity_n__t2;
    /// v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
    const Eigen::Vector3d velocity_e__t1 = stateData__t1->quaternion_n2e() * velocity_n__t1;
    /// x_e (tₖ₋₂) Position in [m/s], in earth coordinates, at the time tₖ₋₂
    const Eigen::Vector3d position_e__t2 = stateData__t2->positionECEF_WGS84();
    /// x_e (tₖ₋₁) Position in [m/s], in earth coordinates, at the time tₖ₋₁
    const Eigen::Vector3d position_e__t1 = stateData__t1->positionECEF_WGS84();

    /// g_n Gravity vector in [m/s^2], in navigation coordinates
    const Eigen::Vector3d gravity_n__t1(0, 0, gravity::gravityMagnitude_Gleason(stateData__t1->latitude()));
    /// g_e Gravity vector in [m/s^2], in earth coordinates
    const Eigen::Vector3d gravity_e__t1 = stateData__t1->quaternion_n2e() * gravity_n__t1;

    if (integrationFrame == IntegrationFrame::ECEF)
    {
        /// q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_p2e__t2 = stateData__t2->quaternion_b2e() * imuPosition->quatGyro_p2b;
        /// q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_p2e__t1 = stateData__t1->quaternion_b2e() * imuPosition->quatGyro_p2b;

        /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
        const Eigen::Vector3d& angularVelocity_ie_e__t0 = InsConst::angularVelocity_ie_e;

        /// q (tₖ) Quaternion, from platform to earth coordinates, at the current time tₖ
        const Eigen::Quaterniond quaternion_p2e__t0 = updateQuaternion_p2e_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                                       angularVelocity_ip_p__t0, angularVelocity_ip_p__t1,
                                                                                       angularVelocity_ie_e__t0,
                                                                                       quaternion_p2e__t1, quaternion_p2e__t2);

        /// v (tₖ), Velocity in earth coordinates, at the current time tₖ
        const Eigen::Vector3d velocity_e__t0 = updateVelocity_e_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                            acceleration_p__t0, acceleration_p__t1,
                                                                            velocity_e__t2,
                                                                            position_e__t2,
                                                                            gravity_e__t1,
                                                                            quaternion_p2e__t0, quaternion_p2e__t1, quaternion_p2e__t2);

        /// x_e (tₖ) Position in [m/s], in earth coordinates, at the time tₖ
        const Eigen::Vector3d position_e__t0 = position_e__t1 + velocity_e__t1 * timeDifferenceSec__t0;

        // Use same timestamp as IMU message
        stateData__t0->insTime = time__t0;

        // Store velocity in the state
        stateData__t0->velocity_n() = stateData__t1->quaternion_e2n() * velocity_e__t0;
        // Store position in the state. Important to do before using the quaternion_n2e.
        stateData__t0->latLonHeight() = trafo::ecef2llh_WGS84(position_e__t0);

        /// IMU platform orientation
        Eigen::Quaterniond quaternion_b2p = imuPosition->quatGyro_p2b.conjugate();
        /// Quaternion for rotation from earth to navigation frame. Depends on position which was updated before
        Eigen::Quaterniond quaternion_e2n = stateData__t0->quaternion_e2n();
        // Store body to navigation frame quaternion in the state
        stateData__t0->quat_b2n_coeff() = (quaternion_e2n * quaternion_p2e__t0 * quaternion_b2p).coeffs();
    }
    else
    {
        /// ω_ip_p (tₖ₋₁) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ₋₁
        const Eigen::Vector3d& angularVelocity_ip_b__t1 = imuPosition->quatGyro_p2b * angularVelocity_ip_p__t1;
        /// ω_ip_p (tₖ) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ
        const Eigen::Vector3d& angularVelocity_ip_b__t0 = imuPosition->quatGyro_p2b * angularVelocity_ip_p__t0;

        /// a_b (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
        const Eigen::Vector3d& acceleration_b__t1 = imuPosition->quatAccel_p2b * acceleration_p__t1;
        /// a_b (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
        const Eigen::Vector3d& acceleration_b__t0 = imuPosition->quatAccel_p2b * acceleration_p__t0;

        /// q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_b2n__t1 = stateData__t1->quaternion_b2n();
        /// q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_b2n__t2 = stateData__t2->quaternion_b2n();

        /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
        const Eigen::Vector3d angularVelocity_ie_n__t1 = stateData__t1->quaternion_e2n() * InsConst::angularVelocity_ie_e;

        /// EarthRadius[0] is prime vertical radius N, EarthRadius[0] is meridian radius M
        Eigen::Vector2d EarthRadius;
        EarthRadius(0) = InsConst::WGS84_a / std::sqrt(1 - InsConst::WGS84_e_squared * std::pow(std::sin(stateData__t1->latitude()), 2));
        EarthRadius(1) = ((1 - InsConst::WGS84_e_squared) / (InsConst::WGS84_a * InsConst::WGS84_a)) * EarthRadius(0) * EarthRadius(0) * EarthRadius(0);

        /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
        Eigen::Vector3d angularVelocity_en_n__t1;
        angularVelocity_en_n__t1(0) = velocity_n__t1(1) / (EarthRadius(0) + velocity_n__t1(2));
        angularVelocity_en_n__t1(1) = -velocity_n__t1(0) / (EarthRadius(1) + velocity_n__t1(2));
        angularVelocity_en_n__t1(2) = -angularVelocity_en_n__t1(0) * std::tan(stateData__t1->latitude());

        /// q (tₖ) Quaternion, from body to navigation coordinates, at the current time tₖ
        const Eigen::Quaterniond quaternion_b2n__t0 = updateQuaternion_b2n_RungeKutta3(timeDifferenceSec__t0,
                                                                                       timeDifferenceSec__t1,
                                                                                       angularVelocity_ip_b__t0,
                                                                                       angularVelocity_ip_b__t1,
                                                                                       angularVelocity_ie_n__t1,
                                                                                       angularVelocity_en_n__t1,
                                                                                       quaternion_b2n__t1,
                                                                                       quaternion_b2n__t2);

        /// v (tₖ), Velocity in navigation coordinates, at the current time tₖ
        const Eigen::Vector3d velocity_n__t0 = updateVelocity_n_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                            acceleration_b__t0, acceleration_b__t1,
                                                                            velocity_n__t1, velocity_n__t2,
                                                                            gravity_n__t1,
                                                                            angularVelocity_ie_n__t1,
                                                                            angularVelocity_en_n__t1,
                                                                            quaternion_b2n__t0, quaternion_b2n__t1, quaternion_b2n__t2);

        /// x_e (tₖ) Position in [m/s], in earth coordinates, at the time tₖ
        const Eigen::Vector3d position_e__t0 = position_e__t1 + velocity_e__t1 * timeDifferenceSec__t0;

        // Use same timestamp as IMU message
        stateData__t0->insTime = time__t0;

        // Store velocity in the state
        stateData__t0->velocity_n() = velocity_n__t0;
        // Store position in the state. Important to do before using the quaternion_n2e.
        stateData__t0->latLonHeight() = trafo::ecef2llh_WGS84(position_e__t0);

        // Store body to navigation frame quaternion in the state
        stateData__t0->quat_b2n_coeff() = quaternion_b2n__t0.coeffs();
    }

    // Rotate Observation
    prevObs.pop_back();
    prevObs.push_front(imuObs__t0);

    invokeCallbacks(stateData__t0);
}