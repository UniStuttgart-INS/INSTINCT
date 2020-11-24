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

void NAV::ImuIntegrator::initialize()
{
    if (incomingLinks[1].first.lock()->type() != State().type())
    {
        LOG_CRITICAL("Please only connect State Nodes to port 1 of node {}", getName());
    }
    stateNode = std::static_pointer_cast<State>(incomingLinks[1].first.lock());
    stateNodeOutputPortIndex = incomingLinks[1].second;
}

void NAV::ImuIntegrator::deinitialize()
{
    stateNode = nullptr;
}

void NAV::ImuIntegrator::integrateObservation(std::shared_ptr<NAV::ImuObs>& imuObs__t0)
{
    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = imuObs__t0->imuPos;

    /// State Data at the time tₖ₋₁
    auto stateData__t1 = std::static_pointer_cast<StateData>(stateNode->requestOutputData(stateNodeOutputPortIndex));

    // No inital state available yet
    if (stateData__t1 == nullptr || imuObs__t2 == nullptr)
    {
        // Rotate Observation
        imuObs__t2 = imuObs__t1;
        imuObs__t1 = imuObs__t0;
        return;
    }

    // First state available
    if (stateData__t2 == nullptr)
    {
        // Rotate StateData
        stateData__t2 = stateData__t1;

        // Rotate Observation
        imuObs__t2 = imuObs__t1;
        imuObs__t1 = imuObs__t0;

        // It is an initial state, not a previous one
        if (!stateData__t1->insTime.has_value())
        {
            stateData__t1->insTime = imuObs__t1->insTime.value();
        }

        invokeCallbacks(stateData__t1);
        return;
    }

    /// Initial State
    auto stateData__init = std::static_pointer_cast<State>(stateNode)->initialState;

    /// Result State Data at the time tₖ
    auto stateData__t0 = std::make_shared<StateData>();

    // Use same timestamp as IMU message
    stateData__t0->insTime = imuObs__t0->insTime.value();

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
    const Vector3d<Platform>& angularVelocity_ip_p__t1 = imuObs__t1->gyroUncompXYZ.value();
    /// ω_ip_p (tₖ) Angular velocity in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    const Vector3d<Platform>& angularVelocity_ip_p__t0 = imuObs__t0->gyroUncompXYZ.value();

    /// a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
    const Vector3d<Platform>& acceleration_p__t1 = imuObs__t1->accelUncompXYZ.value();
    /// a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
    const Vector3d<Platform>& acceleration_p__t0 = imuObs__t0->accelUncompXYZ.value();

    /// v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
    const Vector3d<Navigation>& velocity_n__t1 = stateData__t1->velocity_n();
    /// v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
    const Vector3d<Navigation>& velocity_n__t2 = stateData__t2->velocity_n();
    /// v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
    const Vector3d<Earth> velocity_e__t2 = stateData__t2->quaternion_en() * velocity_n__t2;
    /// v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
    const Vector3d<Earth> velocity_e__t1 = stateData__t1->quaternion_en() * velocity_n__t1;
    /// x_e (tₖ₋₂) Position in [m], in ECEF coordinates, at the time tₖ₋₂
    const Vector3d<Earth> position_e__t2 = stateData__t2->position_ecef();
    /// x_e (tₖ₋₁) Position in [m], in ECEF coordinates, at the time tₖ₋₁
    const Vector3d<Earth> position_e__t1 = stateData__t1->position_ecef();

    /// g_n Gravity vector in [m/s^2], in navigation coordinates
    const Vector3d<Navigation> gravity_n__t1(0, 0, gravity::gravityMagnitude_Gleason(stateData__t1->latitude()));
    /// g_e Gravity vector in [m/s^2], in earth coordinates
    const Vector3d<Earth> gravity_e__t1 = stateData__t1->quaternion_en() * gravity_n__t1;

    if (integrationFrame == IntegrationFrame::ECEF)
    {
        /// q (tₖ₋₂) Quaternion, from gyro platform to earth coordinates, at the time tₖ₋₂
        const Quaterniond<Earth, Platform> quaternion_gyro_ep__t2 = stateData__t2->quaternion_eb() * imuPosition.quatGyro_bp();
        /// q (tₖ₋₁) Quaternion, from gyro platform to earth coordinates, at the time tₖ₋₁
        const Quaterniond<Earth, Platform> quaternion_gyro_ep__t1 = stateData__t1->quaternion_eb() * imuPosition.quatGyro_bp();

        /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
        const Vector3d<Earth>& angularVelocity_ie_e__t0 = InsConst::angularVelocity_ie_e;

        /// q (tₖ) Quaternion, from platform to earth coordinates, at the current time tₖ
        const Quaterniond<Earth, Platform> quaternion_gyro_ep__t0 = updateQuaternion_ep_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                                                    angularVelocity_ip_p__t0, angularVelocity_ip_p__t1,
                                                                                                    angularVelocity_ie_e__t0,
                                                                                                    quaternion_gyro_ep__t1, quaternion_gyro_ep__t2);

        /// q (tₖ₋₂) Quaternion, from accel platform to earth coordinates, at the time tₖ₋₂
        const Quaterniond<Earth, Platform> quaternion_accel_ep__t2 = stateData__t2->quaternion_eb() * imuPosition.quatAccel_bp();
        /// q (tₖ₋₁) Quaternion, from accel platform to earth coordinates, at the time tₖ₋₁
        const Quaterniond<Earth, Platform> quaternion_accel_ep__t1 = stateData__t1->quaternion_eb() * imuPosition.quatAccel_bp();
        /// q (tₖ) Quaternion, from accel platform to earth coordinates, at the time tₖ
        const Quaterniond<Earth, Platform> quaternion_accel_ep__t0 = quaternion_gyro_ep__t0 * imuPosition.quatGyro_pb() * imuPosition.quatAccel_bp();

        /// v (tₖ), Velocity in [m/s], in earth coordinates, at the current time tₖ
        const Vector3d<Earth> velocity_e__t0 = updateVelocity_e_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                            acceleration_p__t0, acceleration_p__t1,
                                                                            velocity_e__t2,
                                                                            position_e__t2,
                                                                            gravity_e__t1,
                                                                            quaternion_accel_ep__t0,
                                                                            quaternion_accel_ep__t1,
                                                                            quaternion_accel_ep__t2);

        /// x_e (tₖ) Position in [m], in earth coordinates, at the time tₖ
        const Vector3d<Earth>& position_e__t0 = updatePosition_e(timeDifferenceSec__t0, position_e__t1, velocity_e__t1);

        // Store velocity in the state
        stateData__t0->velocity_n() = stateData__t1->quaternion_ne() * velocity_e__t0;
        // Store position in the state. Important to do before using the quaternion_en.
        stateData__t0->position_ecef() = position_e__t0;

        /// Quaternion for rotation from earth to navigation frame. Depends on position which was updated before
        Quaterniond<Navigation, Earth> quaternion_ne__t0 = stateData__t0->quaternion_ne();
        // Store body to navigation frame quaternion in the state
        stateData__t0->quaternion_nb() = quaternion_ne__t0 * quaternion_gyro_ep__t0 * imuPosition.quatGyro_pb();
    }
    else if (integrationFrame == IntegrationFrame::NED)
    {
        /// ω_ip_b (tₖ₋₁) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ₋₁
        const Vector3d<Body> angularVelocity_ip_b__t1 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t1;
        /// ω_ip_b (tₖ) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ
        const Vector3d<Body> angularVelocity_ip_b__t0 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t0;

        /// a_b (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
        const Vector3d<Body> acceleration_b__t1 = imuPosition.quatAccel_bp() * acceleration_p__t1;
        /// a_b (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
        const Vector3d<Body> acceleration_b__t0 = imuPosition.quatAccel_bp() * acceleration_p__t0;

        /// q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
        const Quaterniond<Navigation, Body> quaternion_nb__t1 = stateData__t1->quaternion_nb();
        /// q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
        const Quaterniond<Navigation, Body> quaternion_nb__t2 = stateData__t2->quaternion_nb();

        /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
        Vector3d<Navigation> angularVelocity_ie_n__t1 = stateData__t1->quaternion_ne() * InsConst::angularVelocity_ie_e;

        /// North/South (meridian) earth radius [m]
        double R_N = earthRadius_N(InsConst::WGS84_a, InsConst::WGS84_e_squared, stateData__t1->latitude());
        /// East/West (prime vertical) earth radius [m]
        double R_E = earthRadius_E(InsConst::WGS84_a, InsConst::WGS84_e_squared, stateData__t1->latitude());

        /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
        Vector3d<Navigation> angularVelocity_en_n__t1 = transportRate(stateData__t1->latLonAlt(), velocity_n__t1, R_N, R_E);

        /// q (tₖ) Quaternion, from body to navigation coordinates, at the current time tₖ
        Quaterniond<Navigation, Body> quaternion_nb__t0 = updateQuaternion_nb_RungeKutta3(timeDifferenceSec__t0,
                                                                                          timeDifferenceSec__t1,
                                                                                          angularVelocity_ip_b__t0,
                                                                                          angularVelocity_ip_b__t1,
                                                                                          angularVelocity_ie_n__t1,
                                                                                          angularVelocity_en_n__t1,
                                                                                          quaternion_nb__t1,
                                                                                          quaternion_nb__t2);

        /// v (tₖ), Velocity in navigation coordinates, at the current time tₖ
        Vector3d<Navigation> velocity_n__t0 = updateVelocity_n_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                           acceleration_b__t0, acceleration_b__t1,
                                                                           velocity_n__t1, velocity_n__t2,
                                                                           gravity_n__t1,
                                                                           angularVelocity_ie_n__t1,
                                                                           angularVelocity_en_n__t1,
                                                                           quaternion_nb__t0, quaternion_nb__t1, quaternion_nb__t2);

        /// [x_n, x_e, x_d] (tₖ₋₁) Position NED in [m] at the time tₖ₋₁
        Vector3d<Navigation> position_n__t1 = trafo::ecef2ned(position_e__t1, stateData__init->latLonAlt());

        /// [x_n, x_e, x_d] (tₖ) Position NED in [m] at the time tₖ
        Vector3d<Navigation> position_n__t0 = updatePosition_n(timeDifferenceSec__t0, position_n__t1, velocity_n__t1);

        /// x_e (tₖ) Position in [m], in ECEF coordinates, at the time tₖ
        Vector3d<Earth> position_e__t0 = trafo::ned2ecef(position_n__t0, stateData__init->latLonAlt());

        /// Latitude, Longitude and Altitude in [rad, rad, m], at the current time tₖ (see Gleason eq. 6.18 - 6.20)
        // Vector3d<LLA> latLonAlt__t0 = updatePosition_n(timeDifferenceSec__t0, stateData__t1->latLonAlt(),
        //                                                     velocity_n__t1, R_N, R_E);

        // Store velocity in the state
        stateData__t0->velocity_n() = velocity_n__t0;
        // Store position in the state
        stateData__t0->position_ecef() = position_e__t0;

        // Store body to navigation frame quaternion in the state
        stateData__t0->quaternion_nb() = quaternion_nb__t0;
    }

    // Rotate StateData
    stateData__t2 = stateData__t1;

    // Rotate Observation
    imuObs__t2 = imuObs__t1;
    imuObs__t1 = imuObs__t0;

    invokeCallbacks(stateData__t0);
}