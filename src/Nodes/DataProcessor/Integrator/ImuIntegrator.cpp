#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "util/InsMechanization.hpp"
#include "util/InsConstants.hpp"
#include "util/InsGravity.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::ImuIntegrator::ImuIntegrator()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuIntegrator::integrateObservation);
    nm::CreateInputPin(this, "Position ECEF", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Velocity NED", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Quaternion nb", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
}

NAV::ImuIntegrator::~ImuIntegrator()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuIntegrator::typeStatic()
{
    return "ImuIntegrator";
}

std::string NAV::ImuIntegrator::type() const
{
    return typeStatic();
}

std::string NAV::ImuIntegrator::category()
{
    return "Data Processor";
}

void NAV::ImuIntegrator::guiConfig()
{
    if (ImGui::Combo("Integration Frame", reinterpret_cast<int*>(&integrationFrame), "ECEF\0NED\0\0"))
    {
        LOG_DEBUG("{}: Integration Frame changed to {}", nameId(), integrationFrame ? "NED" : "ECEF");
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::ImuIntegrator::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["integrationFrame"] = integrationFrame;

    return j;
}

void NAV::ImuIntegrator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("integrationFrame"))
    {
        integrationFrame = static_cast<IntegrationFrame>(j.at("integrationFrame").get<int>());
    }
}

bool NAV::ImuIntegrator::onCreateLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        size_t endPinIndex = pinIndexFromId(endPin->id);

        int64_t rows = 3;
        int64_t cols = 1;

        if (endPinIndex == InputPortIndex_Quaternion)
        {
            rows = 4;
        }

        if (endPinIndex == InputPortIndex_Position
            || endPinIndex == InputPortIndex_Velocity
            || endPinIndex == InputPortIndex_Quaternion)
        {
            if (startPin->dataIdentifier.front() == "Eigen::MatrixXd")
            {
                if (const auto* pval = std::get_if<void*>(&startPin->data))
                {
                    if (auto* mat = static_cast<Eigen::MatrixXd*>(*pval))
                    {
                        if (mat->rows() == rows && mat->cols() == cols)
                        {
                            return true;
                        }
                        else
                        {
                            LOG_ERROR("{}: The Matrix needs to have the size {}x{}", nameId(), rows, cols);
                        }
                    }
                }
            }
            else if (startPin->dataIdentifier.front() == "BlockMatrix")
            {
                if (const auto* pval = std::get_if<void*>(&startPin->data))
                {
                    if (auto* block = static_cast<BlockMatrix*>(*pval))
                    {
                        auto mat = (*block)();
                        if (mat.rows() == rows && mat.cols() == cols)
                        {
                            return true;
                        }
                        else
                        {
                            LOG_ERROR("{}: The Matrix needs to have the size {}x{}", nameId(), rows, cols);
                        }
                    }
                }
            }

            return false;
        }
    }

    return true;
}

bool NAV::ImuIntegrator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    imuObs__t1 = nullptr;
    imuObs__t2 = nullptr;

    posVelAtt__t2 = nullptr;
    posVelAtt__init = nullptr;

    return true;
}

void NAV::ImuIntegrator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::ImuIntegrator::getCurrentPosition(Eigen::Vector3d& position)
{
    if (Pin* connectedPin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
    {
        if (connectedPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* currentPosition = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
            {
                position = *currentPosition;
                return true;
            }
        }
        else // (connectedPin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* currentPosition = getInputValue<BlockMatrix>(InputPortIndex_Position))
            {
                position = (*currentPosition)();
                return true;
            }
        }
    }

    return false;
}

void NAV::ImuIntegrator::setCurrentPosition(const Eigen::Vector3d& position)
{
    if (Pin* connectedPin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
    {
        if (connectedPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* currentPosition = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
            {
                *currentPosition = position;
                notifyInputValueChanged(InputPortIndex_Position);
            }
        }
        else // (connectedPin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* currentPosition = getInputValue<BlockMatrix>(InputPortIndex_Position))
            {
                (*currentPosition)() = position;
                notifyInputValueChanged(InputPortIndex_Position);
            }
        }
    }
}

bool NAV::ImuIntegrator::getCurrentVelocity(Eigen::Vector3d& velocity)
{
    if (Pin* connectedPin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Velocity).id))
    {
        if (connectedPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* currentVelocity = getInputValue<Eigen::MatrixXd>(InputPortIndex_Velocity))
            {
                velocity = *currentVelocity;
                return true;
            }
        }
        else // (connectedPin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* currentVelocity = getInputValue<BlockMatrix>(InputPortIndex_Velocity))
            {
                velocity = (*currentVelocity)();
                return true;
            }
        }
    }

    return false;
}

void NAV::ImuIntegrator::setCurrentVelocity(const Eigen::Vector3d& velocity)
{
    if (Pin* connectedPin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Velocity).id))
    {
        if (connectedPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* currentVelocity = getInputValue<Eigen::MatrixXd>(InputPortIndex_Velocity))
            {
                *currentVelocity = velocity;
                notifyInputValueChanged(InputPortIndex_Velocity);
            }
        }
        else // (connectedPin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* currentVelocity = getInputValue<BlockMatrix>(InputPortIndex_Velocity))
            {
                (*currentVelocity)() = velocity;
                notifyInputValueChanged(InputPortIndex_Velocity);
            }
        }
    }
}

bool NAV::ImuIntegrator::getCurrentQuaternion_nb(Eigen::Quaterniond& quaternion_nb)
{
    if (Pin* connectedPin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Quaternion).id))
    {
        if (connectedPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* currentQuaternionCoeffs = getInputValue<Eigen::MatrixXd>(InputPortIndex_Quaternion))
            {
                quaternion_nb = Eigen::Quaterniond((*currentQuaternionCoeffs)(0, 0),  // w
                                                   (*currentQuaternionCoeffs)(1, 0),  // x
                                                   (*currentQuaternionCoeffs)(2, 0),  // y
                                                   (*currentQuaternionCoeffs)(3, 0)); // z
                return true;
            }
        }
        else // (connectedPin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* currentQuaternionCoeffs = getInputValue<BlockMatrix>(InputPortIndex_Quaternion))
            {
                auto block = (*currentQuaternionCoeffs)();
                quaternion_nb = Eigen::Quaterniond(block(0, 0),  // w
                                                   block(1, 0),  // x
                                                   block(2, 0),  // y
                                                   block(3, 0)); // z
                return true;
            }
        }
    }

    return false;
}

void NAV::ImuIntegrator::setCurrentQuaternion_nb(const Eigen::Quaterniond& quaternion_nb)
{
    if (Pin* connectedPin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Quaternion).id))
    {
        if (connectedPin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* currentQuaternionCoeffs = getInputValue<Eigen::MatrixXd>(InputPortIndex_Quaternion))
            {
                (*currentQuaternionCoeffs)(0, 0) = quaternion_nb.w();
                (*currentQuaternionCoeffs)(1, 0) = quaternion_nb.x();
                (*currentQuaternionCoeffs)(2, 0) = quaternion_nb.y();
                (*currentQuaternionCoeffs)(3, 0) = quaternion_nb.z();
                notifyInputValueChanged(InputPortIndex_Quaternion);
            }
        }
        else // (connectedPin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* currentQuaternionCoeffs = getInputValue<BlockMatrix>(InputPortIndex_Quaternion))
            {
                auto block = (*currentQuaternionCoeffs)();

                block(0, 0) = quaternion_nb.w();
                block(1, 0) = quaternion_nb.x();
                block(2, 0) = quaternion_nb.y();
                block(3, 0) = quaternion_nb.z();
                notifyInputValueChanged(InputPortIndex_Quaternion);
            }
        }
    }
}

void NAV::ImuIntegrator::integrateObservation(const std::shared_ptr<NAV::NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    /// IMU Observation at the time tₖ
    auto imuObs__t0 = std::static_pointer_cast<ImuObs>(nodeData);

    if (!imuObs__t0->insTime.has_value())
    {
        return;
    }

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = imuObs__t0->imuPos;

    /// State Data at the time tₖ₋₁
    auto posVelAtt__t1 = std::make_shared<PosVelAtt>();
    if (!getCurrentPosition(posVelAtt__t1->position_ecef())
        || !getCurrentVelocity(posVelAtt__t1->velocity_n())
        || !getCurrentQuaternion_nb(posVelAtt__t1->quaternion_nb()))
    {
        return;
    }

    /// Initial State
    if (posVelAtt__init == nullptr)
    {
        posVelAtt__init = posVelAtt__t1;
    }

    // First state available
    if (imuObs__t2 == nullptr)
    {
        // Rotate StateData
        posVelAtt__t2 = posVelAtt__t1;

        // Rotate Observation
        imuObs__t2 = imuObs__t1;
        imuObs__t1 = imuObs__t0;

        return;
    }

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
    const Eigen::Vector3d& velocity_n__t1 = posVelAtt__t1->velocity_n();
    /// v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
    const Eigen::Vector3d& velocity_n__t2 = posVelAtt__t2->velocity_n();
    /// v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
    const Eigen::Vector3d velocity_e__t2 = posVelAtt__t2->quaternion_en() * velocity_n__t2;
    /// v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
    const Eigen::Vector3d velocity_e__t1 = posVelAtt__t1->quaternion_en() * velocity_n__t1;
    /// x_e (tₖ₋₂) Position in [m], in ECEF coordinates, at the time tₖ₋₂
    const Eigen::Vector3d position_e__t2 = posVelAtt__t2->position_ecef();
    /// x_e (tₖ₋₁) Position in [m], in ECEF coordinates, at the time tₖ₋₁
    const Eigen::Vector3d position_e__t1 = posVelAtt__t1->position_ecef();

    /// g_n Gravity vector in [m/s^2], in navigation coordinates
    const Eigen::Vector3d gravity_n__t1(0, 0, gravity::gravityMagnitude_Gleason(posVelAtt__t1->latitude()));
    /// g_e Gravity vector in [m/s^2], in earth coordinates
    const Eigen::Vector3d gravity_e__t1 = posVelAtt__t1->quaternion_en() * gravity_n__t1;

    if (integrationFrame == IntegrationFrame::ECEF)
    {
        /// q (tₖ₋₂) Quaternion, from gyro platform to earth coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_gyro_ep__t2 = posVelAtt__t2->quaternion_eb() * imuPosition.quatGyro_bp();
        /// q (tₖ₋₁) Quaternion, from gyro platform to earth coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_gyro_ep__t1 = posVelAtt__t1->quaternion_eb() * imuPosition.quatGyro_bp();

        /// ω_ie_e (tₖ) Angular velocity in [rad/s], of the inertial to earth system, in earth coordinates, at the time tₖ
        const Eigen::Vector3d& angularVelocity_ie_e__t0 = InsConst::angularVelocity_ie_e;

        /// q (tₖ₋₂) Quaternion, from accel platform to earth coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_accel_ep__t2 = posVelAtt__t2->quaternion_eb() * imuPosition.quatAccel_bp();
        /// q (tₖ₋₁) Quaternion, from accel platform to earth coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_accel_ep__t1 = posVelAtt__t1->quaternion_eb() * imuPosition.quatAccel_bp();

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Attitude Update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// q (tₖ) Quaternion, from platform to earth coordinates, at the current time tₖ
        const Eigen::Quaterniond quaternion_gyro_ep__t0 = updateQuaternion_ep_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                                          angularVelocity_ip_p__t0, angularVelocity_ip_p__t1,
                                                                                          angularVelocity_ie_e__t0,
                                                                                          quaternion_gyro_ep__t1, quaternion_gyro_ep__t2);

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                    Specific force frame transformation                                   */
        /* -------------------------------------------------------------------------------------------------------- */

        /// q (tₖ) Quaternion, from accel platform to earth coordinates, at the time tₖ
        const Eigen::Quaterniond quaternion_accel_ep__t0 = quaternion_gyro_ep__t0 * imuPosition.quatGyro_pb() * imuPosition.quatAccel_bp();

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Velocity update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// v (tₖ), Velocity in [m/s], in earth coordinates, at the current time tₖ
        const Eigen::Vector3d velocity_e__t0 = updateVelocity_e_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                            acceleration_p__t0, acceleration_p__t1,
                                                                            velocity_e__t2,
                                                                            position_e__t2,
                                                                            gravity_e__t1,
                                                                            quaternion_accel_ep__t0,
                                                                            quaternion_accel_ep__t1,
                                                                            quaternion_accel_ep__t2);

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Position update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// x_e (tₖ) Position in [m], in earth coordinates, at the time tₖ
        const Eigen::Vector3d position_e__t0 = updatePosition_e(timeDifferenceSec__t0, position_e__t1, velocity_e__t1);

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                               Store Results                                              */
        /* -------------------------------------------------------------------------------------------------------- */

        /// Result State Data at the time tₖ
        PosVelAtt posVelAtt__t0;
        // Store position in the state. Important to do before using the quaternion_en.
        posVelAtt__t0.position_ecef() = position_e__t0;
        /// Quaternion for rotation from earth to navigation frame. Depends on position which was updated before
        Eigen::Quaterniond quaternion_ne__t0 = posVelAtt__t0.quaternion_ne();
        // Store velocity in the state
        posVelAtt__t0.velocity_n() = quaternion_ne__t0 * velocity_e__t0;
        // Store body to navigation frame quaternion in the state
        posVelAtt__t0.quaternion_nb() = quaternion_ne__t0 * quaternion_gyro_ep__t0 * imuPosition.quatGyro_pb();

        setCurrentPosition(posVelAtt__t0.position_ecef());
        setCurrentVelocity(posVelAtt__t0.velocity_n());
        setCurrentQuaternion_nb(posVelAtt__t0.quaternion_nb());
    }
    else if (integrationFrame == IntegrationFrame::NED)
    {
        /// ω_ip_b (tₖ₋₁) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ₋₁
        const Eigen::Vector3d angularVelocity_ip_b__t1 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t1;
        /// ω_ip_b (tₖ) Angular velocity in [rad/s],
        /// of the inertial to platform system, in body coordinates, at the time tₖ
        const Eigen::Vector3d angularVelocity_ip_b__t0 = imuPosition.quatGyro_bp() * angularVelocity_ip_p__t0;

        /// a_b (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
        const Eigen::Vector3d acceleration_b__t1 = imuPosition.quatAccel_bp() * acceleration_p__t1;
        /// a_b (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
        const Eigen::Vector3d acceleration_b__t0 = imuPosition.quatAccel_bp() * acceleration_p__t0;

        /// q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond quaternion_nb__t1 = posVelAtt__t1->quaternion_nb();
        /// q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
        const Eigen::Quaterniond quaternion_nb__t2 = posVelAtt__t2->quaternion_nb();

        /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
        Eigen::Vector3d angularVelocity_ie_n__t1 = posVelAtt__t1->quaternion_ne() * InsConst::angularVelocity_ie_e;

        /// North/South (meridian) earth radius [m]
        double R_N = earthRadius_N(InsConst::WGS84_a, InsConst::WGS84_e_squared, posVelAtt__t1->latitude());
        /// East/West (prime vertical) earth radius [m]
        double R_E = earthRadius_E(InsConst::WGS84_a, InsConst::WGS84_e_squared, posVelAtt__t1->latitude());

        /// ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
        Eigen::Vector3d angularVelocity_en_n__t1 = transportRate(posVelAtt__t1->latLonAlt(), velocity_n__t1, R_N, R_E);

        /// [x_n, x_e, x_d] (tₖ₋₁) Position NED in [m] at the time tₖ₋₁
        Eigen::Vector3d position_n__t1 = trafo::ecef2ned(position_e__t1, posVelAtt__init->latLonAlt());

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Attitude Update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// q (tₖ) Quaternion, from body to navigation coordinates, at the current time tₖ
        Eigen::Quaterniond quaternion_nb__t0 = updateQuaternion_nb_RungeKutta3(timeDifferenceSec__t0,
                                                                               timeDifferenceSec__t1,
                                                                               angularVelocity_ip_b__t0,
                                                                               angularVelocity_ip_b__t1,
                                                                               angularVelocity_ie_n__t1,
                                                                               angularVelocity_en_n__t1,
                                                                               quaternion_nb__t1,
                                                                               quaternion_nb__t2);

        /* -------------------------------------------------------------------------------------------------------- */
        /*                           Specific force frame transformation & Velocity update                          */
        /* -------------------------------------------------------------------------------------------------------- */

        /// v (tₖ), Velocity in navigation coordinates, at the current time tₖ
        Eigen::Vector3d velocity_n__t0 = updateVelocity_n_RungeKutta3(timeDifferenceSec__t0, timeDifferenceSec__t1,
                                                                      acceleration_b__t0, acceleration_b__t1,
                                                                      velocity_n__t1, velocity_n__t2,
                                                                      gravity_n__t1,
                                                                      angularVelocity_ie_n__t1,
                                                                      angularVelocity_en_n__t1,
                                                                      quaternion_nb__t0, quaternion_nb__t1, quaternion_nb__t2);

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                              Position update                                             */
        /* -------------------------------------------------------------------------------------------------------- */

        /// [x_n, x_e, x_d] (tₖ) Position NED in [m] at the time tₖ
        Eigen::Vector3d position_n__t0 = updatePosition_n(timeDifferenceSec__t0, position_n__t1, velocity_n__t1);

        /// x_e (tₖ) Position in [m], in ECEF coordinates, at the time tₖ
        Eigen::Vector3d position_e__t0 = trafo::ned2ecef(position_n__t0, posVelAtt__init->latLonAlt());

        /// Latitude, Longitude and Altitude in [rad, rad, m], at the current time tₖ (see Gleason eq. 6.18 - 6.20)
        // Vector3d<LLA> latLonAlt__t0 = updatePosition_n(timeDifferenceSec__t0, posVelAtt__t1->latLonAlt(),
        //                                                     velocity_n__t1, R_N, R_E);

        /* -------------------------------------------------------------------------------------------------------- */
        /*                                               Store Results                                              */
        /* -------------------------------------------------------------------------------------------------------- */

        /// Result State Data at the time tₖ
        PosVelAtt posVelAtt__t0;
        // Store position in the state
        posVelAtt__t0.position_ecef() = position_e__t0;
        // Store velocity in the state
        posVelAtt__t0.velocity_n() = velocity_n__t0;
        // Store body to navigation frame quaternion in the state
        posVelAtt__t0.quaternion_nb() = quaternion_nb__t0;

        setCurrentPosition(posVelAtt__t0.position_ecef());
        setCurrentVelocity(posVelAtt__t0.velocity_n());
        setCurrentQuaternion_nb(posVelAtt__t0.quaternion_nb());
    }

    // Rotate StateData
    posVelAtt__t2 = posVelAtt__t1;

    // Rotate Observation
    imuObs__t2 = imuObs__t1;
    imuObs__t1 = imuObs__t0;
}