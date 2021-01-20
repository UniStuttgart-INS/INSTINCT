#include "State.hpp"

#include "util/Logger.hpp"
#include "util/InsMath.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::State::State()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "StateData", Pin::Type::Object, NAV::StateData::type(), &currentState);

    nm::CreateInputPin(this, "StateData\nUpdate", Pin::Type::Flow, { NAV::StateData::type() }, &State::updateState);
    nm::CreateInputPin(this, "GnssObs\nInit", Pin::Type::Flow, { NAV::GnssObs::type() }, &State::initPositionVelocity);
    nm::CreateInputPin(this, "ImuObs\nInit", Pin::Type::Flow, { NAV::ImuObs::type() }, &State::initAttitude);
}

NAV::State::~State()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::State::typeStatic()
{
    return "State";
}

std::string NAV::State::type() const
{
    return typeStatic();
}

std::string NAV::State::category()
{
    return "State";
}

void NAV::State::guiConfig()
{
    if (ImGui::Checkbox("Dynamic State Initialization", &dynamicStateInit))
    {
        LOG_DEBUG("{}: Dynamic State Initialization changed to {}", nameId(), dynamicStateInit);
        flow::ApplyChanges();
    }
    if (dynamicStateInit)
    {
        if (ImGui::InputDouble("Init Duration", &initDuration, 0.1, 1.0, "%.2f s"))
        {
            if (initDuration < 0.01)
            {
                initDuration = 0.01;
            }
            LOG_DEBUG("{}: Init Duration changed to {}", nameId(), initDuration);
            flow::ApplyChanges();
        }
    }
    else
    {
        if (ImGui::InputFloat3("Init LatLonAlt", initLatLonAlt.data()))
        {
            initialState.position_ecef() = trafo::lla2ecef_WGS84({ trafo::deg2rad(initLatLonAlt.at(0)),
                                                                   trafo::deg2rad(initLatLonAlt.at(1)),
                                                                   initLatLonAlt.at(2) });
            LOG_DEBUG("{}: Init LatLonAlt changed to [{}°, {}°, {}m]", nameId(),
                      trafo::rad2deg(initialState.latitude()), trafo::rad2deg(initialState.longitude()), initialState.altitude());
            flow::ApplyChanges();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Latitude in [rad], longitude in [rad], altitude (height above ground) in [m]");
        }
        if (ImGui::InputFloat3("Init RollPitchYaw", initRollPitchYaw.data()))
        {
            initialState.quaternion_nb() = trafo::quat_nb(trafo::deg2rad(initRollPitchYaw.at(0)),
                                                          trafo::deg2rad(initRollPitchYaw.at(1)),
                                                          trafo::deg2rad(initRollPitchYaw.at(2)));
            LOG_DEBUG("{}: Init RollPitchYaw changed to [{}°, {}°, {}°]", nameId(),
                      trafo::rad2deg(initialState.rollPitchYaw().x()),
                      trafo::rad2deg(initialState.rollPitchYaw().y()),
                      trafo::rad2deg(initialState.rollPitchYaw().z()));
            flow::ApplyChanges();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Roll in [deg], pitch in [deg], yaw in [deg]");
        }
        if (ImGui::InputFloat3("Init Velocity", initVelocityNED.data()))
        {
            initialState.velocity_n() = Eigen::Vector3f(initVelocityNED.data()).cast<double>();
            LOG_DEBUG("{}: Init Velocity changed to [{}, {}, {}]", nameId(),
                      initialState.velocity_n().x(),
                      initialState.velocity_n().y(),
                      initialState.velocity_n().z());
            flow::ApplyChanges();
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Initial velocity in navigation (NED) frame [m/s]");
        }
    }
}

[[nodiscard]] json NAV::State::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["dynamicStateInit"] = dynamicStateInit;
    j["initDuration"] = initDuration;
    j["initLatLonAlt"] = initLatLonAlt;
    j["initRollPitchYaw"] = initRollPitchYaw;
    j["initVelocityNED"] = initVelocityNED;

    return j;
}

void NAV::State::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicStateInit"))
    {
        j.at("dynamicStateInit").get_to(dynamicStateInit);
    }
    if (j.contains("initDuration"))
    {
        j.at("initDuration").get_to(initDuration);
    }
    if (j.contains("initLatLonAlt"))
    {
        j.at("initLatLonAlt").get_to(initLatLonAlt);
        initialState.position_ecef() = trafo::lla2ecef_WGS84({ trafo::deg2rad(initLatLonAlt.at(0)),
                                                               trafo::deg2rad(initLatLonAlt.at(1)),
                                                               initLatLonAlt.at(2) });
    }
    if (j.contains("initRollPitchYaw"))
    {
        j.at("initRollPitchYaw").get_to(initRollPitchYaw);
        initialState.quaternion_nb() = trafo::quat_nb(trafo::deg2rad(initRollPitchYaw.at(0)),
                                                      trafo::deg2rad(initRollPitchYaw.at(1)),
                                                      trafo::deg2rad(initRollPitchYaw.at(2)));
    }
    if (j.contains("initVelocityNED"))
    {
        j.at("initVelocityNED").get_to(initVelocityNED);
        initialState.velocity_n() = Eigen::Vector3f(initVelocityNED.data()).cast<double>();
    }

    currentState = StateData();
    currentState.quaternion_nb() = initialState.quaternion_nb();
    currentState.position_ecef() = initialState.position_ecef();
    currentState.velocity_n() = initialState.velocity_n();
}

bool NAV::State::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::State::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::State::updateState(const std::shared_ptr<NodeData>& state, ax::NodeEditor::LinkId /*linkId*/)
{
    // Process the data here
    currentState = *std::static_pointer_cast<StateData>(state);
}

void NAV::State::initAttitude(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);

    // Position and rotation information for conversion of IMU data from platform to body frame
    const auto& imuPosition = obs->imuPos;

    const auto magUncomp_b = imuPosition.quatMag_bp() * obs->magUncompXYZ.value();
    auto magneticHeading = -std::atan2(magUncomp_b.y(), magUncomp_b.x());

    const auto accelUncomp_b = imuPosition.quatAccel_bp() * obs->accelUncompXYZ.value() * -1;
    auto roll = rollFromStaticAccelerationObs(accelUncomp_b);
    auto pitch = pitchFromStaticAccelerationObs(accelUncomp_b);

    // TODO: Determine Velocity first and if vehicle not static, initialize the attitude from velocity

    // Average with previous attitude
    countAveragedAttitude++;
    if (countAveragedAttitude > 1)
    {
        roll = (initialState.rollPitchYaw().x() * (countAveragedAttitude - 1) + roll) / countAveragedAttitude;
        pitch = (initialState.rollPitchYaw().y() * (countAveragedAttitude - 1) + pitch) / countAveragedAttitude;
        magneticHeading = (initialState.rollPitchYaw().z() * (countAveragedAttitude - 1) + magneticHeading) / countAveragedAttitude;
    }

    initialState.quaternion_nb() = trafo::quat_nb(roll, pitch, magneticHeading);

    finalizeInit(obs->insTime.value());
}

void NAV::State::initPositionVelocity(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<GnssObs>(nodeData);

    if (obs->position_ecef.has_value())
    {
        double p_x = obs->position_ecef->x();
        double p_y = obs->position_ecef->y();
        double p_z = obs->position_ecef->z();

        // Already received a position, so calculate velocity
        if (countAveragedPosition > 0)
        {
            auto dt = (obs->insTime.value() - initialState.insTime.value()).count();

            Eigen::Vector3d velocity_e = (obs->position_ecef.value() - initialState.position_ecef()) / dt;
            Eigen::Vector3d velocity_n = initialState.quaternion_ne() * velocity_e;

            // Average with previous velocity
            countAveragedVelocity++;
            if (countAveragedVelocity > 1)
            {
                velocity_n = (initialState.velocity_n() * (countAveragedVelocity - 1) + velocity_n) / countAveragedVelocity;
            }

            initialState.velocity_n() = velocity_n;
        }

        // Average with previous position
        countAveragedPosition++;
        if (countAveragedPosition > 1)
        {
            p_x = (initialState.position_ecef().x() * (countAveragedPosition - 1) + p_x) / countAveragedPosition;
            p_y = (initialState.position_ecef().y() * (countAveragedPosition - 1) + p_y) / countAveragedPosition;
            p_z = (initialState.position_ecef().z() * (countAveragedPosition - 1) + p_z) / countAveragedPosition;
        }
        initialState.position_ecef() = { p_x, p_y, p_z };
        initialState.insTime = obs->insTime;

        finalizeInit(obs->insTime.value());
    }
}

void NAV::State::finalizeInit(const InsTime& currentTime)
{
    constexpr double minimumSamples = 2;
    if (averageStartTime > currentTime)
    {
        averageStartTime = currentTime;
    }
    if ((currentTime - averageStartTime).count() > static_cast<long double>(initDuration)
        && countAveragedAttitude >= minimumSamples && countAveragedPosition >= minimumSamples && countAveragedVelocity >= minimumSamples)
    {
        dynamicStateInit = false;

        LOG_INFO("{}: State initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", name,
                 trafo::rad2deg(initialState.latitude()),
                 trafo::rad2deg(initialState.longitude()),
                 initialState.altitude());
        LOG_INFO("{}: State initialized to v_N {:3.5f} [m/s], v_E {:3.5f}, v_D {:3.5f}", name,
                 initialState.velocity_n().x(),
                 initialState.velocity_n().x(),
                 initialState.velocity_n().z());
        LOG_INFO("{}: State initialized to Roll {:3.5f} [°], Pitch {:3.5f} [°], Yaw {:3.4f} [°]", name,
                 trafo::rad2deg(initialState.rollPitchYaw().x()),
                 trafo::rad2deg(initialState.rollPitchYaw().y()),
                 trafo::rad2deg(initialState.rollPitchYaw().z()));

        currentState = StateData();
        currentState.insTime = currentTime;
        currentState.quaternion_nb() = initialState.quaternion_nb();
        currentState.position_ecef() = initialState.position_ecef();
        currentState.velocity_n() = initialState.velocity_n();
    }
}