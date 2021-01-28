#include "ImuSimulator.hpp"

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"
#include "util/InsGravity.hpp"
#include "util/Time/TimeBase.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/State/StateData.hpp"

NAV::ImuSimulator::ImuSimulator()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, NAV::ImuObs::type(), &ImuSimulator::pollData);
    nm::CreateInputPin(this, "State", Pin::Type::Object, { NAV::StateData::type() });
}

NAV::ImuSimulator::~ImuSimulator()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuSimulator::typeStatic()
{
    return "ImuSimulator";
}

std::string NAV::ImuSimulator::type() const
{
    return typeStatic();
}

std::string NAV::ImuSimulator::category()
{
    return "Data Simulator";
}

void NAV::ImuSimulator::guiConfig()
{
    if (ImGui::InputDouble("Duration", &duration, 0.1, 1.0, "%.6f s"))
    {
        if (duration < 0)
        {
            duration = 0;
        }
        LOG_DEBUG("{}: Duration changed to {}", nameId(), duration);
        flow::ApplyChanges();
    }
    if (ImGui::InputDouble("Frequency", &frequency, 0.1, 1.0, "%.6f Hz"))
    {
        if (frequency <= 0.000000001)
        {
            frequency = 0.1;
        }
        LOG_DEBUG("{}: Frequency changed to {}", nameId(), frequency);
        flow::ApplyChanges();
    }

    ImGui::Separator();

    if (ImGui::InputFloat3("Accel n [m/s²]", accel_n.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Accel_n changed to [{}, {}, {}]", nameId(), accel_n.x(), accel_n.y(), accel_n.z());
        flow::ApplyChanges();
    }
    if (ImGui::InputFloat3("Gyro n [rad/s]", gyro_n.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Gyro_n changed to [{}, {}, {}]", nameId(), gyro_n.x(), gyro_n.y(), gyro_n.z());
        flow::ApplyChanges();
    }
    if (ImGui::InputFloat3("Mag n [Gauss]", mag_n.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Mag_n changed to [{}, {}, {}]", nameId(), mag_n.x(), mag_n.y(), mag_n.z());
        flow::ApplyChanges();
    }

    ImGui::Separator();

    if (ImGui::InputFloat3("Accel_b [m/s²]", accel_b.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Accel_b changed to [{}, {}, {}]", nameId(), accel_b.x(), accel_b.y(), accel_b.z());
        flow::ApplyChanges();
    }
    if (ImGui::InputFloat3("Gyro b [rad/s]", gyro_b.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Gyro_b changed to [{}, {}, {}]", nameId(), gyro_b.x(), gyro_b.y(), gyro_b.z());
        flow::ApplyChanges();
    }
    if (ImGui::InputFloat3("Mag b [Gauss]", mag_b.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Mag_b changed to [{}, {}, {}]", nameId(), mag_b.x(), mag_b.y(), mag_b.z());
        flow::ApplyChanges();
    }

    ImGui::Separator();

    if (ImGui::InputFloat3("Accel_p [m/s²]", accel_p.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Accel_p changed to [{}, {}, {}]", nameId(), accel_p.x(), accel_p.y(), accel_p.z());
        flow::ApplyChanges();
    }
    if (ImGui::InputFloat3("Gyro p [rad/s]", gyro_p.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Gyro_p changed to [{}, {}, {}]", nameId(), gyro_p.x(), gyro_p.y(), gyro_p.z());
        flow::ApplyChanges();
    }
    if (ImGui::InputFloat3("Mag p [Gauss]", mag_p.data(), "%.3e", ImGuiInputTextFlags_CharsScientific))
    {
        LOG_DEBUG("{}: Mag_p changed to [{}, {}, {}]", nameId(), mag_p.x(), mag_p.y(), mag_p.z());
        flow::ApplyChanges();
    }

    ImGui::Separator();

    if (ImGui::InputDouble("Temperature", &temperature, 0.1, 1.0, "%.6f °C"))
    {
        LOG_DEBUG("{}: Temperature changed to {}", nameId(), temperature);
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::ImuSimulator::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["duration"] = duration;
    j["frequency"] = frequency;

    j["accel_n"] = accel_n;
    j["accel_b"] = accel_b;
    j["accel_p"] = accel_p;
    j["gyro_n"] = gyro_n;
    j["gyro_b"] = gyro_b;
    j["gyro_p"] = gyro_p;
    j["mag_n"] = mag_n;
    j["mag_b"] = mag_b;
    j["mag_p"] = mag_p;

    j["temperature"] = temperature;

    return j;
}

void NAV::ImuSimulator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("duration"))
    {
        j.at("duration").get_to(duration);
    }
    if (j.contains("frequency"))
    {
        j.at("frequency").get_to(frequency);
    }

    if (j.contains("accel_n"))
    {
        j.at("accel_n").get_to(accel_n);
    }
    if (j.contains("accel_b"))
    {
        j.at("accel_b").get_to(accel_b);
    }
    if (j.contains("accel_p"))
    {
        j.at("accel_p").get_to(accel_p);
    }
    if (j.contains("gyro_n"))
    {
        j.at("gyro_n").get_to(gyro_n);
    }
    if (j.contains("gyro_b"))
    {
        j.at("gyro_b").get_to(gyro_b);
    }
    if (j.contains("gyro_p"))
    {
        j.at("gyro_p").get_to(gyro_p);
    }
    if (j.contains("mag_n"))
    {
        j.at("mag_n").get_to(mag_n);
    }
    if (j.contains("mag_b"))
    {
        j.at("mag_b").get_to(mag_b);
    }
    if (j.contains("mag_p"))
    {
        j.at("mag_p").get_to(mag_p);
    }

    if (j.contains("temperature"))
    {
        j.at("temperature").get_to(temperature);
    }
}

bool NAV::ImuSimulator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    startTime = util::time::GetCurrentTime();

    return !startTime.empty();
}

void NAV::ImuSimulator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::ImuSimulator::resetNode()
{
    currentSimTime = 0.0;

    return true;
}

std::shared_ptr<NAV::NodeData> NAV::ImuSimulator::pollData(bool peek)
{
    if (currentSimTime > duration)
    {
        return nullptr;
    }

    if (const auto* stateData = getInputValue<StateData>(InputPortIndex_StateData))
    {
        auto quat_bn = Eigen::Quaterniond::Identity();
        auto quat_ne = Eigen::Quaterniond::Identity();
        double latitude = 0;
        if (stateData)
        {
            quat_bn = stateData->quaternion_bn();
            quat_ne = stateData->quaternion_ne();
            latitude = stateData->latitude();
        }

        auto obs = std::make_shared<ImuObs>(imuPos);
        obs->timeSinceStartup = static_cast<uint64_t>(currentSimTime * 1e9);
        obs->insTime = startTime + std::chrono::nanoseconds(obs->timeSinceStartup.value());

        /// g_n Gravity vector in [m/s^2], in navigation coordinates
        Eigen::Vector3d gravity_n{ 0, 0, gravity::gravityMagnitude_Gleason(latitude) };

        /// ω_ie_n Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates
        Eigen::Vector3d angularVelocity_ie_n = quat_ne * InsConst::angularVelocity_ie_e;

        obs->accelUncompXYZ = accel_p.cast<double>() + imuPos.quatAccel_pb() * (accel_b.cast<double>() + quat_bn * (accel_n.cast<double>() - gravity_n));
        obs->gyroUncompXYZ = gyro_p.cast<double>() + imuPos.quatGyro_pb() * (gyro_b.cast<double>() + quat_bn * (gyro_n.cast<double>() + angularVelocity_ie_n));
        obs->magUncompXYZ = mag_p.cast<double>() + imuPos.quatMag_pb() * (mag_b.cast<double>() + quat_bn * mag_n.cast<double>());
        obs->temperature = temperature;

        // Calls all the callbacks
        if (!peek)
        {
            currentSimTime += 1.0 / frequency;
            invokeCallbacks(OutputPortIndex_ImuObs, obs);
        }

        return obs;
    }

    return nullptr;
}