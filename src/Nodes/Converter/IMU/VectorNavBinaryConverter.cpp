#include "VectorNavBinaryConverter.hpp"

#include <cmath>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/InsTransformations.hpp"

NAV::VectorNavBinaryConverter::VectorNavBinaryConverter()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);
    hasConfig = true;
    guiConfigDefaultWindowSize = { 350, 123 };

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObsWDelta::type() });

    nm::CreateInputPin(this, "VectorNavBinaryOutput", Pin::Type::Flow, { NAV::VectorNavBinaryOutput::type() }, &VectorNavBinaryConverter::receiveObs);
}

NAV::VectorNavBinaryConverter::~VectorNavBinaryConverter()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::VectorNavBinaryConverter::typeStatic()
{
    return "VectorNavBinaryConverter";
}

std::string NAV::VectorNavBinaryConverter::type() const
{
    return typeStatic();
}

std::string NAV::VectorNavBinaryConverter::category()
{
    return "Converter";
}

void NAV::VectorNavBinaryConverter::guiConfig()
{
    if (ImGui::Combo(fmt::format("Output Type##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&outputType), "ImuObsWDelta\0PosVelAtt\0\0"))
    {
        LOG_DEBUG("{}: Output Type changed to {}", nameId(), outputType ? "PosVelAtt" : "ImuObsWDelta");

        if (outputType == OutputType_ImuObsWDelta)
        {
            outputPins.at(OutputPortIndex_Converted).dataIdentifier = { NAV::ImuObsWDelta::type() };
            outputPins.at(OutputPortIndex_Converted).name = NAV::ImuObsWDelta::type();
        }
        else if (outputType == OutputType_PosVelAtt)
        {
            outputPins.at(OutputPortIndex_Converted).dataIdentifier = { NAV::PosVelAtt::type() };
            outputPins.at(OutputPortIndex_Converted).name = NAV::PosVelAtt::type();
        }

        for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.front().id))
        {
            nm::RefreshLink(link->id);
        }

        flow::ApplyChanges();
    }

    if (outputType == OutputType_PosVelAtt)
    {
        if (ImGui::Combo(fmt::format("Data Source##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&posVelSource), "Best\0INS\0GNSS 1\0GNSS 2\0\0"))
        {
            LOG_DEBUG("{}: posVelSource changed to {}", nameId(), posVelSource);
            flow::ApplyChanges();
        }
        if (ImGui::Checkbox(fmt::format("Force static##{}", size_t(id)).c_str(), &forceStatic))
        {
            LOG_DEBUG("{}: forceStatic changed to {}", nameId(), forceStatic);
            flow::ApplyChanges();
        }
    }
}

[[nodiscard]] json NAV::VectorNavBinaryConverter::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["outputType"] = outputType;
    j["posVelSource"] = posVelSource;
    j["forceStatic"] = forceStatic;

    return j;
}

void NAV::VectorNavBinaryConverter::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputType"))
    {
        outputType = static_cast<OutputType>(j.at("outputType").get<int>());

        if (!outputPins.empty())
        {
            if (outputType == OutputType_ImuObsWDelta)
            {
                outputPins.at(OutputPortIndex_Converted).dataIdentifier = { NAV::ImuObsWDelta::type() };
                outputPins.at(OutputPortIndex_Converted).name = NAV::ImuObsWDelta::type();
            }
            else if (outputType == OutputType_PosVelAtt)
            {
                outputPins.at(OutputPortIndex_Converted).dataIdentifier = { NAV::PosVelAtt::type() };
                outputPins.at(OutputPortIndex_Converted).name = NAV::PosVelAtt::type();
            }
        }
    }
    if (j.contains("posVelSource"))
    {
        posVelSource = static_cast<PosVelSource>(j.at("posVelSource").get<int>());
    }
    if (j.contains("forceStatic"))
    {
        forceStatic = j.at("forceStatic");
    }
}

bool NAV::VectorNavBinaryConverter::initialize()
{
    LOG_TRACE("{}: called", nameId());

    posVelAtt__init = nullptr;

    return true;
}

void NAV::VectorNavBinaryConverter::receiveObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto vnObs = std::dynamic_pointer_cast<const VectorNavBinaryOutput>(nodeData);

    std::shared_ptr<const NodeData> convertedData = nullptr;

    if (outputType == OutputType_ImuObsWDelta)
    {
        convertedData = convert2ImuObsWDelta(vnObs);
    }
    else if (outputType == OutputType_PosVelAtt)
    {
        convertedData = convert2PosVelAtt(vnObs);
    }

    if (convertedData)
    {
        invokeCallbacks(OutputPortIndex_Converted, convertedData);
    }
}

std::shared_ptr<const NAV::ImuObsWDelta> NAV::VectorNavBinaryConverter::convert2ImuObsWDelta(const std::shared_ptr<const VectorNavBinaryOutput>& vnObs)
{
    auto imuObs = std::make_shared<ImuObsWDelta>(vnObs->imuPos);

    imuObs->insTime = vnObs->insTime;

    if (vnObs->timeOutputs)
    {
        if (vnObs->timeOutputs->timeField & vn::protocol::uart::TimeGroup::TIMEGROUP_TIMESTARTUP)
        {
            imuObs->timeSinceStartup = vnObs->timeOutputs->timeStartup;
        }
    }
    if (vnObs->imuOutputs)
    {
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPMAG)
        {
            imuObs->magUncompXYZ = vnObs->imuOutputs->uncompMag.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPACCEL)
        {
            imuObs->accelUncompXYZ = vnObs->imuOutputs->uncompAccel.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_UNCOMPGYRO)
        {
            imuObs->gyroUncompXYZ = vnObs->imuOutputs->uncompGyro.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_MAG)
        {
            imuObs->magCompXYZ = vnObs->imuOutputs->mag.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ACCEL)
        {
            imuObs->accelCompXYZ = vnObs->imuOutputs->accel.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_ANGULARRATE)
        {
            imuObs->gyroCompXYZ = vnObs->imuOutputs->angularRate.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_TEMP)
        {
            imuObs->temperature = vnObs->imuOutputs->temp;
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTATHETA)
        {
            imuObs->dtime = vnObs->imuOutputs->deltaTime;
            imuObs->dtheta = vnObs->imuOutputs->deltaTheta.cast<double>();
        }
        if (vnObs->imuOutputs->imuField & vn::protocol::uart::ImuGroup::IMUGROUP_DELTAVEL)
        {
            imuObs->dvel = vnObs->imuOutputs->deltaV.cast<double>();
        }
    }

    if (imuObs->magUncompXYZ.has_value() || imuObs->accelUncompXYZ.has_value() || imuObs->gyroUncompXYZ.has_value()
        || imuObs->magCompXYZ.has_value() || imuObs->accelCompXYZ.has_value() || imuObs->gyroCompXYZ.has_value()
        || (!std::isnan(imuObs->dtime) && (imuObs->dtheta.has_value() || imuObs->dvel.has_value())))
    {
        return imuObs;
    }

    LOG_ERROR("{}: Conversion failed. No relevant data found in the input data.", nameId());
    return nullptr;
}

std::shared_ptr<const NAV::PosVelAtt> NAV::VectorNavBinaryConverter::convert2PosVelAtt(const std::shared_ptr<const VectorNavBinaryOutput>& vnObs)
{
    std::optional<Eigen::Quaterniond> quat_nb;
    std::optional<Eigen::Vector3d> p_ecef;
    std::optional<Eigen::Vector3d> p_lla;
    std::optional<Eigen::Vector3d> v_n;

    if (vnObs->attitudeOutputs)
    {
        if (vnObs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
        {
            quat_nb = vnObs->attitudeOutputs->qtn.cast<double>();
        }
        else if (vnObs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
        {
            auto ypr = trafo::deg2rad3(vnObs->attitudeOutputs->ypr.cast<double>());
            quat_nb = trafo::quat_nb(ypr(2), ypr(1), ypr(0));
        }
        else if (vnObs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
        {
            quat_nb = vnObs->attitudeOutputs->dcm.cast<double>();
        }
    }

    if ((posVelSource == PosVelSource_Best || posVelSource == PosVelSource_Ins)
        && vnObs->insOutputs && (vnObs->insOutputs->insStatus.mode() == 1 || vnObs->insOutputs->insStatus.mode() == 2))
    {
        if (vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
        {
            p_lla = { trafo::deg2rad(vnObs->insOutputs->posLla(0)),
                      trafo::deg2rad(vnObs->insOutputs->posLla(1)),
                      vnObs->insOutputs->posLla(2) };
        }
        if (vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
        {
            p_ecef = vnObs->insOutputs->posEcef;
        }

        if (vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
        {
            v_n = vnObs->insOutputs->velNed.cast<double>();
        }
        else if ((vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
                 && (p_ecef.has_value() || p_lla.has_value()))
        {
            Eigen::Vector3d lla = p_lla.has_value() ? p_lla.value() : trafo::ecef2lla_WGS84(p_ecef.value());
            v_n = trafo::quat_ne(lla(0), lla(1)) * vnObs->insOutputs->velEcef.cast<double>();
        }
        else if ((vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
                 && quat_nb.has_value())
        {
            v_n = quat_nb.value() * vnObs->insOutputs->velBody.cast<double>();
        }
    }

    if ((posVelSource == PosVelSource_Best || posVelSource == PosVelSource_Gnss1)
        && vnObs->gnss1Outputs && vnObs->gnss1Outputs->fix >= 2)
    {
        if (!p_ecef.has_value() && !p_lla.has_value())
        {
            if (vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                p_lla = { trafo::deg2rad(vnObs->gnss1Outputs->posLla(0)),
                          trafo::deg2rad(vnObs->gnss1Outputs->posLla(1)),
                          vnObs->gnss1Outputs->posLla(2) };
            }
            if (vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                p_ecef = vnObs->gnss1Outputs->posEcef;
            }
        }

        if (!v_n.has_value())
        {
            if (vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                v_n = vnObs->gnss1Outputs->velNed.cast<double>();
            }
            else if ((vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                     && (p_ecef.has_value() || p_lla.has_value()))
            {
                Eigen::Vector3d lla = p_lla.has_value() ? p_lla.value() : trafo::ecef2lla_WGS84(p_ecef.value());
                v_n = trafo::quat_ne(lla(0), lla(1)) * vnObs->gnss1Outputs->velEcef.cast<double>();
            }
        }
    }
    if ((posVelSource == PosVelSource_Best || posVelSource == PosVelSource_Gnss2)
        && vnObs->gnss2Outputs && vnObs->gnss2Outputs->fix >= 2)
    {
        if (!p_ecef.has_value() && !p_lla.has_value())
        {
            if (vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                p_lla = { trafo::deg2rad(vnObs->gnss2Outputs->posLla(0)),
                          trafo::deg2rad(vnObs->gnss2Outputs->posLla(1)),
                          vnObs->gnss2Outputs->posLla(2) };
            }
            if (vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                p_ecef = vnObs->gnss2Outputs->posEcef;
            }
        }

        if (!v_n.has_value())
        {
            if (vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                v_n = vnObs->gnss2Outputs->velNed.cast<double>();
            }
            else if ((vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                     && (p_ecef.has_value() || p_lla.has_value()))
            {
                Eigen::Vector3d lla = p_lla.has_value() ? p_lla.value() : trafo::ecef2lla_WGS84(p_ecef.value());
                v_n = trafo::quat_ne(lla(0), lla(1)) * vnObs->gnss2Outputs->velEcef.cast<double>();
            }
        }
    }

    auto posVelAttObs = std::make_shared<PosVelAtt>();

    posVelAttObs->insTime = vnObs->insTime;

    if ((p_ecef.has_value() || p_lla.has_value()) && v_n.has_value())
    {
        if (p_ecef.has_value())
        {
            posVelAttObs->setPosition_e(p_ecef.value());
        }
        else
        {
            posVelAttObs->setPosition_lla(p_lla.value());
        }
        posVelAttObs->setVelocity_n(v_n.value());

        if (!quat_nb.has_value())
        {
            LOG_DEBUG("{}: Conversion succeeded but has no attitude info.", nameId());
        }
        else
        {
            posVelAttObs->setAttitude_nb(quat_nb.value());
        }

        if (posVelAtt__init == nullptr)
        {
            posVelAtt__init = posVelAttObs;
        }

        if (forceStatic)
        {
            posVelAttObs->setPosition_e(posVelAtt__init->position_ecef());
            posVelAttObs->setVelocity_n(Eigen::Vector3d::Zero());
            posVelAttObs->setAttitude_nb(posVelAtt__init->quaternion_nb());
        }

        return posVelAttObs;
    }

    LOG_ERROR("{}: Conversion failed. No position or velocity data found in the input data.", nameId());
    return nullptr;
}