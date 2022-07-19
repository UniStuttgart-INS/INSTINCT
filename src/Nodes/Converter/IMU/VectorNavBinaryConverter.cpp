#include "VectorNavBinaryConverter.hpp"

#include <cmath>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

NAV::VectorNavBinaryConverter::VectorNavBinaryConverter()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 350, 123 };

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
    if (ImGui::Combo(fmt::format("Output Type##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_outputType), "ImuObsWDelta\0PosVelAtt\0\0"))
    {
        LOG_DEBUG("{}: Output Type changed to {}", nameId(), _outputType ? "PosVelAtt" : "ImuObsWDelta");

        if (_outputType == OutputType_ImuObsWDelta)
        {
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::ImuObsWDelta::type() };
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::ImuObsWDelta::type();
        }
        else if (_outputType == OutputType_PosVelAtt)
        {
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::PosVelAtt::type() };
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::PosVelAtt::type();
        }

        for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.front().id))
        {
            nm::RefreshLink(link->id);
        }

        flow::ApplyChanges();
    }

    if (_outputType == OutputType_PosVelAtt)
    {
        if (ImGui::Combo(fmt::format("Data Source##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_posVelSource), "Best\0INS\0GNSS 1\0GNSS 2\0\0"))
        {
            LOG_DEBUG("{}: _posVelSource changed to {}", nameId(), _posVelSource);
            flow::ApplyChanges();
        }
        if (ImGui::Checkbox(fmt::format("Force static##{}", size_t(id)).c_str(), &_forceStatic))
        {
            LOG_DEBUG("{}: _forceStatic changed to {}", nameId(), _forceStatic);
            flow::ApplyChanges();
        }
    }
}

[[nodiscard]] json NAV::VectorNavBinaryConverter::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["outputType"] = _outputType;
    j["posVelSource"] = _posVelSource;
    j["forceStatic"] = _forceStatic;

    return j;
}

void NAV::VectorNavBinaryConverter::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputType"))
    {
        j.at("outputType").get_to(_outputType);

        if (!outputPins.empty())
        {
            if (_outputType == OutputType_ImuObsWDelta)
            {
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::ImuObsWDelta::type() };
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::ImuObsWDelta::type();
            }
            else if (_outputType == OutputType_PosVelAtt)
            {
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::PosVelAtt::type() };
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::PosVelAtt::type();
            }
        }
    }
    if (j.contains("posVelSource"))
    {
        j.at("posVelSource").get_to(_posVelSource);
    }
    if (j.contains("forceStatic"))
    {
        _forceStatic = j.at("forceStatic");
    }
}

bool NAV::VectorNavBinaryConverter::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _posVelAtt__init = nullptr;

    return true;
}

void NAV::VectorNavBinaryConverter::receiveObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto vnObs = std::static_pointer_cast<const VectorNavBinaryOutput>(nodeData);

    std::shared_ptr<const NodeData> convertedData = nullptr;

    if (_outputType == OutputType_ImuObsWDelta)
    {
        convertedData = convert2ImuObsWDelta(vnObs);
    }
    else if (_outputType == OutputType_PosVelAtt)
    {
        convertedData = convert2PosVelAtt(vnObs);
    }

    if (convertedData)
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_CONVERTED, convertedData);
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
    std::optional<Eigen::Quaterniond> n_Quat_b;
    std::optional<Eigen::Vector3d> e_position;
    std::optional<Eigen::Vector3d> lla_position;
    std::optional<Eigen::Vector3d> n_velocity;

    if (vnObs->attitudeOutputs)
    {
        if (vnObs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_QUATERNION)
        {
            n_Quat_b = vnObs->attitudeOutputs->qtn.cast<double>();
        }
        else if (vnObs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_YAWPITCHROLL)
        {
            auto ypr = deg2rad(vnObs->attitudeOutputs->ypr.cast<double>());
            n_Quat_b = trafo::n_Quat_b(ypr(2), ypr(1), ypr(0));
        }
        else if (vnObs->attitudeOutputs->attitudeField & vn::protocol::uart::AttitudeGroup::ATTITUDEGROUP_DCM)
        {
            n_Quat_b = vnObs->attitudeOutputs->dcm.cast<double>();
        }
    }

    if ((_posVelSource == PosVelSource_Best || _posVelSource == PosVelSource_Ins)
        && vnObs->insOutputs && (vnObs->insOutputs->insStatus.mode() == 1 || vnObs->insOutputs->insStatus.mode() == 2))
    {
        if (vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSLLA)
        {
            lla_position = { deg2rad(vnObs->insOutputs->posLla(0)),
                             deg2rad(vnObs->insOutputs->posLla(1)),
                             vnObs->insOutputs->posLla(2) };
        }
        if (vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_POSECEF)
        {
            e_position = vnObs->insOutputs->posEcef;
        }

        if (vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELNED)
        {
            n_velocity = vnObs->insOutputs->velNed.cast<double>();
        }
        else if ((vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELECEF)
                 && (e_position.has_value() || lla_position.has_value()))
        {
            Eigen::Vector3d lla = lla_position.has_value() ? lla_position.value() : trafo::ecef2lla_WGS84(e_position.value());
            n_velocity = trafo::n_Quat_e(lla(0), lla(1)) * vnObs->insOutputs->velEcef.cast<double>();
        }
        else if ((vnObs->insOutputs->insField & vn::protocol::uart::InsGroup::INSGROUP_VELBODY)
                 && n_Quat_b.has_value())
        {
            n_velocity = n_Quat_b.value() * vnObs->insOutputs->velBody.cast<double>();
        }
    }

    if ((_posVelSource == PosVelSource_Best || _posVelSource == PosVelSource_Gnss1)
        && vnObs->gnss1Outputs && vnObs->gnss1Outputs->fix >= 2)
    {
        if (!e_position.has_value() && !lla_position.has_value())
        {
            if (vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                lla_position = { deg2rad(vnObs->gnss1Outputs->posLla(0)),
                                 deg2rad(vnObs->gnss1Outputs->posLla(1)),
                                 vnObs->gnss1Outputs->posLla(2) };
            }
            if (vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                e_position = vnObs->gnss1Outputs->posEcef;
            }
        }

        if (!n_velocity.has_value())
        {
            if (vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                n_velocity = vnObs->gnss1Outputs->velNed.cast<double>();
            }
            else if ((vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                     && (e_position.has_value() || lla_position.has_value()))
            {
                Eigen::Vector3d lla = lla_position.has_value() ? lla_position.value() : trafo::ecef2lla_WGS84(e_position.value());
                n_velocity = trafo::n_Quat_e(lla(0), lla(1)) * vnObs->gnss1Outputs->velEcef.cast<double>();
            }
        }
    }
    if ((_posVelSource == PosVelSource_Best || _posVelSource == PosVelSource_Gnss2)
        && vnObs->gnss2Outputs && vnObs->gnss2Outputs->fix >= 2)
    {
        if (!e_position.has_value() && !lla_position.has_value())
        {
            if (vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSLLA)
            {
                lla_position = { deg2rad(vnObs->gnss2Outputs->posLla(0)),
                                 deg2rad(vnObs->gnss2Outputs->posLla(1)),
                                 vnObs->gnss2Outputs->posLla(2) };
            }
            if (vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_POSECEF)
            {
                e_position = vnObs->gnss2Outputs->posEcef;
            }
        }

        if (!n_velocity.has_value())
        {
            if (vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELNED)
            {
                n_velocity = vnObs->gnss2Outputs->velNed.cast<double>();
            }
            else if ((vnObs->gnss2Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_VELECEF)
                     && (e_position.has_value() || lla_position.has_value()))
            {
                Eigen::Vector3d lla = lla_position.has_value() ? lla_position.value() : trafo::ecef2lla_WGS84(e_position.value());
                n_velocity = trafo::n_Quat_e(lla(0), lla(1)) * vnObs->gnss2Outputs->velEcef.cast<double>();
            }
        }
    }

    auto posVelAttObs = std::make_shared<PosVelAtt>();

    posVelAttObs->insTime = vnObs->insTime;

    if ((e_position.has_value() || lla_position.has_value()) && n_velocity.has_value())
    {
        if (e_position.has_value())
        {
            posVelAttObs->setPosition_e(e_position.value());
        }
        else
        {
            posVelAttObs->setPosition_lla(lla_position.value());
        }
        posVelAttObs->setVelocity_n(n_velocity.value());

        if (!n_Quat_b.has_value())
        {
            LOG_DEBUG("{}: Conversion succeeded but has no attitude info.", nameId());
        }
        else
        {
            posVelAttObs->setAttitude_n_Quat_b(n_Quat_b.value());
        }

        if (_posVelAtt__init == nullptr)
        {
            _posVelAtt__init = posVelAttObs;
        }

        if (_forceStatic)
        {
            posVelAttObs->setPosition_e(_posVelAtt__init->e_position());
            posVelAttObs->setVelocity_n(Eigen::Vector3d::Zero());
            posVelAttObs->setAttitude_n_Quat_b(_posVelAtt__init->n_Quat_b());
        }

        return posVelAttObs;
    }

    LOG_ERROR("{}: Conversion failed. No position or velocity data found in the input data.", nameId());
    return nullptr;
}