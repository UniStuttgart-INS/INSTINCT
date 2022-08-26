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
    if (ImGui::Combo(fmt::format("Output Type##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_outputType), "ImuObsWDelta\0PosVelAtt\0GnssObs\0\0"))
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
        else if (_outputType == OutputType_GnssObs)
        {
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::GnssObs::type() };
            outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::GnssObs::type();
        }

        for (auto& link : outputPins.front().links)
        {
            if (auto* connectedPin = link.getConnectedPin())
            {
                outputPins.front().recreateLink(*connectedPin);
            }
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
            else if (_outputType == OutputType_GnssObs)
            {
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).dataIdentifier = { NAV::GnssObs::type() };
                outputPins.at(OUTPUT_PORT_INDEX_CONVERTED).name = NAV::GnssObs::type();
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

void NAV::VectorNavBinaryConverter::receiveObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::PinId /*pinId*/)
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
    else if (_outputType == OutputType_GnssObs)
    {
        convertedData = convert2GnssObs(vnObs);
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

std::shared_ptr<const NAV::GnssObs> NAV::VectorNavBinaryConverter::convert2GnssObs(const std::shared_ptr<const VectorNavBinaryOutput>& vnObs)
{
    auto gnssObs = std::make_shared<GnssObs>();

    gnssObs->insTime = vnObs->insTime;

    if (vnObs->gnss1Outputs)
    {
        if (vnObs->gnss1Outputs->gnssField & vn::protocol::uart::GpsGroup::GPSGROUP_RAWMEAS)
        {
            for (const auto& satRaw : vnObs->gnss1Outputs->raw.satellites)
            {
                bool skipMeasurement = false;
                SatelliteSystem satSys = SatSys_None;
                switch (satRaw.sys)
                {
                case vendor::vectornav::SatSys::GPS:
                    satSys = GPS;
                    break;
                case vendor::vectornav::SatSys::SBAS:
                    satSys = SBAS;
                    break;
                case vendor::vectornav::SatSys::Galileo:
                    satSys = GAL;
                    break;
                case vendor::vectornav::SatSys::BeiDou:
                    satSys = BDS;
                    break;
                case vendor::vectornav::SatSys::IMES:
                    LOG_TRACE("VectorNav SatRawElement satellite system '{}' is not supported yet. Skipping measurement.", satRaw.sys);
                    skipMeasurement = true;
                    break;
                case vendor::vectornav::SatSys::QZSS:
                    satSys = QZSS;
                    break;
                case vendor::vectornav::SatSys::GLONASS:
                    satSys = GLO;
                    break;
                default: // IRNSS not in vectorNav
                    LOG_TRACE("VectorNav SatRawElement satellite system '{}' is not supported yet. Skipping measurement.", satRaw.sys);
                    skipMeasurement = true;
                    break;
                }

                Frequency frequency = Freq_None;
                Code code;
                switch (SatelliteSystem_(satSys))
                {
                case GPS:
                    switch (satRaw.freq)
                    {
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L1:
                        frequency = G01;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::P_Code:
                            code = Code::G1P;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::G1C;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Y_Code:
                            code = Code::G1Y;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::M_Code:
                            code = Code::G1M;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Codeless:
                            code = Code::G1N;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::M_Chan:
                            code = Code::G1S;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::L_Chan:
                            code = Code::G1L;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::G1X;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Z_Tracking:
                            code = Code::G1W;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L2:
                        frequency = G02;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::P_Code:
                            code = Code::G2P;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::G2C;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::SemiCodeless:
                            code = Code::G2D;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Y_Code:
                            code = Code::G2Y;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::M_Code:
                            code = Code::G2M;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Codeless:
                            code = Code::G2N;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::M_Chan:
                            code = Code::G2S;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::L_Chan:
                            code = Code::G2L;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::G2X;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Z_Tracking:
                            code = Code::G2W;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L5:
                        frequency = G05;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::G5I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::G5Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::G5X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    default:
                        LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq);
                        skipMeasurement = true;
                        break;
                    }
                    break;
                case SBAS:
                    switch (satRaw.freq)
                    {
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L1:
                        frequency = S01;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::S1C;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L5:
                        frequency = S05;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::S5I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::S5Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::S5X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    default:
                        LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq);
                        skipMeasurement = true;
                        break;
                    }
                    break;
                case GAL:
                    switch (satRaw.freq)
                    {
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L1:
                        frequency = E01;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::E1C;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::A_Chan:
                            code = Code::E1A;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::B_Chan:
                            code = Code::E1B;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::E1X;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::ABC:
                            code = Code::E1Z;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L5:
                        frequency = E08;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::E8I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::E8Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::E8X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::E6:
                        frequency = E06;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::B_Chan:
                            code = Code::E6B;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::E6C;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::E6X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::E5a:
                        frequency = E05;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::E5I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::E5Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::E5X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::E5b:
                        frequency = E07;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::E7I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::E7Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::E7X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    default:
                        LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq);
                        skipMeasurement = true;
                        break;
                    }
                    break;
                case BDS:
                    switch (satRaw.freq)
                    {
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L1:
                        frequency = B01;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::B2I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::B2Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::B2X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::E6:
                        frequency = B06;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::A_Chan:
                            code = Code::B6A;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::B6I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::B6Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::B6X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::E5b:
                        frequency = B08;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::B7I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::B7Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::B7X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    default:
                        LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq);
                        skipMeasurement = true;
                        break;
                    }
                    break;
                case QZSS:
                    switch (satRaw.freq)
                    {
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L1:
                        frequency = J01;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::J1C;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::M_Chan:
                            code = Code::J1S;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::L_Chan:
                            code = Code::J1L;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::J1X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L2:
                        frequency = J02;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::M_Chan:
                            code = Code::J2S;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::L_Chan:
                            code = Code::J2L;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::J2X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L5:
                        frequency = J05;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::I_Chan:
                            code = Code::J5I;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::Q_Chan:
                            code = Code::J5Q;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::J5X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::E6:
                        frequency = J06;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::M_Chan:
                            code = Code::J6S;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::L_Chan:
                            code = Code::J6L;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::BC_Chan:
                            code = Code::J6X;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    default:
                        LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq);
                        skipMeasurement = true;
                        break;
                    }
                    break;
                case GLO:
                    switch (satRaw.freq)
                    {
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L1:
                        frequency = R01;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::R1C;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::P_Code:
                            code = Code::R1P;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    case vendor::vectornav::RawMeas::SatRawElement::Freq::L2:
                        frequency = R02;
                        switch (satRaw.chan)
                        {
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::CA_Code:
                            code = Code::R2C;
                            break;
                        case vendor::vectornav::RawMeas::SatRawElement::Chan::P_Code:
                            code = Code::R2P;
                            break;
                        default:
                            LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' channel '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq, satRaw.chan);
                            skipMeasurement = true;
                            break;
                        }
                        break;
                    default:
                        LOG_TRACE("VectorNav SatRawElement satellite system '{}' frequency '{}' is not supported yet. Skipping measurement.", satRaw.sys, satRaw.freq);
                        skipMeasurement = true;
                        break;
                    }
                    break;
                case IRNSS: // IRNSS not in vectorNav
                case SatSys_None:
                    skipMeasurement = true;
                    break;
                }

                if (skipMeasurement)
                {
                    continue;
                }

                (*gnssObs)(frequency, satRaw.svId, code).pseudorange = satRaw.pr;
                (*gnssObs)(frequency, satRaw.svId, code).carrierPhase = satRaw.cp;
                (*gnssObs)(frequency, satRaw.svId, code).doppler = satRaw.dp;
                (*gnssObs)(frequency, satRaw.svId, code).CN0 = satRaw.cno;

                // LLI has not been implemented yet, but can be calculated from vendor::vectornav::RawMeas::SatRawElement::Flags
                // (*gnssObs)[{ frequency, satRaw.svId }].LLI = ...
            }
        }
    }

    return gnssObs;
}