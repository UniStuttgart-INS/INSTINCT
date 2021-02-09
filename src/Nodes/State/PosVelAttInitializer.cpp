#include "PosVelAttInitializer.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/UartSensors/Ublox/UbloxTypes.hpp"
#include "util/InsTransformations.hpp"

NAV::PosVelAttInitializer::PosVelAttInitializer()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &PosVelAttInitializer::receiveImuObs);
    nm::CreateInputPin(this, "GnssObs", Pin::Type::Flow, { NAV::UbloxObs::type(), NAV::RtklibPosObs::type() }, &PosVelAttInitializer::receiveGnssObs);
    nm::CreateInputPin(this, "Position ECEF", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Velocity NED", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });
    nm::CreateInputPin(this, "Quaternion nb", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" });

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow);
    nm::CreateOutputPin(this, "GnssObs", Pin::Type::Flow);
}

NAV::PosVelAttInitializer::~PosVelAttInitializer()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::PosVelAttInitializer::typeStatic()
{
    return "PosVelAttInitializer";
}

std::string NAV::PosVelAttInitializer::type() const
{
    return typeStatic();
}

std::string NAV::PosVelAttInitializer::category()
{
    return "State";
}

void NAV::PosVelAttInitializer::guiConfig()
{
    if (ImGui::InputDouble("Initialization Duration", &initDuration, 0.0, 0.0, "%.3f s"))
    {
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::PosVelAttInitializer::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["initDuration"] = initDuration;

    return j;
}

void NAV::PosVelAttInitializer::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("initDuration"))
    {
        j.at("initDuration").get_to(initDuration);
    }
}

bool NAV::PosVelAttInitializer::onCreateLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    if (endPin)
    {
        if (endPin->id == inputPins.at(InputPortIndex_Position).id)
        {
            determinePosition = InitFlag::CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Velocity).id)
        {
            determineVelocity = InitFlag::CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Attitude).id)
        {
            determineAttitude = InitFlag::CONNECTED;
        }
    }

    return true;
}

void NAV::PosVelAttInitializer::onDeleteLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    if (endPin)
    {
        if (endPin->id == inputPins.at(InputPortIndex_Position).id)
        {
            determinePosition = InitFlag::NOT_CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Velocity).id)
        {
            determineVelocity = InitFlag::NOT_CONNECTED;
        }
        else if (endPin->id == inputPins.at(InputPortIndex_Attitude).id)
        {
            determineAttitude = InitFlag::NOT_CONNECTED;
        }
    }
}

bool NAV::PosVelAttInitializer::initialize()
{
    LOG_TRACE("{}: called", nameId());

    countAveragedVelocity = 0.0;
    countAveragedAttitude = 0.0;

    if (determinePosition == InitFlag::INITIALIZED)
    {
        determinePosition = InitFlag::CONNECTED;
    }
    if (determineVelocity == InitFlag::INITIALIZED)
    {
        determineVelocity = InitFlag::CONNECTED;
    }
    if (determineAttitude == InitFlag::INITIALIZED)
    {
        determineAttitude = InitFlag::CONNECTED;
    }

    return true;
}

void NAV::PosVelAttInitializer::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::PosVelAttInitializer::finalizeInit()
{
    LOG_TRACE("{}: called", nameId());

    if (determinePosition != InitFlag::CONNECTED
        && determineVelocity != InitFlag::CONNECTED
        && determineAttitude != InitFlag::CONNECTED)
    {
        if (determinePosition == InitFlag::INITIALIZED)
        {
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
                    {
                        auto latLonAlt = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix->data(), 3));
                        LOG_INFO("{}: Position initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", nameId(),
                                 trafo::rad2deg(latLonAlt.x()),
                                 trafo::rad2deg(latLonAlt.y()),
                                 latLonAlt.z());
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Position))
                    {
                        auto matrix = (*value)();

                        auto latLonAlt = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data(), 3));
                        LOG_INFO("{}: Position initialized to Lat {:3.4f} [°], Lon {:3.4f} [°], Alt {:4.4f} [m]", nameId(),
                                 trafo::rad2deg(latLonAlt.x()), trafo::rad2deg(latLonAlt.y()), latLonAlt.z());
                    }
                }
            }
        }
        if (determineVelocity == InitFlag::INITIALIZED)
        {
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Velocity).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Velocity))
                    {
                        LOG_INFO("{}: State initialized to v_N {:3.5f}, v_E {:3.5f}, v_D {:3.5f} [m/s]", nameId(),
                                 (*matrix)(0, 0), (*matrix)(1, 0), (*matrix)(2, 0));
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Velocity))
                    {
                        auto matrix = (*value)();
                        LOG_INFO("{}: State initialized to v_N {:3.5f}, v_E {:3.5f}, v_D {:3.5f} [m/s]", nameId(),
                                 matrix(0, 0), matrix(1, 0), matrix(2, 0));
                    }
                }
            }
        }
        if (determineAttitude == InitFlag::INITIALIZED)
        {
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Attitude).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Attitude))
                    {
                        LOG_INFO("{}: State initialized to Roll {:3.5f}, Pitch {:3.5f}, Yaw {:3.4f} [°]", nameId(),
                                 trafo::rad2deg((*matrix)(0, 0)),
                                 trafo::rad2deg((*matrix)(1, 0)),
                                 trafo::rad2deg((*matrix)(2, 0)));
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Attitude))
                    {
                        auto matrix = (*value)();
                        LOG_INFO("{}: State initialized to Roll {:3.5f}, Pitch {:3.5f}, Yaw {:3.4f} [°]", nameId(),
                                 trafo::rad2deg(matrix(0, 0)),
                                 trafo::rad2deg(matrix(1, 0)),
                                 trafo::rad2deg(matrix(2, 0)));
                    }
                }
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);

    if (!obs->insTime.has_value())
    {
        LOG_ERROR("{}: Can only process data with an insTime", nameId());
        return;
    }

    if (startTime.empty())
    {
        startTime = obs->insTime.value();
    }
}

void NAV::PosVelAttInitializer::receiveGnssObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId)
{
    if (determinePosition != InitFlag::CONNECTED
        && determineVelocity != InitFlag::CONNECTED
        && determineAttitude != InitFlag::CONNECTED)
    {
        invokeCallbacks(OutputPortIndex_GnssObs, nodeData);
        return;
    }

    if (Link* link = nm::FindLink(linkId))
    {
        if (Pin* sourcePin = nm::FindPin(link->startPinId))
        {
            if (sourcePin->dataIdentifier.front() == RtklibPosObs::type())
            {
                receiveRtklibPosObs(std::static_pointer_cast<RtklibPosObs>(nodeData));
            }
            else if (sourcePin->dataIdentifier.front() == UbloxObs::type())
            {
                receiveUbloxObs(std::static_pointer_cast<UbloxObs>(nodeData));
            }
        }
    }
}

void NAV::PosVelAttInitializer::receiveUbloxObs(const std::shared_ptr<UbloxObs>& obs)
{
    if (obs->msgClass == sensors::ublox::UbxClass::UBX_CLASS_NAV)
    {
        auto msgId = static_cast<sensors::ublox::UbxNavMessages>(obs->msgId);
        if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_ATT)
        {
            LOG_DEBUG("{}: UBX_NAV_ATT: Roll {}, Pitch {}, Heading {} [deg]", nameId(),
                      std::get<sensors::ublox::UbxNavAtt>(obs->data).roll * 1e-5,
                      std::get<sensors::ublox::UbxNavAtt>(obs->data).pitch * 1e-5,
                      std::get<sensors::ublox::UbxNavAtt>(obs->data).heading * 1e-5);
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSECEF)
        {
            LOG_DEBUG("{}: UBX_NAV_POSECEF: ECEF {}, {}, {} [m]", nameId(),
                      std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefX * 1e-2,
                      std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefY * 1e-2,
                      std::get<sensors::ublox::UbxNavPosecef>(obs->data).ecefZ * 1e-2);
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_POSLLH)
        {
            Eigen::Vector3d latLonAlt(trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lat * 1e-7),
                                      trafo::deg2rad(std::get<sensors::ublox::UbxNavPosllh>(obs->data).lon * 1e-7),
                                      std::get<sensors::ublox::UbxNavPosllh>(obs->data).height * 1e-3);

            LOG_DEBUG("{}: UBX_NAV_POSLLH: ECEF {}, {}, {} [m]", nameId(),
                      trafo::lla2ecef_WGS84(latLonAlt).x(),
                      trafo::lla2ecef_WGS84(latLonAlt).y(),
                      trafo::lla2ecef_WGS84(latLonAlt).z());
        }
        else if (msgId == sensors::ublox::UbxNavMessages::UBX_NAV_VELNED)
        {
            LOG_DEBUG("{}: UBX_NAV_VELNED: {}, {}, {} [cm/s], {} [deg]", nameId(),
                      std::get<sensors::ublox::UbxNavVelned>(obs->data).velN,
                      std::get<sensors::ublox::UbxNavVelned>(obs->data).velE,
                      std::get<sensors::ublox::UbxNavVelned>(obs->data).velD,
                      std::get<sensors::ublox::UbxNavVelned>(obs->data).heading * 1e-5);
        }
    }
}

void NAV::PosVelAttInitializer::receiveRtklibPosObs(const std::shared_ptr<RtklibPosObs>& obs)
{
    if (determinePosition != InitFlag::NOT_CONNECTED && obs->position_ecef.has_value())
    {
        if (obs->sdXYZ.has_value())
        {
            positionAccuracyFullfilled.set(0, obs->sdXYZ->x() <= positionAccuracyThreshold);
            positionAccuracyFullfilled.set(1, obs->sdXYZ->y() <= positionAccuracyThreshold);
            positionAccuracyFullfilled.set(2, obs->sdXYZ->z() <= positionAccuracyThreshold);
        }
        else if (obs->sdNEU.has_value())
        {
            positionAccuracyFullfilled.set(0, obs->sdNEU->x() <= positionAccuracyThreshold);
            positionAccuracyFullfilled.set(1, obs->sdNEU->y() <= positionAccuracyThreshold);
            positionAccuracyFullfilled.set(2, obs->sdNEU->z() <= positionAccuracyThreshold);
        }
        if (positionAccuracyFullfilled.all())
        {
            if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Position).id))
            {
                if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
                {
                    if (auto* matrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Position))
                    {
                        (*matrix)(0, 0) = obs->position_ecef->x();
                        (*matrix)(1, 0) = obs->position_ecef->y();
                        (*matrix)(2, 0) = obs->position_ecef->z();
                        determinePosition = InitFlag::INITIALIZED;
                        finalizeInit();
                    }
                }
                else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
                {
                    if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Position))
                    {
                        auto matrix = (*value)();
                        matrix(0, 0) = obs->position_ecef->x();
                        matrix(1, 0) = obs->position_ecef->y();
                        matrix(2, 0) = obs->position_ecef->z();
                        determinePosition = InitFlag::INITIALIZED;
                        finalizeInit();
                    }
                }
            }
        }
    }
}
