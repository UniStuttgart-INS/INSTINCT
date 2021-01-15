#include "KvhSensor.hpp"

#include "util/Logger.hpp"

#include "gui/widgets/HelpMarker.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/UartSensors/KVH/KvhUtilities.hpp"

#include "NodeData/IMU/KvhObs.hpp"

NAV::KvhSensor::KvhSensor()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    // TODO: Update the library to handle different baudrates
    selectedBaudrate = baudrate2Selection(Baudrate::BAUDRATE_921600);

    nm::CreateOutputPin(this, "KvhObs", Pin::Type::Flow, NAV::KvhObs::type());
}

NAV::KvhSensor::~KvhSensor()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::KvhSensor::typeStatic()
{
    return "KvhSensor";
}

std::string NAV::KvhSensor::type() const
{
    return typeStatic();
}

std::string NAV::KvhSensor::category()
{
    return "Data Provider";
}

void NAV::KvhSensor::guiConfig()
{
    if (ImGui::InputTextWithHint("SensorPort", "/dev/ttyUSB0", &sensorPort))
    {
        LOG_DEBUG("{}: SensorPort changed to {}", nameId(), sensorPort);
        flow::ApplyChanges();
        deinitialize();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("COM port where the sensor is attached to\n"
                             "- \"COM1\" (Windows format for physical and virtual (USB) serial port)\n"
                             "- \"/dev/ttyS1\" (Linux format for physical serial port)\n"
                             "- \"/dev/ttyUSB0\" (Linux format for virtual (USB) serial port)\n"
                             "- \"/dev/tty.usbserial-FTXXXXXX\" (Mac OS X format for virtual (USB) serial port)\n"
                             "- \"/dev/ttyS0\" (CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1)");
}

[[nodiscard]] json NAV::KvhSensor::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["UartSensor"] = UartSensor::save();

    return j;
}

void NAV::KvhSensor::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("UartSensor"))
    {
        UartSensor::restore(j.at("UartSensor"));
    }
}

bool NAV::KvhSensor::initialize()
{
    deinitialize();

    LOG_TRACE("{}: called", nameId());

    if (!Node::initialize())
    {
        return false;
    }

    // connect to the sensor
    try
    {
        sensor->connect(sensorPort, sensorBaudrate());

        LOG_DEBUG("{} connected on port {} with baudrate {}", nameId(), sensorPort, sensorBaudrate());
    }
    catch (...)
    {
        LOG_ERROR("{} could not connect", nameId());
        return false;
    }

    sensor->registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

    return isInitialized = true;
}

void NAV::KvhSensor::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!isInitialized)
    {
        return;
    }

    if (sensor->isConnected())
    {
        try
        {
            sensor->unregisterAsyncPacketReceivedHandler();
        }
        catch (...)
        {}
        sensor->disconnect();
    }

    Node::deinitialize();
}

void NAV::KvhSensor::asciiOrBinaryAsyncMessageReceived(void* userData, uart::protocol::Packet& p, [[maybe_unused]] size_t index)
{
    auto* kvhSensor = static_cast<KvhSensor*>(userData);

    if (p.type() == uart::protocol::Packet::Type::TYPE_BINARY)
    {
        auto obs = std::make_shared<KvhObs>(kvhSensor->imuPos, p);

        sensors::kvh::decryptKvhObs(obs);

        LOG_DATA("DATA({}): {}, {}, {}",
                 kvhSensor->name, obs->sequenceNumber, obs->temperature.value(), obs->status);

        // Check if a packet was skipped
        if (kvhSensor->prevSequenceNumber == UINT8_MAX)
        {
            kvhSensor->prevSequenceNumber = obs->sequenceNumber;
        }
        if (obs->sequenceNumber != 0 && (obs->sequenceNumber < kvhSensor->prevSequenceNumber || obs->sequenceNumber > kvhSensor->prevSequenceNumber + 2))
        {
            LOG_WARN("{}: Sequence Number changed from {} to {}", kvhSensor->name, kvhSensor->prevSequenceNumber, obs->sequenceNumber);
        }
        kvhSensor->prevSequenceNumber = obs->sequenceNumber;

        // Calls all the callbacks
        kvhSensor->invokeCallbacks(OutputPortIndex_KvhObs, obs);
    }
    else if (p.type() == uart::protocol::Packet::Type::TYPE_ASCII)
    {
        LOG_WARN("{}: Received an ASCII Async message: {}", kvhSensor->name, p.datastr());
    }
}