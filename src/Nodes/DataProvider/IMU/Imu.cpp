#include "Imu.hpp"

#include <sstream>
#include "util/Logger.hpp"
#include "util/InsTransformations.hpp"

NAV::Imu::Imu(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : Node(name)
{
    imuPos = std::make_shared<ImuPos>();

    if (options.count("Accel pos"))
    {
        std::stringstream lineStream(options.at("Accel pos"));
        std::string value;
        if (std::getline(lineStream, value, ';'))
        {
            imuPos->posAccel_b.x() = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                imuPos->posAccel_b.y() = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    imuPos->posAccel_b.z() = std::stod(value);
                }
            }
        }
    }
    if (options.count("Gyro pos"))
    {
        std::stringstream lineStream(options.at("Gyro pos"));
        std::string value;
        if (std::getline(lineStream, value, ';'))
        {
            imuPos->posGyro_b.x() = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                imuPos->posGyro_b.y() = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    imuPos->posGyro_b.z() = std::stod(value);
                }
            }
        }
    }
    if (options.count("Mag pos"))
    {
        std::stringstream lineStream(options.at("Mag pos"));
        std::string value;
        if (std::getline(lineStream, value, ';'))
        {
            imuPos->posMag_b.x() = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                imuPos->posMag_b.y() = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    imuPos->posMag_b.z() = std::stod(value);
                }
            }
        }
    }
    if (options.count("Accel rot"))
    {
        std::stringstream lineStream(options.at("Accel rot"));
        std::string value;
        double rotX{};
        double rotY{};
        double rotZ{};
        if (std::getline(lineStream, value, ';'))
        {
            rotX = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                rotY = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    rotZ = std::stod(value);
                }
            }
        }
        imuPos->quatAccel_p2b = trafo::quat_p2b(rotX, rotY, rotZ);
    }
    if (options.count("Gyro rot"))
    {
        std::stringstream lineStream(options.at("Gyro rot"));
        std::string value;
        double rotX{};
        double rotY{};
        double rotZ{};
        if (std::getline(lineStream, value, ';'))
        {
            rotX = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                rotY = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    rotZ = std::stod(value);
                }
            }
        }
        imuPos->quatGyro_p2b = trafo::quat_p2b(rotX, rotY, rotZ);
    }
    if (options.count("Mag rot"))
    {
        std::stringstream lineStream(options.at("Mag rot"));
        std::string value;
        double rotX{};
        double rotY{};
        double rotZ{};
        if (std::getline(lineStream, value, ';'))
        {
            rotX = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                rotY = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    rotZ = std::stod(value);
                }
            }
        }
        imuPos->quatMag_p2b = trafo::quat_p2b(rotX, rotY, rotZ);
    }
}