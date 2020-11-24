#include "Imu.hpp"

#include <sstream>
#include "util/Logger.hpp"
#include "util/InsTransformations.hpp"

NAV::Imu::Imu(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : Node(name)
{
    if (options.count("Accel pos"))
    {
        std::stringstream lineStream(options.at("Accel pos"));
        std::string value;
        if (std::getline(lineStream, value, ';'))
        {
            imuPos.positionAccel_b.x() = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                imuPos.positionAccel_b.y() = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    imuPos.positionAccel_b.z() = std::stod(value);
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
            imuPos.positionGyro_b.x() = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                imuPos.positionGyro_b.y() = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    imuPos.positionGyro_b.z() = std::stod(value);
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
            imuPos.positionMag_b.x() = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                imuPos.positionMag_b.y() = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    imuPos.positionMag_b.z() = std::stod(value);
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
        imuPos.quaternionAccel_bp = trafo::quat_bp(trafo::deg2rad(rotX), trafo::deg2rad(rotY), trafo::deg2rad(rotZ));
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
        imuPos.quaternionGyro_bp = trafo::quat_bp(trafo::deg2rad(rotX), trafo::deg2rad(rotY), trafo::deg2rad(rotZ));
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
        imuPos.quaternionMag_bp = trafo::quat_bp(trafo::deg2rad(rotX), trafo::deg2rad(rotY), trafo::deg2rad(rotZ));
    }
}