#include "State.hpp"

NAV::State::State(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    currentState = std::make_shared<StateData>();

    if (options.count("Init LatLonAlt"))
    {
        std::stringstream lineStream(options.at("Init LatLonAlt"));
        std::string value;
        if (std::getline(lineStream, value, ';'))
        {
            currentState->latitude() = trafo::deg2rad(std::stod(value));
            if (std::getline(lineStream, value, ';'))
            {
                currentState->longitude() = trafo::deg2rad(std::stod(value));
                if (std::getline(lineStream, value, ';'))
                {
                    currentState->height() = std::stod(value);
                }
            }
        }
    }
    if (options.count("Init RollPitchYaw"))
    {
        std::stringstream lineStream(options.at("Init RollPitchYaw"));
        std::string value;
        double roll{};
        double pitch{};
        double yaw{};
        if (std::getline(lineStream, value, ';'))
        {
            roll = trafo::deg2rad(std::stod(value));
            if (std::getline(lineStream, value, ';'))
            {
                pitch = trafo::deg2rad(std::stod(value));
                if (std::getline(lineStream, value, ';'))
                {
                    yaw = trafo::deg2rad(std::stod(value));
                }
            }
        }
        currentState->quat_b2n_coeff() = trafo::quat_b2n(roll, pitch, yaw).coeffs();
    }
    if (options.count("Init Velocity"))
    {
        std::stringstream lineStream(options.at("Init Velocity"));
        std::string value;
        double v_n{};
        double v_e{};
        double v_d{};
        if (std::getline(lineStream, value, ';'))
        {
            v_n = std::stod(value);
            if (std::getline(lineStream, value, ';'))
            {
                v_e = std::stod(value);
                if (std::getline(lineStream, value, ';'))
                {
                    v_d = std::stod(value);
                }
            }
        }
        currentState->velocity_n() = Eigen::Vector3d(v_n, v_e, v_d);
    }
}

void NAV::State::updateState(std::shared_ptr<StateData>& state)
{
    // Process the data here
    currentState = state;

    // For now does not invoke callbacks, as the state is used on request basis
    // invokeCallbacks(currentState);
}