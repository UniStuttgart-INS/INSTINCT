#include "State.hpp"

NAV::State::State(const std::string& name, const std::map<std::string, std::string>& options)
    : Node(name)
{
    initialState = std::make_shared<StateData>();
    currentState = std::make_shared<StateData>();

    if (options.count("Init LatLonAlt"))
    {
        std::stringstream lineStream(options.at("Init LatLonAlt"));
        std::string value;
        if (std::getline(lineStream, value, ';'))
        {
            initialState->latitude() = trafo::deg2rad(std::stod(value));
            if (std::getline(lineStream, value, ';'))
            {
                initialState->longitude() = trafo::deg2rad(std::stod(value));
                if (std::getline(lineStream, value, ';'))
                {
                    initialState->height() = std::stod(value);
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
        initialState->quat_nb_coeff() = trafo::quat_nb(roll, pitch, yaw).coeffs();
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
        initialState->velocity_n() = Eigen::Vector3d(v_n, v_e, v_d);
    }

    currentState->X = initialState->X;
}

void NAV::State::updateState(std::shared_ptr<StateData>& state)
{
    // Process the data here
    currentState = state;

    // For now does not invoke callbacks, as the state is used on request basis
    // invokeCallbacks(currentState);
}