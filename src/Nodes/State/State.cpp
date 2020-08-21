#include "State.hpp"

NAV::State::State(const std::string& name, const std::map<std::string, std::string>& /* options */)
    : Node(name) {}

void NAV::State::updateState(std::shared_ptr<StateData>& state)
{
    // Process the data here
    currentState = state;

    invokeCallbacks(currentState);
}
