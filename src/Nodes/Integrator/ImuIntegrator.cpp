#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

NAV::ImuIntegrator::ImuIntegrator(const std::string& name, [[maybe_unused]] const std::map<std::string, std::string>& options)
    : Node(name)
{
}

void NAV::ImuIntegrator::integrateObservation(std::shared_ptr<NAV::ImuObs>& obs)
{
    // Get the current state data
    const auto& sourceNode = incomingLinks[1].first.lock();
    auto& sourcePortIndex = incomingLinks[1].second;
    auto currentStateData = std::static_pointer_cast<StateData>(sourceNode->requestOutputData(sourcePortIndex));

    // Fill if empty
    if (prevObs.empty())
    {
        prevObs.push_back(obs);
        prevObs.push_back(obs);
        prevStates.push_back(currentStateData);
        prevStates.push_back(currentStateData);
    }

    /// tₖ₋₂ Time at 2 previous epochs
    const auto& t__k2 = prevObs.at(1)->insTime.value();
    /// tₖ₋₁ Time at previous epoch
    const auto& t__k1 = prevObs.at(0)->insTime.value();
    /// tₖ Current Time
    const auto& t__k = obs->insTime.value();

    /// ω_ip_p (tₖ₋₁) Rotation rate in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    const auto& omega_ip_p__k1 = prevObs.at(0)->gyroUncompXYZ.value();
    /// ω_ip_p (tₖ) Rotation rate in [rad/s],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    const auto& omega_ip_p__k = obs->gyroUncompXYZ.value();

    // Δtₖ₋₁ = (tₖ₋₁ - tₖ₋₂) Time difference in [seconds]
    auto Dt__k1 = (t__k1 - t__k2).count();
    // Δtₖ = (tₖ - tₖ₋₁) Time difference in [seconds]
    auto Dt__k = (t__k - t__k1).count();

    /// Δα_ip_p (tₖ₋₁) The delta rotation angles in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d Dalpha_ip_p__k1 = Dt__k1 * omega_ip_p__k1;
    /// Δα_ip_p (tₖ) The delta rotation angles in [radian],
    /// of the inertial to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d Dalpha_ip_p__k = Dt__k * omega_ip_p__k;

    /// C_ep (tₖ₋₂) Direction Cosine Matrix at the time tₖ₋₂
    const auto& DCM_e2p__k2 = prevStates.at(1)->DCM_e2p();
    /// C_ep (tₖ₋₁) Direction Cosine Matrix at the time tₖ₋₁
    const auto& DCM_e2p__k1 = prevStates.at(0)->DCM_e2p();

    /// ω_ie_e (tₖ) Rotation rate in [rad/s],
    /// of the inertial to earth system, in platform coordinates, at the time tₖ
    Eigen::Vector3d omega_ie_e = Eigen::Vector3d::Ones();

    /// Δβ⁠_ep_p (tₖ₋₁) The delta rotation angles in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ₋₁
    Eigen::Vector3d Dbeta_ep_p__k1 = Dalpha_ip_p__k1 - DCM_e2p__k2 * omega_ie_e * Dt__k1;
    /// Δβ⁠_ep_p (tₖ) The delta rotation angles in [radian],
    /// of the earth to platform system, in platform coordinates, at the time tₖ
    Eigen::Vector3d Dbeta_ep_p__k = Dalpha_ip_p__k - DCM_e2p__k1 * omega_ie_e * Dt__k;

    LOG_INFO("Δ⁠β⁠_ep_p__tₖ₋₁: {}", Dbeta_ep_p__k1);
    LOG_INFO("Δβ⁠_ep_p__tₖ: {}", Dbeta_ep_p__k);
    // auto dt = 2 * delta_t;

    // omega_ep^p = omega_ip^p - C_e^p * omega_ie^e

    // Rotate Data
    if (prevObs.size() == 2)
    {
        prevObs.pop_back();
        prevStates.pop_back();
    }
    prevObs.push_front(obs);
    prevStates.push_front(currentStateData);

    invokeCallbacks(obs);
}