#include "ARMA.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include <Eigen/Dense>
#include <iterator>

NAV::ARMA::ARMA()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ARMA::receiveImuObs);

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
}

NAV::ARMA::~ARMA()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ARMA::typeStatic()
{
    return "ARMA";
}

std::string NAV::ARMA::type() const
{
    return typeStatic();
}

std::string NAV::ARMA::category()
{
    return "DataProcessor";
}

void NAV::ARMA::guiConfig()
{
    ImGui::Checkbox("calculate ACF", &ACF_CHECK);
    ImGui::Checkbox("calculate PACF", &PACF_CHECK);
    ImGui::InputInt("Deque size", &deque_size);
    ImGui::SameLine();
}

[[nodiscard]] json NAV::ARMA::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    // j["outputFrequency"] = outputFrequency;

    return j;
}

void NAV::ARMA::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("outputFrequency"))
    {
        // j.at("outputFrequency").get_to(outputFrequency);
    }
}

bool NAV::ARMA::initialize()
{
    LOG_TRACE("{}: called", nameId());
    // acf
    acf = Eigen::VectorXd::Zero(deque_size);

    // pacf
    pacf = Eigen::VectorXd::Zero(deque_size - 1);
    e_hat_initial = Eigen::VectorXd::Zero(deque_size);

    // arma
    A = Eigen::MatrixXd::Zero(deque_size - std::max(p, q), p + q);
    y = Eigen::MatrixXd::Zero(deque_size, 3);
    y_hat = Eigen::MatrixXd::Zero(deque_size, 3);
    x = Eigen::VectorXd::Zero(p + q);
    e_hat = Eigen::VectorXd::Zero(deque_size);
    return true;
}

void NAV::ARMA::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

/// @brief calculate autocorrelation function (ACF)
/// @param[in] y vector of data
void acf_function(const Eigen::VectorXd& y, bool& ACF_CHECK, int p, Eigen::VectorXd& acf)
{
    int tt = 0; // acf loop iterator
    int tau = 0;
    int acf_size = static_cast<int>(y.size());

    double cov = 0.0;
    double var = 0.0;
    double mean = y.mean();

    for (tt = 0; tt < acf_size; tt++)
    {
        var += (y(tt) - mean) * (y(tt) - mean); // sum of variance
    }
    if (ACF_CHECK)
    { //box checker
        acf_size = p + 2;
    }
    for (tau = 0; tau < acf_size; tau++)
    { // acf: correlation to delta_tau

        for (tt = 0; tt < acf_size - tau; tt++)
        {
            cov += (y(tt) - mean) * (y(tt + tau) - mean); // 'cov' sum
        }
        acf(tau) = cov / var;
        cov = 0; // reset 'cov' for next sum
    }
}

/// @brief calculate partial autocorrelation function (PACF)
/// @param[in] y vector of data @param[in] acf vector of acf @param[in] p order of AR process
/// @param[out] pacf vector of pacf values @param[out] initial_e_hat vector of initial ê for Hannan-Rissanen
void pacf_function(const Eigen::VectorXd& y, Eigen::VectorXd& acf, int p, bool& PACF_CHECK, Eigen::VectorXd& pacf, Eigen::VectorXd& e_hat_initial)
{
    Eigen::VectorXd phi_tau;
    Eigen::VectorXd phi_tau_i;
    Eigen::VectorXd phi_initial;

    phi_tau = Eigen::VectorXd::Zero(y.size());
    phi_tau_i = Eigen::VectorXd::Zero(y.size());
    phi_initial = Eigen::VectorXd::Zero(p + 1);

    int ii = 0;
    int pacf_size = static_cast<int>(y.size());

    double sum1 = 0.0;
    double sum2 = 0.0;
    double sum3 = 0.0;

    // PACF while E(eta_t) = 0
    // first iteration step
    pacf(0) = acf(1); // initial value phi_1_1 = p(1)
    phi_tau_i(0) = pacf(0);
    sum1 = acf(1) * acf(1);
    sum2 = acf(1) * acf(1);

    e_hat_initial(0) = 0;

    for (int tau = 2; tau < pacf_size; tau++)
    {
        // Hannan-Rissanen initial phi
        if (tau == p + 2)
        {
            phi_initial = phi_tau_i;
        }
        if (tau < p + 2 || PACF_CHECK) // skip this for tau > p + 2 if PACF CHECK is false
        {
            pacf(tau - 1) = (acf(tau) - sum1) / (1 - sum2);
            sum1 = 0;
            sum2 = 0;

            for (ii = 0; ii < tau - 1; ii++)
            {
                phi_tau(ii) = (phi_tau_i(ii) - pacf(tau - 1) * phi_tau_i(tau - ii - 2));
                sum1 += phi_tau(ii) * acf(tau - ii); // top sum
                sum2 += phi_tau(ii) * acf(ii + 1);   // bottom sum
            }
            phi_tau_i = phi_tau; // last element of phi_tau_i -> phi_tau_tau
            phi_tau_i(tau - 1) = pacf(tau - 1);
            sum1 += pacf(tau - 1) * acf(1);
            sum2 += pacf(tau - 1) * acf(tau);
        }

        // Hannan-Rissanen initial e_hat
        if (tau > p + 1)
        {
            for (int m = 0; m < p + 1; m++)
            {
                sum3 += y(tau - m - 2) * phi_initial(m);
            }
            e_hat_initial(tau - 1) = y(tau - 1) - sum3;
            sum3 = 0;
        }
        else
        {
            e_hat_initial(tau - 1) = 0;
        }
    }

    for (int m = 0; m < p + 1; m++)
    { // e_hat at Y_n
        sum3 += y(pacf_size - m - 2) * phi_initial(m);
    }
    e_hat_initial(pacf_size - 1) = y(pacf_size - 1) - sum3;
}

/// @brief fill A matrix for Hannan-Rissanen
/// @param[in] y vector of data @param[in] e_hat_initial for least squares @param[in] p order of AR process @param[in] q order of MA process
void matrix_function(const Eigen::VectorXd& y, const Eigen::VectorXd& e_hat_initial, int p, int q, Eigen::MatrixXd& A)
{
    for (int t_HR = p; t_HR < y.size(); t_HR++)
    {
        for (int i_HR = 0; i_HR < p; i_HR++)
        {
            A(t_HR - p, i_HR) = y(t_HR - i_HR - 1); // AR
        }
        for (int i_HR = p; i_HR < p + q; i_HR++)
        {
            A(t_HR - p, i_HR) = -e_hat_initial(t_HR - i_HR + p - 1); // MA
        }
    }
}

void NAV::ARMA::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);
    auto newImuObs = std::make_shared<ImuObs>(obs->imuPos); // nicht in jeder schleife?
    buffer.push_back(obs);
    if (static_cast<int>(buffer.size()) == deque_size)
    {
        k = 0;
        for (auto& obs : buffer)
        {
            const Eigen::Vector3d acc = obs->accelUncompXYZ.value();
            y(k, 0) = acc(0);
            y(k, 1) = acc(1);
            y(k, 2) = acc(2);
            k++;
        }
        for (int obs_num = 0; obs_num < 3; obs_num++) // number of observations (e.g. acceleration (x,y,z))
        {
            // calculate acf
            acf_function(y.col(obs_num), ACF_CHECK, p, acf);

            // calculate pacf & initial ê
            pacf_function(y.col(obs_num), acf, p, PACF_CHECK, pacf, e_hat_initial);

            // arma process
            double sum_HR = 0.0;

            for (int it = 0; it < 2; it++)
            {
                matrix_function(y.col(obs_num), e_hat_initial, p, q, A);

                x = (A.transpose() * A).ldlt().solve(A.transpose() * y.col(obs_num).tail(deque_size - p)); // t > max(p, q)
                e_hat = e_hat_initial;                                                                     // e_hat copy

                for (int i_HR = 0; i_HR < p; i_HR++)
                {                            // calculate e_hat for new variables
                    e_hat_initial(i_HR) = 0; // t <= max(p,q) = 0
                    y_hat(i_HR, obs_num) = y(i_HR, obs_num);
                }
                for (int t_HR = p; t_HR < deque_size; t_HR++)
                { // AR
                    for (int i_HR = 0; i_HR < p; i_HR++)
                    {
                        sum_HR += x(i_HR) * y(t_HR - i_HR - 1, obs_num);
                    }
                    for (int i_HR = p; i_HR < p + q; i_HR++)
                    { // MA
                        sum_HR -= x(i_HR) * e_hat(t_HR - i_HR + p - 1);
                    }
                    e_hat_initial(t_HR) = y(t_HR, obs_num) - sum_HR;
                    y_hat(t_HR, obs_num) = sum_HR;
                    sum_HR = 0;
                }
            }
        }
        LOG_TRACE("{}: called {}", nameId(), obs->insTime->GetStringOfDate());
        newImuObs->insTime = obs->insTime.value();
        newImuObs->accelUncompXYZ = Eigen::Vector3d(y_hat(deque_size - 1, 0), y_hat(deque_size - 1, 1), y_hat(deque_size - 1, 2));
        invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
        buffer.pop_front(); //delete first element of deque
    }
    else
    {
        newImuObs = obs;
        invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
    }
}