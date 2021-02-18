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
    /*if (ImGui::Combo("combo 2 (one-liner)", &orderSelection, "aaaa\0bbbb\0cccc\0dddd\0eeee\0\0"))
    {
        flow::ApplyChanges();
    }*/

    ImGui::InputInt("Deque size", &deque_size);
    //ImGui::SameLine();
    ImGui::InputInt("Deque overlap", &refresh_overlap);
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

    return true;
}

void NAV::ARMA::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

/// @brief calculate autocorrelation function (ACF)
/// @param[in] x vector of data
Eigen::VectorXd acf_function(const Eigen::VectorXd& x)
{
    int tt = 0; // acf loop iterator
    int tau = 0;

    double cov = 0.0; // variable declaration
    double var = 0.0;

    Eigen::VectorXd acf(x.size());

    for (tt = 0; tt < x.size(); tt++)
    {
        var += (x(tt) - x.mean()) * (x(tt) - x.mean()); // sum of variance
    }

    for (tau = 0; tau < x.size(); tau++)
    { // acf: correlation to delta_tau

        for (tt = 0; tt < x.size() - tau; tt++)
        {
            cov += (x(tt) - x.mean()) * (x(tt + tau) - x.mean()); // 'cov' sum
        }
        acf(tau) = cov / var; // calculate sum/variance
        cov = 0;              // reset 'covar' for next sum
    }
    return acf;
}

/// @brief calculate partial autocorrelation function (PACF)
/// @param[in] x vector of data @param[in] acf vector of acf @param[in] p order of AR process
/// @param[out] pacf_output joined vector of pacf values and initial e_hat for Hannan-Rissanen
Eigen::VectorXd pacf_function(const Eigen::VectorXd& x, Eigen::VectorXd& acf, int p)
{
    int ii = 0;

    double sum1 = 0.0;
    double sum2 = 0.0;
    double sum3 = 0.0;

    Eigen::VectorXd pacf(x.size() - 1);
    Eigen::VectorXd e_hat_initial(x.size());
    Eigen::VectorXd phi_tau(x.size());
    Eigen::VectorXd phi_tau_i(x.size());
    Eigen::VectorXd phi_initial(p + 1);
    Eigen::VectorXd pacf_output(x.size() * 2 - 1); // size of vectors pacf + e_hat_initial

    // PACF while E(eta_t) = 0

    pacf(0) = acf(1); // initial value phi_1_1 = p(1)
    phi_tau_i(0) = pacf(0);
    sum1 = acf(1) * acf(1);
    sum2 = acf(1) * acf(1);

    e_hat_initial(0) = 0; // first iteration step

    for (int tau = 2; tau < x.size(); tau++)
    {
        if (tau == p + 2)
        { // Hannan Rissanen initial phi
            for (int m = 0; m < p + 1; m++)
            {
                phi_initial(m) = phi_tau_i(m);
            }
        }

        pacf(tau - 1) = (acf(tau) - sum1) / (1 - sum2);
        sum1 = 0;
        sum2 = 0;
        phi_tau = Eigen::VectorXd::Zero(x.size() - 1);

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

        if (tau > p + 1)
        {
            for (int m = 0; m < p + 1; m++)
            {
                sum3 += x(tau - m - 2) * phi_initial(m);
            }
            e_hat_initial(tau - 1) = x(tau - 1) - sum3; // calculate initial e_hat for hannan-rissanen
            sum3 = 0;
        }
        else
        {
            e_hat_initial(tau - 1) = 0;
        }
    }

    for (int m = 0; m < p + 1; m++)
    { // e_hat at Y_n
        sum3 += x(x.size() - m - 2) * phi_initial(m);
    }
    e_hat_initial(x.size() - 1) = x(x.size() - 1) - sum3;

    pacf_output << pacf, e_hat_initial;
    return pacf_output;
}

/// @brief fill A matrix for Hannan-Rissanen
/// @param[in] x vector of data @param[in] e_hat_initial for least squares @param[in] p order of AR process @param[in] q order of MA process
Eigen::MatrixXd matrix_function(const Eigen::VectorXd& x, const Eigen::VectorXd& e_hat_initial, int p, int q)
{
    Eigen::MatrixXd A(static_cast<int>(x.size()) - std::max(p, q), p + q); // conversion long to int?
    for (int t_HR = p; t_HR < x.size(); t_HR++)
    { // fill A
        for (int i_HR = 0; i_HR < p; i_HR++)
        {
            A(t_HR - p, i_HR) = x(t_HR - i_HR - 1); // AR
        }
        for (int i_HR = p; i_HR < p + q; i_HR++)
        {
            A(t_HR - p, i_HR) = -e_hat_initial(t_HR - i_HR + p - 1); // MA
        }
    }
    return A;
}

void NAV::ARMA::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    buffer.push_back(std::static_pointer_cast<ImuObs>(nodeData));

    if (static_cast<int>(buffer.size()) == deque_size)
    {
        Eigen::MatrixXd y_hat(buffer.size(), 3);
        Eigen::MatrixXd x(buffer.size(), 3);

        Eigen::VectorXd x_vec(p + q);
        Eigen::VectorXd e_hat(buffer.size());
        k = 0;
        for (auto& obs : buffer)
        {
            const Eigen::Vector3d& acc = obs->accelUncompXYZ.value();

            x(k, 0) = acc(0);
            x(k, 1) = acc(1);
            x(k, 2) = acc(2);
            k++;
        }

        for (int obs_num = 0; obs_num < 3; obs_num++)
        {
            // calculate acf
            Eigen::VectorXd acf = acf_function(x.col(obs_num));

            // calculate pacf & initial ê
            Eigen::VectorXd pacf_result = pacf_function(x.col(obs_num), acf, p);
            Eigen::VectorXd pacf = pacf_result.head(x.rows() - 1);
            Eigen::VectorXd e_hat_initial = pacf_result.tail(x.rows());

            // arma process
            double sum_HR = 0.0;

            for (int it = 0; it < 2; it++)
            {
                Eigen::MatrixXd A = matrix_function(x.col(obs_num), e_hat_initial, p, q);

                x_vec = (A.transpose() * A).ldlt().solve(A.transpose() * x.col(obs_num).tail(x.rows() - p)); // t > max(p, q)
                //
                e_hat = e_hat_initial;

                for (int i_HR = 0; i_HR < p; i_HR++)
                {                            // calculate e_hat for new variables
                    e_hat_initial(i_HR) = 0; // t <= max(p,q) = 0
                    y_hat(i_HR, obs_num) = x(i_HR, obs_num);
                }
                for (int t_HR = p; t_HR < x.rows(); t_HR++)
                { // AR
                    for (int i_HR = 0; i_HR < p; i_HR++)
                    {
                        sum_HR += x_vec(i_HR) * x(t_HR - i_HR - 1, obs_num);
                    }
                    for (int i_HR = p; i_HR < p + q; i_HR++)
                    { // MA
                        sum_HR -= x_vec(i_HR) * e_hat(t_HR - i_HR + p - 1);
                    }
                    e_hat_initial(t_HR) = x(t_HR, obs_num) - sum_HR;
                    y_hat(t_HR, obs_num) = sum_HR;
                    sum_HR = 0;
                }
            }
        }
        // deque iterator
        auto iter = buffer.begin();
        std::advance(iter, overlap);
        k = 0;
        for (; iter != buffer.end(); iter++)
        {
            auto obs = *iter;
            LOG_TRACE("{}: called {}", nameId(), obs->insTime->GetStringOfDate());

            auto newImuObs = std::make_shared<ImuObs>(obs->imuPos);

            newImuObs->insTime = obs->insTime;
            newImuObs->accelUncompXYZ = Eigen::Vector3d(y_hat(k, 0), y_hat(k, 1), y_hat(k, 2));
            k++;
            invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
        }

        if (overlap == 0)
        {
            overlap = refresh_overlap;
        }
        buffer.erase(buffer.begin(), buffer.end() - overlap); //erase part of already processed data
    }
}
