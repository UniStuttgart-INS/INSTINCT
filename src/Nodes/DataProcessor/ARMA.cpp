#include "ARMA.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include <Eigen/Dense>
#include <iterator>
#include <boost/math/distributions/students_t.hpp>

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
    ImGui::InputInt("Deque size", &deque_size); // int input for initialization size
    ImGui::InputInt("p", &p);
    ImGui::InputInt("q", &q);
    /*ImGui::Checkbox("process ACF", &ACF_CHECK); // checkboxes for ACF, PACF
        ImGui::SameLine();
        ImGui::Checkbox("process PACF", &PACF_CHECK);*/
    static ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg;
    if (ImGui::BeginTable("##table1", 3, flags))
    {
        ImGui::TableSetupColumn("ARMA Parameter");
        ImGui::TableSetupColumn("estimate");
        ImGui::TableSetupColumn("p");
        ImGui::TableHeadersRow();

        for (int table_row = 0; table_row < x.size(); table_row++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (table_row < p)
            {
                ImGui::Text("phi %d", table_row + 1);
            }
            else
            {
                ImGui::Text("theta %d", table_row - p + 1);
            }
            ImGui::TableNextColumn();
            ImGui::Text("%f", x(table_row));
            ImGui::TableNextColumn();
            ImGui::Text("%f", emp_sig(table_row));
        }
        ImGui::EndTable();
    }
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

    buffer.clear();

    // INIT_ARMA
    p_mem = Eigen::VectorXi::Constant(num_obs, p); // init p, q
    q_mem = Eigen::VectorXi::Constant(num_obs, q);
    y = Eigen::VectorXd::Zero(deque_size);  // trajectory container
    x = Eigen::VectorXd::Zero(p + q);       // ARMA slope parameters
    emp_sig = Eigen::VectorXd::Zero(p + q); // empirical significance (p-Value)
    y_hat = Eigen::VectorXd::Zero(deque_size);

    // CALC_ARMA
    m = static_cast<int>(std::max(p, q));
    y_mean = 0.0;
    y_hat_t = Eigen::VectorXd::Zero(num_obs);

    return true;
}

void NAV::ARMA::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

/// @brief calculate autocorrelation function (ACF)
/// @param[in] y vector of data
void acf_function(Eigen::VectorXd& y, int p, Eigen::VectorXd& acf)
{
    bool ACF_CHECK = false;

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
    if (ACF_CHECK) // skip 'cause of runtime
    {
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
void pacf_function(Eigen::VectorXd& y, Eigen::VectorXd& acf, int p, Eigen::VectorXd& pacf, Eigen::VectorXd& e_hat_initial)
{
    bool PACF_CHECK = false;
    int pacf_size = static_cast<int>(y.size());
    int ar_approx = p + 2; // AR process of order > max(p,q) to approximate arma

    Eigen::VectorXd phi_tau = Eigen::VectorXd::Zero(pacf_size);
    Eigen::VectorXd phi_tau_i = Eigen::VectorXd::Zero(pacf_size);
    Eigen::VectorXd phi_initial = Eigen::VectorXd::Zero(ar_approx);

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

    for (int tau = 2; tau <= pacf_size; tau++)
    {
        // Hannan-Rissanen initial phi
        if (tau == ar_approx + 1)
        {
            phi_initial = phi_tau_i.head(tau - 1);
        }

        // skip this if phi_initial is determined and PACF CHECK is false:
        if (tau < ar_approx + 1 || (PACF_CHECK && tau < pacf_size))
        {
            pacf(tau - 1) = (acf(tau) - sum1) / (1 - sum2);
            sum1 = 0;
            sum2 = 0;

            for (int i = 0; i < tau - 1; i++)
            {
                phi_tau(i) = (phi_tau_i(i) - pacf(tau - 1) * phi_tau_i(tau - i - 2));
                sum1 += phi_tau(i) * acf(tau - i); // top sum
                sum2 += phi_tau(i) * acf(i + 1);   // bottom sum
            }
            phi_tau_i = phi_tau; // last element of phi_tau_i -> phi_tau_tau
            phi_tau_i(tau - 1) = pacf(tau - 1);
            sum1 += pacf(tau - 1) * acf(1);
            sum2 += pacf(tau - 1) * acf(tau);
        }

        // Hannan-Rissanen initial e_hat
        if (tau > ar_approx)
        {
            for (int iter = 0; iter < ar_approx; iter++)
            {
                sum3 += y(tau - iter - 2) * phi_initial(iter);
            }
            e_hat_initial(tau - 1) = y(tau - 1) - sum3;
            sum3 = 0;
        }
        else
        {
            e_hat_initial(tau - 1) = 0;
        }
    }
}
/// @brief fill A matrix for Hannan-Rissanen
/// @param[in] y vector of data @param[in] e_hat_initial residuals @param[in] p order of AR process @param[in] q order of MA process
void matrix_function(Eigen::VectorXd& y, Eigen::VectorXd& e_hat, int p, int q, int m, Eigen::MatrixXd& A)
{
    for (int t_HR = m; t_HR < y.size(); t_HR++)
    {
        //A(t_HR, 0) = 1; //as phi0 being constant
        for (int i_HR = 0; i_HR < p; i_HR++)
        {
            A(t_HR - m, i_HR) = y(t_HR - i_HR - 1); // AR
        }
        for (int i_HR = p; i_HR < p + q; i_HR++)
        {
            A(t_HR - m, i_HR) = -e_hat(t_HR - i_HR + p - 1); // MA
        }
    }
}

/// @brief Initialize ARMA parameters
/// @param[in] y vector of data @param[in] @param[in] p order of AR process @param[in] q order of MA process
void estimation(Eigen::VectorXd& y, int p, int q, int m, int deque_size, Eigen::VectorXd& x, Eigen::VectorXd& emp_sig, Eigen::VectorXd& y_hat)
{
    // declaration
    // acf
    Eigen::VectorXd acf = Eigen::VectorXd::Zero(deque_size);
    // pacf
    Eigen::VectorXd pacf = Eigen::VectorXd::Zero(deque_size - 1);
    Eigen::VectorXd e_hat = Eigen::VectorXd::Zero(deque_size);
    // arma
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(deque_size - m, p + q);
    // arma parameter test
    Eigen::VectorXd t = Eigen::VectorXd::Zero(p + q);
    Eigen::MatrixXd Cov_inv = Eigen::MatrixXd::Zero(p + q, p + q); // inverse Covariance matrix of least squares estimator

    int df = deque_size - p - q - 1;  //degrees of freedom   // aus schleife rausziehen?
    boost::math::students_t dist(df); // T distribution

    // calculate acf
    acf_function(y, p, acf);

    // calculate pacf & initial ê
    pacf_function(y, acf, p, pacf, e_hat);
    // arma process
    //for (int it = 0; it < 2; it++)
    while (e_hat.transpose() * e_hat > 0.1 * df)
    {
        matrix_function(y, e_hat, p, q, m, A); //set A

        x = (A.transpose() * A).ldlt().solve(A.transpose() * y.tail(deque_size - m)); // t > max(p, q)

        for (int i_HR = 0; i_HR < m; i_HR++) //for t <= max(p,q)
        {
            y_hat(i_HR) = y(i_HR); //y_hat = y
        }
        y_hat.tail(deque_size - m) = A * x;
        e_hat = y - y_hat;
        std::cout << e_hat.transpose() * e_hat << std::endl;
    }

    // arma parameter test
    double e_square = e_hat.transpose() * e_hat;
    double var_e = e_square / df;
    Cov_inv = (A.transpose() * A).inverse();
    for (int j = 0; j < p + q; j++)
    {
        t(j) = x(j) / sqrt(var_e * Cov_inv(j, j));
        emp_sig(j) = 2 * boost::math::pdf(dist, t(j));
    }
}

void NAV::ARMA::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);
    auto newImuObs = std::make_shared<ImuObs>(obs->imuPos); // nicht in jeder schleife?
    buffer.push_back(obs);
    if (static_cast<int>(buffer.size()) == deque_size)
    {
        std::cout << "Initializing..." << std::endl;

        for (int obs_index = 0; obs_index < num_obs; obs_index++)
        {
            p = p_mem(obs_index); // reset p, q
            q = q_mem(obs_index);

            k = 0;
            for (auto& obs : buffer)
            {
                const Eigen::Vector3d acc = obs->accelUncompXYZ.value();
                y(k) = acc(obs_index);
                k++;
            }
            y_mean = y.mean();
            y = y - y_mean * Eigen::VectorXd::Ones(deque_size, 1); // reduce y by mean
            //std::cout << y.mean() << std::endl;
            INITIALIZE = true;
            while (INITIALIZE)
            {
                x.resize(p + q);       // resize
                emp_sig.resize(p + q); // resize

                m = static_cast<int>(std::max(p, q));
                estimation(y, p, q, m, deque_size, x, emp_sig, y_hat);
                // zero slope parameter test: search for emp_sig(p-Value) > alpha(0.05)
                if (emp_sig.maxCoeff() > 0.05)
                {
                    int arma_it = 0;
                    for (arma_it = 0; arma_it < p + q; arma_it++)
                    {
                        if (emp_sig(arma_it) == emp_sig.maxCoeff()) // find index of maximum
                        {
                            break;
                        }
                    }
                    if (arma_it < p)
                    {
                        p--; // reduce MA order by 1
                    }
                    else
                    {
                        q--; // reduce AR order by 1
                    }
                }
                else
                {
                    std::cout << "Initialized parameters for trajectory" << std::endl;
                    INITIALIZE = false;
                    //sleep(1);
                }
            }
            y_hat_t(obs_index) = y_hat(deque_size - 1) + y_mean;
        }
        // output
        LOG_TRACE("{}: called {}", nameId(), obs->insTime->GetStringOfDate());
        newImuObs->insTime = obs->insTime.value();
        newImuObs->accelUncompXYZ = Eigen::Vector3d(y_hat_t);
        newImuObs->gyroUncompXYZ = obs->gyroUncompXYZ;
        invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
        buffer.pop_front();
    }
    else // output = input while filling deque
    {
        newImuObs = obs;
        invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
    }
}