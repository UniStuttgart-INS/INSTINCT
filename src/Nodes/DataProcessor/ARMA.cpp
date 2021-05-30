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
    // GUI ARMA node input
    ImGui::InputInt("Deque size", &deque_size); // int input of modelling size
    ImGui::InputInt("p", &p);                   // int input of initial AR-order
    ImGui::InputInt("q", &q);                   // int input of initial MA-order
    static ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg;
    if (ImGui::BeginTable("##table1", 3, flags)) // display ARMA parameters (phi and theta) in table
    {
        ImGui::TableSetupColumn("ARMA Parameter");
        ImGui::TableSetupColumn("estimate");
        ImGui::TableSetupColumn("p");
        ImGui::TableHeadersRow();

        for (int table_row = 0; table_row < x.size(); table_row++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (table_row < p) // columns for AR-parameters (phi)
            {
                ImGui::Text("phi %d", table_row + 1);
            }
            else // columns for MA-parameters (theta)
            {
                ImGui::Text("theta %d", table_row - p + 1);
            }
            ImGui::TableNextColumn();
            ImGui::Text("%f", x(table_row)); //display parameter
            ImGui::TableNextColumn();
            ImGui::Text("%f", emp_sig(table_row)); // display p-Value of zero slope test
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

    buffer.clear(); // clear deque

    // declaration
    p_mem = p; // reset p, q to initial for next observation
    q_mem = q;
    y = Eigen::MatrixXd::Zero(deque_size, num_obs); // measurement data
    y_rbm = Eigen::VectorXd::Zero(deque_size);      // y (reduced by mean)
    x = Eigen::VectorXd::Zero(p + q);               // ARMA slope parameters
    emp_sig = Eigen::VectorXd::Zero(p + q);         // empirical significance (p-Value) of parameters
    y_hat = Eigen::VectorXd::Zero(deque_size);      // ARMA estimates for y_rbm

    m = static_cast<int>(std::max(p, q)); // value of superior order (p or q)
    y_mean = 0.0;
    y_hat_t = Eigen::VectorXd::Zero(num_obs); // output container

    return true;
}

void NAV::ARMA::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

/// @brief calculate autocorrelation function (ACF)
/// @param[in] y vector of data @param[in] p order of AR process
/// @param[out] acf vector of acf values
void acf_function(Eigen::VectorXd& y, int p, Eigen::VectorXd& acf)
{
    int acf_size = static_cast<int>(y.size()); // size of y
    /* Calculation of initial ê through Yule-Walker:
    model approximation through AR(m)-process with m > max(p,q)
    therefore limited amount of ACF values required to calculate PACF */
    int p_approx = p + 3; // in this case AR(p + 3)

    double cov = 0.0; // covariance of measured values
    double var = 0.0; // variance of measured values
    double mean = y.mean();

    for (int i = 0; i < acf_size; i++)
    {
        var += (y(i) - mean) * (y(i) - mean); // sum of variance
    }
    // Calculation limited to required ACF values for Yule-Walker
    for (int tau = 0; tau < p_approx + 1; tau++)
    { // acf: correlation to delta_tau

        for (int i = 0; i < acf_size - tau; i++)
        {
            cov += (y(i) - mean) * (y(i + tau) - mean); // sum of covariance
        }
        acf(tau) = cov / var;
        cov = 0; // reset 'cov' for next sum
    }
}

/// @brief calculate partial autocorrelation function (PACF) via Durbin-Levinson
/// @param[in] y vector of data @param[in] acf vector of acf @param[in] p order of AR process
/// @param[out] pacf vector of pacf values @param[out] initial_e_hat vector of initial ê for Hannan-Rissanen
void pacf_function(Eigen::VectorXd& y, Eigen::VectorXd& acf, int p, Eigen::VectorXd& pacf, Eigen::VectorXd& e_hat_initial)
{
    int pacf_size = static_cast<int>(y.size());
    /* Calculation of initial ê through Yule-Walker:
    AR(m) parameters (phi_1...m) are equal to phi_m_1...m from PACF */
    int p_approx = p + 3; // AR order > max(p,q) to approximate ARMA

    Eigen::VectorXd phi_tau = Eigen::VectorXd::Zero(p_approx);     // phi_tau_1...tau
    Eigen::VectorXd phi_tau_i = Eigen::VectorXd::Zero(p_approx);   // phi_tau-1_1...tau-1 (from previous iteration)
    Eigen::VectorXd phi_initial = Eigen::VectorXd::Zero(p_approx); // Yule-Walker ARMA-Parameters

    double sum1 = 0.0;
    double sum2 = 0.0;
    double sum3 = 0.0;

    // PACF while E(eta_t) = 0
    // first iteration step
    pacf(0) = acf(1); // initial value phi_1_1 = p(1)
    phi_tau_i(0) = pacf(0);
    sum1 = acf(1) * acf(1);
    sum2 = acf(1) * acf(1);

    e_hat_initial(0) = 0; // first entry

    for (int tau = 2; tau <= pacf_size; tau++)
    {
        // Hannan-Rissanen initial phi
        if (tau == p_approx + 1) // hand over AR parameters at matching moment
        {
            phi_initial = phi_tau_i;
        }
        // PACF calculation through Durbin-Levinson
        // skip this if phi_initial is determined:
        if (tau < p_approx + 1)
        {
            pacf(tau - 1) = (acf(tau) - sum1) / (1 - sum2);
            sum1 = 0;
            sum2 = 0;

            for (int i = 0; i < tau - 1; i++)
            {
                phi_tau(i) = (phi_tau_i(i) - pacf(tau - 1) * phi_tau_i(tau - i - 2));
                sum1 += phi_tau(i) * acf(tau - i); // numerator sum
                sum2 += phi_tau(i) * acf(i + 1);   // denominator sum
            }
            phi_tau_i = phi_tau;
            phi_tau_i(tau - 1) = pacf(tau - 1); // last element of phi_tau_i -> phi_tau_tau
            sum1 += pacf(tau - 1) * acf(1);
            sum2 += pacf(tau - 1) * acf(tau);
        }

        /* Hannan-Rissanen initial ê
        calculate estimates of y (y_hat) for AR(p_approx)-process to determine resulting ê */
        if (tau > p_approx)
        {
            for (int iter = 0; iter < p_approx; iter++)
            {
                sum3 += y(tau - iter - 2) * phi_initial(iter); // y_hat of current tau
            }
            e_hat_initial(tau - 1) = y(tau - 1) - sum3; // ê = y - y_hat
            sum3 = 0;
        }
        else
        {
            e_hat_initial(tau - 1) = 0; // ê_t for t < m = 0 (condition in Yule-Walker estimation)
        }
    }
}
/// @brief fill A matrix for least squares
/// @param[in] y vector of data @param[in] e_hat residuals @param[in] p order of AR process @param[in] q order of MA process
void matrix_function(Eigen::VectorXd& y, Eigen::VectorXd& e_hat, int p, int q, int m, Eigen::MatrixXd& A)
{
    for (int t = m; t < y.size(); t++) // rows
    {
        for (int i = 0; i < p; i++) // AR columns
        {
            A(t - m, i) = y(t - i - 1); // dependency of y_t-1 ... y_t-p
        }
        for (int i = p; i < p + q; i++) // MA columns
        {
            A(t - m, i) = -e_hat(t - i + p - 1); // dependency of ê_t-1 ... ê_t-q
        }
    }
}

/// @brief Calculate ARMA parameters through Hannan-Rissanen
/// @param[in] y vector of data @param[in] @param[in] p order of AR process @param[in] q order of MA process
void hannan_rissanen(Eigen::VectorXd& y, int p, int q, int m, int deque_size, Eigen::VectorXd& x, Eigen::VectorXd& emp_sig, Eigen::VectorXd& y_hat)
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
    Eigen::MatrixXd ata_inv = Eigen::MatrixXd::Zero(p + q, p + q); // inverse Covariance matrix of least squares estimator

    // necessary for parameter test:
    int df = deque_size - p - q - 1;  //degrees of freedom
    boost::math::students_t dist(df); // T distribution

    // calculate acf
    acf_function(y, p, acf);

    // calculate pacf & initial ê
    pacf_function(y, acf, p, pacf, e_hat);
    // arma process
    for (int it = 0; it < 2; it++)
    {
        matrix_function(y, e_hat, p, q, m, A); //set A

        x = (A.transpose() * A).ldlt().solve(A.transpose() * y.tail(deque_size - m)); // t > max(p, q)

        for (int i_HR = 0; i_HR < m; i_HR++) //for t <= max(p,q)
        {
            y_hat(i_HR) = y(i_HR); //y_hat = y
        }
        y_hat.tail(deque_size - m) = A * x; // calculate y_hat and ê for t > max(p,q)
        e_hat = y - y_hat;
    }

    // arma parameter test (parameter significance test)
    double e_square = e_hat.transpose() * e_hat;
    double var_e = e_square / df; // sigma^2_e
    ata_inv = (A.transpose() * A).inverse();
    for (int j = 0; j < p + q; j++)
    {
        t(j) = x(j) / sqrt(var_e * ata_inv(j, j));     // t quantile
        emp_sig(j) = 2 * boost::math::pdf(dist, t(j)); // probability for two-sided t-Test
    }
}

void NAV::ARMA::receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);
    auto newImuObs = std::make_shared<ImuObs>(obs->imuPos);
    buffer.push_back(obs); // push latest IMU epoch to deque

    if (static_cast<int>(buffer.size()) == deque_size) // deque filled
    {
        std::cout << "Initializing..." << std::endl;

        k = 0;
        for (auto& obs : buffer) // read observations from buffer to y
        {
            const Eigen::Vector3d acc = obs->accelCompXYZ.value(); // acceleration in x, y and z
            const Eigen::Vector3d gyro = obs->gyroCompXYZ.value(); // gyro in x, y and z
            y.row(k) << acc.transpose(), gyro.transpose();         // write to y
            k++;
        }
        for (int obs_nr = 0; obs_nr < num_obs; obs_nr++) // build ARMA-model for each observation
        {
            p = p_mem; // reset p, q to initial for next observation
            q = q_mem;

            y_mean = y.col(obs_nr).mean();
            y_rbm = y.col(obs_nr) - y_mean * Eigen::VectorXd::Ones(deque_size, 1); // reduce y by mean

            INITIALIZE = true; // set INITIALIZE true for each observation
            while (INITIALIZE) // while initializing ARMA-parameters
            {
                if (p + q == 0) //arma(0,0) -> y_hat = 0
                {
                    y_hat(deque_size - 1) = 0;
                    break;
                }
                x.resize(p + q);       // resize
                emp_sig.resize(p + q); // resize

                m = static_cast<int>(std::max(p, q));

                hannan_rissanen(y_rbm, p, q, m, deque_size, x, emp_sig, y_hat); // parameter estimation

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
                    if (arma_it < p) // if p-value has maximum for phi
                    {
                        p--; // reduce AR order by 1
                    }
                    else // if p-value has maximum for theta
                    {
                        q--; // reduce MA order by 1
                    }
                }
                else
                {
                    std::cout << "Initialized parameters for trajectory" << std::endl;
                    INITIALIZE = false; // initialized parameters for observation
                }
            }
            y_hat_t(obs_nr) = y_hat(deque_size - 1) + y_mean; // hand over last entry of y_hat and add y_mean
        }
        // output
        LOG_TRACE("{}: called {}", nameId(), obs->insTime->GetStringOfDate());
        newImuObs->insTime = obs->insTime.value();
        newImuObs->accelCompXYZ = Eigen::Vector3d(y_hat_t.head(3)); // output estimations of accelerometer observations
        newImuObs->gyroCompXYZ = Eigen::Vector3d(y_hat_t.tail(3));  // output estimations of gyro observations
        invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
        buffer.pop_front();
    }
    else // output = input while filling deque
    {
        newImuObs = obs;
        invokeCallbacks(OutputPortIndex_ImuObs, newImuObs);
    }
}