#include "ARMA.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include <Eigen/Dense>
#include <iterator>
#include <boost/math/distributions/students_t.hpp>

NAV::experimental::ARMA::ARMA()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() }, &ARMA::receiveImuObs);

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
}

NAV::experimental::ARMA::~ARMA()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::experimental::ARMA::typeStatic()
{
    return "ARMA";
}

std::string NAV::experimental::ARMA::type() const
{
    return typeStatic();
}

std::string NAV::experimental::ARMA::category()
{
    return "Experimental/DataProcessor";
}

void NAV::experimental::ARMA::guiConfig()
{
    // GUI ARMA node input
    ImGui::InputInt("Deque size", &_deque_size); // int input of modelling size
    ImGui::InputInt("p", &_p);                   // int input of initial AR-order
    ImGui::InputInt("q", &_q);                   // int input of initial MA-order
    static ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg;
    if (ImGui::BeginTable("##table1", 3, flags)) // display ARMA parameters (phi and theta) in table
    {
        ImGui::TableSetupColumn("ARMA Parameter");
        ImGui::TableSetupColumn("estimate");
        ImGui::TableSetupColumn("p");
        ImGui::TableHeadersRow();

        for (int table_row = 0; table_row < _x.size(); table_row++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (table_row < _p) // columns for AR-parameters (phi)
            {
                ImGui::Text("phi %d", table_row + 1);
            }
            else // columns for MA-parameters (theta)
            {
                ImGui::Text("theta %d", table_row - _p + 1);
            }
            ImGui::TableNextColumn();
            ImGui::Text("%f", _x(table_row)); // display parameter
            ImGui::TableNextColumn();
            ImGui::Text("%f", _emp_sig(table_row)); // display p-Value of zero slope test
        }
        ImGui::EndTable();
    }
}

[[nodiscard]] json NAV::experimental::ARMA::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["deque_size"] = _deque_size;
    j["p"] = _p;
    j["q"] = _q;

    return j;
}

void NAV::experimental::ARMA::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("deque_size"))
    {
        j.at("deque_size").get_to(_deque_size);
    }
    if (j.contains("p"))
    {
        j.at("p").get_to(_p);
    }
    if (j.contains("q"))
    {
        j.at("q").get_to(_q);
    }
}

bool NAV::experimental::ARMA::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _buffer.clear(); // clear deque

    // declaration
    _p_mem = _p; // reset p, q to initial for next observation
    _q_mem = _q;
    _y = Eigen::MatrixXd::Zero(_deque_size, _num_obs); // measurement data
    _y_rbm = Eigen::VectorXd::Zero(_deque_size);       // y (reduced by mean)
    _x = Eigen::VectorXd::Zero(_p + _q);               // ARMA slope parameters
    _emp_sig = Eigen::VectorXd::Zero(_p + _q);         // empirical significance (p-Value) of parameters
    _y_hat = Eigen::VectorXd::Zero(_deque_size);       // ARMA estimates for y_rbm

    _m = static_cast<int>(std::max(_p, _q)); // value of superior order (p or q)
    _y_mean = 0.0;
    _y_hat_t = Eigen::VectorXd::Zero(_num_obs); // output container

    return true;
}

void NAV::experimental::ARMA::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::experimental::ARMA::acf_function(Eigen::VectorXd& y, int p, Eigen::VectorXd& acf)
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

void NAV::experimental::ARMA::pacf_function(Eigen::VectorXd& y, Eigen::VectorXd& acf, int p, Eigen::VectorXd& pacf, Eigen::VectorXd& e_hat_initial)
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
    phi_tau_i(0) = pacf(0);
#pragma GCC diagnostic pop
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

void NAV::experimental::ARMA::matrix_function(Eigen::VectorXd& y, Eigen::VectorXd& e_hat, int p, int q, int m, Eigen::MatrixXd& A)
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

void NAV::experimental::ARMA::hannan_rissanen(Eigen::VectorXd& y, int p, int q, int m, int deque_size, Eigen::VectorXd& x, Eigen::VectorXd& emp_sig, Eigen::VectorXd& y_hat)
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
    int df = deque_size - p - q - 1;  // degrees of freedom
    boost::math::students_t dist(df); // T distribution

    // calculate acf
    acf_function(y, p, acf);

    // calculate pacf & initial ê
    pacf_function(y, acf, p, pacf, e_hat);
    // arma process
    for (int it = 0; it < 2; it++)
    {
        matrix_function(y, e_hat, p, q, m, A); // set A

        x = (A.transpose() * A).ldlt().solve(A.transpose() * y.tail(deque_size - m)); // t > max(p, q)

        for (int i_HR = 0; i_HR < m; i_HR++) // for t <= max(p,q)
        {
            y_hat(i_HR) = y(i_HR); // y_hat = y
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

void NAV::experimental::ARMA::receiveImuObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<const ImuObs>(nodeData);
    auto newImuObs = std::make_shared<ImuObs>(obs->imuPos);
    _buffer.push_back(obs); // push latest IMU epoch to deque

    if (static_cast<int>(_buffer.size()) == _deque_size) // deque filled
    {
        std::cout << "Initializing..." << std::endl;

        _k = 0;
        for (auto& obs : _buffer) // read observations from buffer to y
        {
            const Eigen::Vector3d acc = obs->accelCompXYZ.value(); // acceleration in x, y and z
            const Eigen::Vector3d gyro = obs->gyroCompXYZ.value(); // gyro in x, y and z
            _y.row(_k) << acc.transpose(), gyro.transpose();       // write to y
            _k++;
        }
        for (int obs_nr = 0; obs_nr < _num_obs; obs_nr++) // build ARMA-model for each observation
        {
            _p = _p_mem; // reset p, q to initial for next observation
            _q = _q_mem;

            _y_mean = _y.col(obs_nr).mean();
            _y_rbm = _y.col(obs_nr) - _y_mean * Eigen::VectorXd::Ones(_deque_size, 1); // reduce y by mean

            INITIALIZE = true; // set INITIALIZE true for each observation
            while (INITIALIZE) // while initializing ARMA-parameters
            {
                if (_p + _q == 0) // arma(0,0) -> y_hat = 0
                {
                    _y_hat(_deque_size - 1) = 0;
                    break;
                }
                _x.resize(_p + _q);       // resize
                _emp_sig.resize(_p + _q); // resize

                _m = static_cast<int>(std::max(_p, _q));

                hannan_rissanen(_y_rbm, _p, _q, _m, _deque_size, _x, _emp_sig, _y_hat); // parameter estimation

                // zero slope parameter test: search for emp_sig(p-Value) > alpha(0.05)
                if (_emp_sig.maxCoeff() > 0.05)
                {
                    int arma_it = 0;
                    for (arma_it = 0; arma_it < _p + _q; arma_it++)
                    {
                        if (_emp_sig(arma_it) == _emp_sig.maxCoeff()) // find index of maximum
                        {
                            break;
                        }
                    }
                    if (arma_it < _p) // if p-value has maximum for phi
                    {
                        _p--; // reduce AR order by 1
                    }
                    else // if p-value has maximum for theta
                    {
                        _q--; // reduce MA order by 1
                    }
                }
                else
                {
                    std::cout << "Initialized parameters for trajectory" << std::endl;
                    INITIALIZE = false; // initialized parameters for observation
                }
            }
            _y_hat_t(obs_nr) = _y_hat(_deque_size - 1) + _y_mean; // hand over last entry of y_hat and add y_mean
        }
        // output
        LOG_TRACE("{}: called {}", nameId(), obs->insTime->toYMDHMS());
        newImuObs->insTime = obs->insTime.value();
        newImuObs->accelCompXYZ = Eigen::Vector3d(_y_hat_t.head(3)); // output estimations of accelerometer observations
        newImuObs->gyroCompXYZ = Eigen::Vector3d(_y_hat_t.tail(3));  // output estimations of gyro observations
        invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, newImuObs);
        _buffer.pop_front();
    }
    else // output = input while filling deque
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_IMU_OBS, obs);
    }
}