/// @file ARMA.hpp
/// @brief ARMA Node
/// @author MSc. Janis Thürsam (janis.thuersam@yahoo.de)
/// @date 2021-01-13

#pragma once

#include "Nodes/Node.hpp"

#include "util/Eigen.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include <deque>

namespace NAV::experimental
{
class ARMA : public Node
{
  public:
    /// @brief Default constructor
    ARMA();
    /// @brief Destructor
    ~ARMA() override;
    /// @brief Copy constructor
    ARMA(const ARMA&) = delete;
    /// @brief Move constructor
    ARMA(ARMA&&) = delete;
    /// @brief Copy assignment operator
    ARMA& operator=(const ARMA&) = delete;
    /// @brief Move assignment operator
    ARMA& operator=(ARMA&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t InputPortIndex_ImuObs = 0;  ///< @brief Flow (ImuObs)
    constexpr static size_t OutputPortIndex_ImuObs = 1; ///< @brief Flow (ImuObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Sensor Data
    /// @param[in] nodeData Data to plot
    /// @param[in] linkId Id of the link over which the data is received
    void receiveImuObs(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief calculate autocorrelation function (ACF)
    /// @param[in] y vector of data
    /// @param[in] p order of AR process
    /// @param[out] acf vector of acf values
    static void acf_function(Eigen::VectorXd& y, int p, Eigen::VectorXd& acf);

    /// @brief calculate partial autocorrelation function (PACF) via Durbin-Levinson
    /// @param[in] y vector of data
    /// @param[in] acf vector of acf
    /// @param[in] p order of AR process
    /// @param[out] pacf vector of pacf values
    /// @param[out] initial_e_hat vector of initial ê for Hannan-Rissanen
    static void pacf_function(Eigen::VectorXd& y, Eigen::VectorXd& acf, int p, Eigen::VectorXd& pacf, Eigen::VectorXd& e_hat_initial);

    /// @brief Calculate ARMA parameters through Hannan-Rissanen
    /// @param[in] y vector of data
    /// @param[in] p order of AR process
    /// @param[in] q order of MA process
    void hannan_rissanen(Eigen::VectorXd& y, int p, int q, int m, int deque_size, Eigen::VectorXd& x, Eigen::VectorXd& emp_sig, Eigen::VectorXd& y_hat);

    /// @brief fill A matrix for least squares
    /// @param[in] y vector of data
    /// @param[in] e_hat residuals
    /// @param[in] p order of AR process
    /// @param[in] q order of MA process
    static void matrix_function(Eigen::VectorXd& y, Eigen::VectorXd& e_hat, int p, int q, int m, Eigen::MatrixXd& A);

    std::deque<std::shared_ptr<ImuObs>> buffer;

    /// loop iterator
    int k = 0;

    bool INITIALIZE = false; ///< parameter initialization indicator
    // ARMA order
    int p = 2; ///< AR order
    int q = 2; ///< MA order

    int deque_size = 1000; ///< modelling size
    int num_obs = 6;       ///< number of observations (3-axis accelerometer / 3-axis gyro)

    Eigen::MatrixXd y;       ///< measurement data
    Eigen::VectorXd y_rbm;   ///< y (reduced by mean)
    Eigen::VectorXd y_hat;   ///< ARMA estimates for y_rbm
    Eigen::VectorXd emp_sig; ///< empirical significance (p-Value) of parameters
    Eigen::VectorXd x;       ///< ARMA slope parameters
    Eigen::VectorXd y_hat_t; ///< output container

    int p_mem = 0; ///< p memory to reset for each observation
    int q_mem = 0; ///< q memory to reset for each observation

    int m = 0; ///< value of superior order (p or q)
    double y_mean = 0;
};

} // namespace NAV::experimental
