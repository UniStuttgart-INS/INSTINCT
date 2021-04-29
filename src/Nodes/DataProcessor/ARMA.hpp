/// @file ARMA.hpp
/// @brief ARMA Node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-13

#pragma once

#include "Nodes/Node.hpp"

#include "util/Eigen.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include <deque>

namespace NAV
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

    std::deque<std::shared_ptr<ImuObs>> buffer;

    //loop iterator
    int k = 0;

    bool INITIALIZE = false;
    // arma order
    int p = 2;
    int q = 2;

    // buffer initialization
    int deque_size = 1000;
    int num_obs = 3;

    // INIT ARMA
    Eigen::MatrixXd y;
    Eigen::VectorXd y_rbm;
    Eigen::VectorXd y_hat;
    Eigen::VectorXd emp_sig;
    Eigen::VectorXd x;

    Eigen::VectorXi p_mem = Eigen::VectorXi::Zero(num_obs);
    Eigen::VectorXi q_mem = Eigen::VectorXi::Zero(num_obs);

    // CALC ARMA
    int m;
    double y_mean;
    Eigen::VectorXd y_hat_t;
};

} // namespace NAV
