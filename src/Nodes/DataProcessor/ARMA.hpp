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

    bool INITIALIZE = false; // parameter initialization indicator
    // ARMA order
    int p = 2; // AR order
    int q = 2; // MA order

    int deque_size = 1000; // modelling size
    int num_obs = 6;       // number of observations (3-axis accelerometer / 3-axis gyro)

    Eigen::MatrixXd y;       // measurement data
    Eigen::VectorXd y_rbm;   // y (reduced by mean)
    Eigen::VectorXd y_hat;   // ARMA estimates for y_rbm
    Eigen::VectorXd emp_sig; // empirical significance (p-Value) of parameters
    Eigen::VectorXd x;       // ARMA slope parameters
    Eigen::VectorXd y_hat_t; // output container

    int p_mem; // p, q memory to reset for each observation
    int q_mem;

    int m; // value of superior order (p or q)
    double y_mean;
};

} // namespace NAV
