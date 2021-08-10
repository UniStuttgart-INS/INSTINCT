/// @file LooselyCoupledKF.hpp
/// @brief Kalman Filter class for the loosely coupled INS/GNSS integration
/// @author T. Topp (topp@ins.uni-stuttgart.de) and M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-08-04

#pragma once

#include "Nodes/Node.hpp"
#include "NodeData/State/PosVelAtt.hpp"

namespace NAV
{
class LooselyCoupledKF : public Node
{
  public:
    /// @brief Default constructor
    LooselyCoupledKF();
    /// @brief Destructor
    ~LooselyCoupledKF() override;
    /// @brief Copy constructor
    LooselyCoupledKF(const LooselyCoupledKF&) = delete;
    /// @brief Move constructor
    LooselyCoupledKF(LooselyCoupledKF&&) = delete;
    /// @brief Copy assignment operator
    LooselyCoupledKF& operator=(const LooselyCoupledKF&) = delete;
    /// @brief Move assignment operator
    LooselyCoupledKF& operator=(LooselyCoupledKF&&) = delete;
    /// @brief String representation of the class type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the class type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the class category
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
    constexpr static size_t OutputPortIndex_PosVelAtt__t0 = 0; ///< @brief Flow (PosVelAtt)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the measured state PosVelAtt at the time t_k
    /// @param[in] nodeData State vector (PosVelAtt)
    /// @param[in] linkId Id of the link over which the data is received
    void recvState__t0(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Filters the observation data from INS and GNSS
    void filterObservation();

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                          Propagation                                                     */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Updates the state transition matrix 𝚽 limited to first order in 𝐅𝜏ₛ
    /// @param[in] quaternion_nb Attitude of the body with respect to n-system
    /// @param[in] specForce_ib_b Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coord.
    /// @param[in] velocity_n Velocity in n-system in [m / s]
    /// @param[in] position_lla Position as Lat Lon Alt in [rad rad m]
    /// @param[in] tau_s time interval in [s]
    /// @note See Groves (2013) chapter 14.2.4, equations (14.63) and (14.72)
    static Eigen::MatrixXd transitionMatrix(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& specForce_ib_b, const Eigen::Vector3d& velocity_n, const Eigen::Vector3d& position_lla, double tau_s);

    /// @brief Submatrix 𝐅_11 of the system matrix 𝐅
    /// @param[in] angularRate_in_n Angular rate vector of the n-system with respect to the i-system in [rad / s], resolved in the n-system
    /// @return The 3x3 matrix 𝐅_11
    /// @note See Groves (2013) equation (14.64)
    static Eigen::Matrix3d systemMatrixF_11_n(const Eigen::Vector3d& angularRate_in_n);

    /// @brief Submatrix 𝐅_12 of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_12
    /// @note See Groves (2013) equation (14.65)
    static Eigen::Matrix3d systemMatrixF_12_n(double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_13 of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @return The 3x3 matrix 𝐅_13
    /// @note See Groves (2013) equation (14.66)
    static Eigen::Matrix3d systemMatrixF_13_n(double latitude_b, double height_b, const Eigen::Vector3d& v_eb_n);

    /// @brief Submatrix 𝐅_21 of the system matrix 𝐅
    /// @param[in] quaternion_nb Attitude of the body with respect to n-system
    /// @param[in] specForce_ib_b Specific force of the body with respect to inertial frame in [m / s^2], resolved in body coord.
    /// @return The 3x3 matrix 𝐅_21
    /// @note See Groves (2013) equation (14.67)
    static Eigen::Matrix3d systemMatrixF_21_n(const Eigen::Quaterniond& quaternion_nb, const Eigen::Vector3d& specForce_ib_b);

    /// @brief Submatrix 𝐅_22 of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_22
    /// @note See Groves (2013) equation (14.68)
    static Eigen::Matrix3d systemMatrixF_22_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_23 of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_23
    /// @note See Groves (2013) equation (14.69)
    static Eigen::Matrix3d systemMatrixF_23_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_32 of the system matrix 𝐅
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_32
    /// @note See Groves (2013) equation (14.70)
    static Eigen::Matrix3d systemMatrixF_32_n(double latitude_b, double height_b);

    /// @brief Submatrix 𝐅_33 of the system matrix 𝐅
    /// @param[in] v_eb_n Velocity of the body with respect to the e-system in [m / s], resolved in the n-system
    /// @param[in] latitude_b Geodetic latitude of the body in [rad]
    /// @param[in] height_b Geodetic height of the body in [m]
    /// @return The 3x3 matrix 𝐅_33
    /// @note See Groves (2013) equation (14.71)
    static Eigen::Matrix3d systemMatrixF_33_n(const Eigen::Vector3d& v_eb_n, double latitude_b, double height_b);

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                          Correction                                                      */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Observation at time t_k
    std::shared_ptr<PosVelAtt> posVelAtt__t0 = nullptr;
};
} // namespace NAV