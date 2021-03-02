/// @file Transformation.hpp
/// @brief Transformation Node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-02-01

#pragma once

#include "Nodes/Node.hpp"

#include "util/Eigen.hpp"

namespace NAV
{
class Transformation : public Node
{
  public:
    /// @brief Default constructor
    Transformation();
    /// @brief Destructor
    ~Transformation() override;
    /// @brief Copy constructor
    Transformation(const Transformation&) = delete;
    /// @brief Move constructor
    Transformation(Transformation&&) = delete;
    /// @brief Copy assignment operator
    Transformation& operator=(const Transformation&) = delete;
    /// @brief Move assignment operator
    Transformation& operator=(Transformation&&) = delete;

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

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(Pin* startPin, Pin* endPin) override;

    /// @brief Notifies the node, that some data was changed on one of it's output ports
    /// @param[in] linkId Id of the link on which data is changed
    void notifyOnOutputValueChanged(ax::NodeEditor::LinkId linkId) override;

  private:
    constexpr static size_t OutputPortIndex_Matrix = 0; ///< @brief Matrix
    constexpr static size_t InputPortIndex_Matrix = 0;  ///< @brief Matrix

    enum class Type : int
    {
        ECEF_2_LLArad,
        ECEF_2_LLAdeg,
        LLArad_2_ECEF,
        LLAdeg_2_ECEF,
        Quat_nb_2_RollPitchYawRad,
        Quat_nb_2_RollPitchYawDeg,
        RollPitchYawRad_2_Quat_nb,
        RollPitchYawDeg_2_Quat_nb,
        ECEF_2_NED,
        NED_2_ECEF,
        // CONJUGATE,
        // TRANSPOSE,
    };

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Function to call when the input value is changed
    /// @param[in] linkId Link Id over which the notification is sent
    void notifyOnInputValueChanged(ax::NodeEditor::LinkId linkId);

    /// @brief Checks if the connected Matrix has the specified size
    /// @param[in] transformationType The type of the Transformation
    /// @param[in] startPin Pin where the matrix is found
    static bool inputMatrixHasSize(Type transformationType, Pin* startPin);

    /// @brief Set the Matrix Size according to the specified Transformation
    /// @param[in] transformationType The type of the Transformation
    void setMatrixSize(Type transformationType);

    /// The matrix object
    Eigen::MatrixXd matrix;

    /// Algorithm to use
    Type selectedTransformation = Type::ECEF_2_LLArad;

    /// Reference point [𝜙 latitude, λ longitude, altitude]^T in [rad, rad, m]
    /// which represents the origin of the local frame for the ECEF <=> NED conversion
    Eigen::Vector3d latLonAlt_ref = Eigen::Vector3d::Zero();
};

} // namespace NAV
