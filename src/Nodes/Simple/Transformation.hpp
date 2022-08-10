/// @file Transformation.hpp
/// @brief Transformation Node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-02-01

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Eigen.hpp"

namespace NAV
{
/// @brief Applies transformations to incoming NodeData
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
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
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

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_MATRIX = 0; ///< @brief Matrix
    constexpr static size_t INPUT_PORT_INDEX_MATRIX = 0;  ///< @brief Matrix

    /// @brief Possible Transformation types
    enum class Type : int
    {
        ECEF_2_LLArad,
        ECEF_2_LLAdeg,
        LLArad_2_ECEF,
        LLAdeg_2_ECEF,
        n_Quat_b_2_RollPitchYawRad,
        n_Quat_b_2_RollPitchYawDeg,
        RollPitchYawRad_2_n_Quat_b,
        RollPitchYawDeg_2_n_Quat_b,
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
    /// @param[in] startPin Pin where the matrix is found
    bool inputMatrixHasSize(Pin* startPin);

    /// @brief Updates the Matrix Size according to the selected Transformation
    void updateMatrixSize();

    /// The matrix object
    Eigen::MatrixXd _matrix;

    /// Algorithm to use
    Type _selectedTransformation = Type::ECEF_2_LLArad;
};

} // namespace NAV
