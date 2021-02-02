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

    /// @brief Notifies the node ifself, that some data was changed
    /// @param[in] linkId Id of the link on which data is changed
    void onNotifyValueChanged(ax::NodeEditor::LinkId linkId) override;

  private:
    constexpr static size_t OutputPortIndex_Matrix = 1; ///< @brief Matrix
    constexpr static size_t InputPortIndex_Matrix = 0;  ///< @brief Matrix

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Function to call when the value is changed
    /// @param[in] linkId Link Id over which the notification is sent
    void notifyFunction(ax::NodeEditor::LinkId linkId);

    /// The matrix object
    Eigen::MatrixXd matrix;
};

} // namespace NAV
