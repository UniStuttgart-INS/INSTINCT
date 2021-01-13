/// @file Demo.hpp
/// @brief Demo Node which demonstrates all capabilities
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-13

#pragma once

#include "Nodes/Node.hpp"

#include <Eigen/Core>

namespace NAV
{
class Demo : public Node
{
  public:
    /// @brief Default constructor
    Demo();
    /// @brief Destructor
    ~Demo() override;
    /// @brief Copy constructor
    Demo(const Demo&) = delete;
    /// @brief Move constructor
    Demo(Demo&&) = delete;
    /// @brief Copy assignment operator
    Demo& operator=(const Demo&) = delete;
    /// @brief Move assignment operator
    Demo& operator=(Demo&&) = delete;

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

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Resets the node. In case of file readers, that moves the read cursor to the start
    void resetNode() override;

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(Pin* startPin, Pin* endPin) override;

    /// @brief Called when a link is to be deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void onDeleteLink(Pin* startPin, Pin* endPin) override;

    struct DemoData
    {
        int integer = 12;
        bool boolean = false;
    };

  private:
    constexpr static size_t OutputPortIndex_NodeData = 1; ///< @brief Flow (NodeData)
    constexpr static size_t OutputPortIndex_InsObs = 2;   ///< @brief Flow (InsObs)
    constexpr static size_t InputPortIndex_StateData = 0; ///< @brief Object (StateData)

    /// @brief Receive Sensor Data
    /// @param[in] nodeData Data to plot
    /// @param[in] linkId Id of the link over which the data is received
    void receiveSensorData(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Receive File Reader Data
    /// @param[in] nodeData Data to plot
    /// @param[in] linkId Id of the link over which the data is received
    void receiveFileReaderData(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<NodeData> pollData(bool peek = false);
    /// Counter for data Reading
    size_t iPollData = 0;
    /// Amount of Observations to be read
    const size_t nPollData = 20;

    bool valueBool = true;
    int valueInt = -123;
    float valueFloat = 65.4F;
    double valueDouble = 1242352.342;
    std::string valueString = "Demo";
    DemoData valueObject;
    Eigen::MatrixXd valueMatrix;

    DemoData callbackFunction(int integer, bool boolean);
};

} // namespace NAV
