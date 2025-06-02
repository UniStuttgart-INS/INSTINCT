// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Demo.hpp
/// @brief Demo Node which demonstrates all capabilities
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-13

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Eigen.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include "util/CallbackTimer.hpp"

namespace NAV
{
/// @brief Demonstrates the basic GUI functionality of nodes
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
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. It is guaranteed that the node is initialized when this is called.
    bool resetNode() override;

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Called when a link was deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterDeleteLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Data struct transmitted over an output port
    struct DemoData
    {
        std::array<int, 3> integer = { 12, -2, 2 }; ///< Integer inside the DemoData
        bool boolean = false;                       ///< Boolean inside the DemoData
    };

    // private: // All this would be usually private. The Demo node however does not include private items to generate a complete Doxygen documentation

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// Update the pins depending on the GUI
    void updatePins();

    /// Pin types used in this demo
    enum class DemoPins : uint8_t
    {
        Delegate, ///< Delegate pins giving access to the complete connected node
        Flow,     ///< Flow pins transmitting data as timestamped shared pointers
        Bool,     ///< Booleans
        Int,      ///< Integer numbers
        Float,    ///< Float numbers
        Double,   ///< Double numbers
        String,   ///< Strings
        Object,   ///< Custom objects
        Matrix,   ///< Matrix objects
    };

    /// @brief Calculates the pin index for the given type
    /// @param[in] pinType Pin type to use
    /// @return Index of the pin (input and output pins have same indices)
    std::optional<size_t> getPinIdx(DemoPins pinType) const;

    /// @brief Receive callback on the Flow pin
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveData(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Polls data from the file. This function is needed, if we have multiple output pins, polling data.
    /// @note Not used in the node as only one output flow pin which does not need peeking and therefore utilizes the NAV::Demo::pollData() function
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> peekPollData(bool peek = false);

    /// @brief Polls data from the file
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData();

    /// @brief Updates the output flow pin depending on the GUI selection
    void updateOutputFlowPin();

    /// Whether to have a file reader instead of a sensor output pin
    bool _fileReaderInsteadSensor = false;

    /// Timer object to handle async data requests
    CallbackTimer _timer;

    /// @brief Function which performs the async data reading
    /// @param[in] userData Pointer to the object
    static void readSensorDataThread(void* userData);

    /// @brief Output frequency for the simulated sensor data
    int _outputFrequency = 1;
    /// @brief Counter how often data was received
    int _receivedDataCnt = 0;

    /// Counter for data Reading
    int _iPollData = 0;
    /// Amount of Observations to be read
    int _nPollData = 20;

    bool _enableDelegate = false; ///< Switch to enable the delegate pin
    bool _enableFlow = true;      ///< Switch to enable the flow pin
    bool _enableBool = false;     ///< Switch to enable the bool pin
    bool _enableInt = false;      ///< Switch to enable the int pin
    bool _enableFloat = false;    ///< Switch to enable the float pin
    bool _enableDouble = false;   ///< Switch to enable the double pin
    bool _enableString = false;   ///< Switch to enable the string pin
    bool _enableObject = false;   ///< Switch to enable the object pin
    bool _enableMatrix = false;   ///< Switch to enable the matrix pin

    bool _valueBool = true;                                         ///< Value which is represented over the Bool pin
    int _valueInt = -1;                                             ///< Value which is represented over the Int pin
    float _valueFloat = 65.4F;                                      ///< Value which is represented over the Float pin
    double _valueDouble = 1242.342;                                 ///< Value which is represented over the Double pin
    std::string _valueString = "Demo";                              ///< Value which is represented over the String pin
    DemoData _valueObject;                                          ///< Value which is represented over the Object pin
    Eigen::MatrixXd _valueMatrix = Eigen::MatrixXd::Identity(3, 3); ///< Value which is represented over the Matrix pin
    size_t _stringUpdateCounter = 0;                                ///< Counter of how often the string was updated

    /// @brief Function to call when the string is updated
    /// @param[in] insTime Time the data was received
    /// @param[in] pinIdx Index of the pin the data is received on
    void stringUpdatedNotifyFunction(const InsTime& insTime, size_t pinIdx);
};

} // namespace NAV
