// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file VectorNavBinaryConverter.hpp
/// @brief Converts VectorNavBinaryOutput
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-07-09

#pragma once

#include "internal/Node/Node.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/GNSS/GnssObs.hpp"

#include <array>
#include <memory>

namespace NAV
{
/// Converts VectorNavBinaryOutput
class VectorNavBinaryConverter : public Node
{
  public:
    /// @brief Default constructor
    VectorNavBinaryConverter();
    /// @brief Destructor
    ~VectorNavBinaryConverter() override;
    /// @brief Copy constructor
    VectorNavBinaryConverter(const VectorNavBinaryConverter&) = delete;
    /// @brief Move constructor
    VectorNavBinaryConverter(VectorNavBinaryConverter&&) = delete;
    /// @brief Copy assignment operator
    VectorNavBinaryConverter& operator=(const VectorNavBinaryConverter&) = delete;
    /// @brief Move assignment operator
    VectorNavBinaryConverter& operator=(VectorNavBinaryConverter&&) = delete;

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

    /// Enum specifying the type of the output message
    enum class OutputType
    {
        ImuObs,       ///< Extract ImuObs data
        ImuObsWDelta, ///< Extract ImuObsWDelta data
        PosVelAtt,    ///< Extract PosVelAtt data
        GnssObs,      ///< Extract GnssObs data
        COUNT,        ///< Number of items in the enum
    };

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_CONVERTED = 0;              ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_VECTORNAV_BINARY_OUTPUT = 0; ///< @brief Flow (VectorNavBinaryOutput)

    /// The selected output type in the GUI
    OutputType _outputType = OutputType::ImuObsWDelta;

    /// Enum specifying the source for the PosVelAtt conversion
    enum PosVelSource
    {
        PosVelSource_Best,  ///< INS > GNSS1 > GNSS2
        PosVelSource_Ins,   ///< Take only INS values into account
        PosVelSource_Gnss1, ///< Take only GNSS1 values into account
        PosVelSource_Gnss2, ///< Take only GNSS2 values into account
    };

    /// The selected PosVel source in the GUI
    PosVelSource _posVelSource = PosVelSource_Best;

    /// GUI option. If checked forces position to a static value and velocity to 0
    bool _forceStatic = false;

    /// Whether to extract the compensated data or the uncompensated
    bool _useCompensatedData = false;

    /// Position, Velocity and Attitude at initialization (needed for static data)
    std::shared_ptr<const PosVelAtt> _posVelAtt__init = nullptr;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Converts the VectorNavBinaryOutput observation to the selected message type
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Converts the VectorNavBinaryOutput to a ImuObsWDelta observation
    /// @param[in] vnObs VectorNavBinaryOutput to process
    /// @return The converted data
    std::shared_ptr<const ImuObsWDelta> convert2ImuObsWDelta(const std::shared_ptr<const VectorNavBinaryOutput>& vnObs);

    /// @brief Converts the VectorNavBinaryOutput to a ImuObs observation
    /// @param[in] vnObs VectorNavBinaryOutput to process
    /// @return The converted data
    std::shared_ptr<const ImuObs> convert2ImuObs(const std::shared_ptr<const VectorNavBinaryOutput>& vnObs);

    /// @brief Converts the VectorNavBinaryOutput to a PosVelAtt observation
    /// @param[in] vnObs VectorNavBinaryOutput to process
    /// @return The converted data
    std::shared_ptr<const PosVelAtt> convert2PosVelAtt(const std::shared_ptr<const VectorNavBinaryOutput>& vnObs);

    /// @brief Converts the VectorNavBinaryOutput to a GnssObs observation
    /// @param[in] vnObs VectorNavBinaryOutput to process
    /// @return The converted data
    static std::shared_ptr<const GnssObs> convert2GnssObs(const std::shared_ptr<const VectorNavBinaryOutput>& vnObs);
};

/// @brief Converts the enum to a string
/// @param[in] value Enum value to convert into text
/// @return String representation of the enum
const char* to_string(NAV::VectorNavBinaryConverter::OutputType value);

} // namespace NAV