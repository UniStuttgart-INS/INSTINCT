// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Combiner.hpp
/// @brief Calculates differences between signals
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-29

#pragma once

#include <limits>
#include <memory>
#include <unordered_set>
#include <map>

#include "Navigation/Time/InsTime.hpp"
#include "NodeData/NodeData.hpp"
#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"

#include "Navigation/Math/PolynomialRegressor.hpp"

#include "NodeData/GNSS/GnssCombination.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/SppSolution.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsSimulated.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/VectorNavBinaryOutput.hpp"
#include "NodeData/State/InsGnssLCKFSolution.hpp"
#include "NodeData/State/InsGnssTCKFSolution.hpp"
#include "NodeData/State/PosVelAtt.hpp"

#include "util/Logger/CommonLog.hpp"
#include "util/Container/ScrollingBuffer.hpp"
#include "util/Container/Unordered_map.hpp"

namespace NAV
{
/// @brief Calculates differences between signals
class Combiner : public Node, public CommonLog
{
  public:
    /// @brief Default constructor
    Combiner();
    /// @brief Destructor
    ~Combiner() override;
    /// @brief Copy constructor
    Combiner(const Combiner&) = delete;
    /// @brief Move constructor
    Combiner(Combiner&&) = delete;
    /// @brief Copy assignment operator
    Combiner& operator=(const Combiner&) = delete;
    /// @brief Move assignment operator
    Combiner& operator=(Combiner&&) = delete;

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

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_DYN_DATA = 0; ///< @brief Flow (DynamicData)

    /// Possible data identifiers to connect
    static inline std::vector<std::string> _dataIdentifier = { Pos::type(),
                                                               PosVel::type(),
                                                               PosVelAtt::type(),
                                                               InsGnssLCKFSolution::type(),
                                                               InsGnssTCKFSolution::type(),
                                                               GnssCombination::type(),
                                                               GnssObs::type(),
                                                               SppSolution::type(),
                                                               RtklibPosObs::type(),
                                                               ImuObs::type(),
                                                               ImuObsWDelta::type(),
                                                               ImuObsSimulated::type(),
                                                               KvhObs::type(),
                                                               VectorNavBinaryOutput::type() };

    /// Combination of data
    struct Combination
    {
        /// Term of a combination equation
        struct Term
        {
            double factor = 1.0;                                         ///< Factor to multiply the term with
            size_t pinIndex = 0;                                         ///< Pin Index
            std::variant<size_t, std::string> dataSelection = size_t(0); ///< Data Index or Data identifier

            PolynomialRegressor<double> polyReg{ 1, 2 };                   ///< Polynomial Regressor to interpolate data
            ScrollingBuffer<std::shared_ptr<const NodeData>> rawData{ 2 }; ///< Last raw data to add if we send

            /// @brief Get a string description of the combination
            /// @param node Combiner node pointer
            /// @param descriptors Data descriptors
            [[nodiscard]] std::string description(const Combiner* node, const std::vector<std::string>& descriptors) const
            {
                if (std::holds_alternative<size_t>(dataSelection) && std::get<size_t>(dataSelection) < descriptors.size())
                {
                    return fmt::format("{} {} ({})", factor == 1.0 ? "+" : (factor == -1.0 ? "-" : fmt::format("{:.2f}", factor)),
                                       descriptors.at(std::get<size_t>(dataSelection)), node->inputPins.at(pinIndex).name);
                }
                if (std::holds_alternative<std::string>(dataSelection))
                {
                    return fmt::format("{} {} ({})", factor == 1.0 ? "+" : (factor == -1.0 ? "-" : fmt::format("{:.2f}", factor)),
                                       std::get<std::string>(dataSelection), node->inputPins.at(pinIndex).name);
                }
                return fmt::format("N/A ({})", node->inputPins.at(pinIndex).name);
            }
        };

        /// List of terms making up the combination
        std::vector<Term> terms{ Term{ .factor = +1.0, .pinIndex = 0 },
                                 Term{ .factor = -1.0, .pinIndex = 1 } };

        /// @brief Get a string description of the combination
        /// @param node Combiner node pointer
        [[nodiscard]] std::string description(const Combiner* node) const
        {
            std::string desc;
            for (const auto& term : terms)
            {
                auto descriptors = node->getDataDescriptors(term.pinIndex);
                auto termDescription = term.description(node, descriptors);

                if (!desc.empty())
                {
                    if (termDescription.starts_with("+ ") || termDescription.starts_with("- "))
                    {
                        desc += " ";
                    }
                    else
                    {
                        desc += " + ";
                    }
                }
                desc += termDescription;
            }

            return desc;
        }
    };

    /// Combinations to calculate
    std::vector<Combination> _combinations{ Combination() };

    /// Pin data
    struct PinData
    {
        /// Time of the last observation processed
        InsTime lastTime;
        /// Min time between messages
        double minTimeStep = std::numeric_limits<double>::infinity();
        /// Extra data descriptors for dynamic data
        std::vector<std::string> dynDataDescriptors;
    };

    /// Data per pin
    std::vector<PinData> _pinData;

    /// Reference pin
    size_t _refPinIdx = 0;

    /// Output missing combinations with NaN instead of removing
    bool _outputMissingAsNaN = false;

    /// Send request information
    struct SendRequest
    {
        size_t combIndex = 0;                                                         ///< Combination Index
        std::unordered_set<size_t> termIndices;                                       ///< Term indices, which are already calculated
        double result = 0.0;                                                          ///< Calculation result
        std::vector<std::pair<std::string, std::shared_ptr<const NodeData>>> rawData; ///< List of the raw data of all terms contributing to the result
    };

    /// Chronological list of send request
    std::map<InsTime, std::vector<SendRequest>> _sendRequests;

    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ 0, this, pinAddCallback, pinDeleteCallback };

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Returns a list of descriptors for the pin
    /// @param pinIndex Pin Index to look for the descriptor
    [[nodiscard]] std::vector<std::string> getDataDescriptors(size_t pinIndex) const;

    /// @brief Receive Data Function
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveData(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Write info to a json object
    /// @param[out] j Json output
    /// @param[in] data Object to read info from
    friend void to_json(json& j, const Combination& data);
    /// @brief Read info from a json object
    /// @param[in] j Json variable to read info from
    /// @param[out] data Output object
    friend void from_json(const json& j, Combination& data);
    /// @brief Write info to a json object
    /// @param[out] j Json output
    /// @param[in] data Object to read info from
    friend void to_json(json& j, const Combination::Term& data);
    /// @brief Read info from a json object
    /// @param[in] j Json variable to read info from
    /// @param[out] data Output object
    friend void from_json(const json& j, Combination::Term& data);
};

} // namespace NAV
