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

#include <unordered_set>
#include <map>

#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"

#include "Navigation/Math/PolynomialRegressor.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/State/LcKfInsGnssErrors.hpp"
#include "NodeData/State/TcKfInsGnssErrors.hpp"
#include "NodeData/GNSS/GnssCombination.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/SppSolution.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsSimulated.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

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

    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// Possible data identifiers to connect
    static inline std::vector<std::string> _dataIdentifier = { Pos::type(),
                                                               PosVel::type(),
                                                               PosVelAtt::type(),
                                                               LcKfInsGnssErrors::type(),
                                                               TcKfInsGnssErrors::type(),
                                                               GnssCombination::type(),
                                                               SppSolution::type(),
                                                               RtklibPosObs::type(),
                                                               ImuObs::type(),
                                                               ImuObsSimulated::type(),
                                                               KvhObs::type(),
                                                               ImuObsWDelta::type(),
                                                               VectorNavBinaryOutput::type() };

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ 0, this, pinAddCallback, pinDeleteCallback };

    /// Combination of data
    struct Combination
    {
        /// Term of a combination equation
        struct Term
        {
            double factor = 1.0;  ///< Factor to multiply the term with
            size_t pinIndex = 0;  ///< Pin Index
            size_t dataIndex = 0; ///< Data Index

            PolynomialRegressor<double> polyReg{ 1, 2 };           ///< Polynomial Regressor to interpolate data
            ScrollingBuffer<std::vector<std::string>> events{ 2 }; ///< Last events to add if we send

            /// @brief Get a string description of the combination
            /// @param node Combiner node pointer
            /// @param descriptors Data descriptors
            [[nodiscard]] std::string description(const Combiner* node, const std::vector<std::string>& descriptors) const
            {
                if (dataIndex < descriptors.size())
                {
                    return fmt::format("{} {} ({})", factor == 1.0 ? "+" : (factor == -1.0 ? "-" : fmt::format("{:.2f}", factor)),
                                       descriptors.at(dataIndex), node->inputPins.at(pinIndex).name);
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

    /// Send request information
    struct SendRequest
    {
        size_t combIndex = 0;                   ///< Combination Index
        std::unordered_set<size_t> termIndices; ///< Term indices, which are already calculated
        double result = 0.0;                    ///< Calculation result
        std::vector<std::string> events;        ///< List of events of all terms contributing to the result
    };

    /// Chronological list of send request
    std::map<InsTime, std::vector<SendRequest>> _sendRequests;

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Returns a list of descriptors for the pin
    /// @param pinIndex Pin Index to look for the descriptor
    [[nodiscard]] std::vector<std::string> getDataDescriptors(size_t pinIndex) const;

    /// @brief Checks if there are more pins with data for the same epoch
    /// @param insTime Time to check for
    [[nodiscard]] bool isLastObsThisEpoch(const InsTime& insTime) const;

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
