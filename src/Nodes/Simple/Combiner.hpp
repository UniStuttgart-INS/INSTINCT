/// @file Combiner.hpp
/// @brief Combiner Node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-02-01

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Eigen.hpp"

namespace NAV
{
/// @brief Combines two input ports into a single output port
class Combiner : public Node
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

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Called when a link was deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterDeleteLink(OutputPin& startPin, InputPin& endPin) override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_FLOW = 0;       ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_FLOW_FIRST = 0;  ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_FLOW_SECOND = 1; ///< @brief Flow

    /// @brief Set the Pin Identifiers for the other pin depending on the connected pin
    /// @param[in] connectedPinIndex The connected pin
    /// @param[in] otherPinIndex The unconnected pin which needs the identifiers to be set
    /// @param[in] dataIdentifiers The data Identifier to be considered
    void setPinIdentifiers(size_t connectedPinIndex, size_t otherPinIndex, const std::vector<std::string>& dataIdentifiers);

    /// @brief Checks if link on the pin is still valid and refreshes if so
    /// @param[in] oldDataIdentifiers Data identifiers which were previously set
    void updateOutputPin(const std::vector<std::string>& oldDataIdentifiers);

    /// @brief Receive data
    /// @param[in] nodeData Observation received
    /// @param[in] pinId Id of the pin the data is received on
    void receiveData(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::PinId pinId);
};

} // namespace NAV
