/**
 * @file RtklibPosGnuPlot.hpp
 * @brief Plots RTKLIB Pos Data
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-06-02
 */

#pragma once

#include "../GnuPlot.hpp"

#include "NodeData/GNSS/RtklibPosObs.hpp"

namespace NAV
{
/// Plots RTKLIB Pos Data
class RtklibPosGnuPlot final : public GnuPlot
{
  public:
    /**
     * @brief Construct a new RTKLIB Pos Gnu Plot object
     * 
     * @param[in] name Name of the Node
     * @param[in] options Program options string map
     */
    RtklibPosGnuPlot(const std::string& name, const std::map<std::string, std::string>& options);

    RtklibPosGnuPlot() = default;                                  ///< Default Constructor
    ~RtklibPosGnuPlot() final;                                     ///< Destructor
    RtklibPosGnuPlot(const RtklibPosGnuPlot&) = delete;            ///< Copy constructor
    RtklibPosGnuPlot(RtklibPosGnuPlot&&) = delete;                 ///< Move constructor
    RtklibPosGnuPlot& operator=(const RtklibPosGnuPlot&) = delete; ///< Copy assignment operator
    RtklibPosGnuPlot& operator=(RtklibPosGnuPlot&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("RtklibPosGnuPlot");
    }

    /**
     * @brief Returns the String representation of the Class Category
     * 
     * @retval constexpr std::string_view The class category
     */
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("Plot");
    }

    /**
     * @brief Returns Gui Configuration options for the class
     * 
     * @retval std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> The gui configuration
     */
    [[nodiscard]] std::vector<std::tuple<ConfigOptions, std::string, std::string, std::vector<std::string>>> guiConfig() const final
    {
        return { { Node::CONFIG_LIST_LIST_INT, "Data to plot", "Specify what data should be plotted.\n\nData with the same Window Id gets plotted into the same GnuPlot window.\nWindow Id '-1' disables the plot.", { "gpsToW|latitude|[longitude]|height|x-ecef|y-ecef|z-ecef", "gpsToW|[latitude]|longitude|height|x-ecef|y-ecef|z-ecef|Q|ns|sdn|sde|sdu|sdx|sdy|sdz|sdne|sdeu|sdun|sdxy|sdyz|sdzx|age|ratio", "-1|-1|100" } } };
    }

    /**
     * @brief Returns the context of the class
     * 
     * @retval constexpr std::string_view The class context
     */
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::ALL;
    }

    /**
     * @brief Returns the number of Ports
     * 
     * @param[in] portType Specifies the port type
     * @retval constexpr uint8_t The number of ports
     */
    [[nodiscard]] constexpr uint8_t nPorts(PortType portType) const final
    {
        switch (portType)
        {
        case PortType::In:
            return 1U;
        case PortType::Out:
            break;
        }

        return 0U;
    }

    /**
     * @brief Returns the data types provided by this class
     * 
     * @param[in] portType Specifies the port type
     * @param[in] portIndex Port index on which the data is sent
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                return RtklibPosObs().type();
            }
            break;
        case PortType::Out:
            break;
        }

        return std::string_view("");
    }

    /**
     * @brief Handles the data sent on the input port
     * 
     * @param[in] portIndex The input port index
     * @param[in, out] data The data send on the input port
     */
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) final
    {
        if (portIndex == 0)
        {
            auto obs = std::static_pointer_cast<RtklibPosObs>(data);
            plotRtklibPosObs(obs);
        }
    }
    /**
     * @brief Requests the node to send out its data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputData(uint8_t /* portIndex */) final { return nullptr; }

    /**
     * @brief Requests the node to peek its output data
     * 
     * @param[in] portIndex The output port index
     * @retval std::shared_ptr<NodeData> The requested data or nullptr if no data available
     */
    [[nodiscard]] std::shared_ptr<NodeData> requestOutputDataPeek(uint8_t /* portIndex */) final { return nullptr; }

  private:
    /**
     * @brief Plots an VectorNav Observation
     * 
     * @param[in] obs The received observation
     */
    void plotRtklibPosObs(std::shared_ptr<RtklibPosObs>& obs);
};

} // namespace NAV
