/**
 * @file GnuPlot.hpp
 * @brief Abstract class for Gnuplotting
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-14
 */

#pragma once

#include "../Node.hpp"

#include "gnuplot-iostream.h"
#include <deque>
#include <vector>
#include <map>

#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"

namespace NAV
{
/// Abstract class for Gnuplotting
class GnuPlot final : public Node
{
  public:
    /**
     * @brief Construct a new Gnu Plot object
     * 
     * @param[in] name Name of the Node
     * @param[in] options Program options string map
     */
    GnuPlot(const std::string& name, const std::map<std::string, std::string>& options);

    GnuPlot() = default;                         ///< Default Constructor
    ~GnuPlot() final;                            ///< Destructor
    GnuPlot(const GnuPlot&) = delete;            ///< Copy constructor
    GnuPlot(GnuPlot&&) = delete;                 ///< Move constructor
    GnuPlot& operator=(const GnuPlot&) = delete; ///< Copy assignment operator
    GnuPlot& operator=(GnuPlot&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the String representation of the Class Type
     * 
     * @retval constexpr std::string_view The class type
     */
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("GnuPlot");
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
     * @retval std::vector<ConfigOptions> The gui configuration
     */
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        // clang-format off
        return {
            { CONFIG_FLOAT, "X Display Scope",
                            "Data older/smaller than the specified scope gets discarded.\ne.g. Shows only the last x seconds.\n\nIgnored in post processing mode",
                            { "0", "10", "100" } },
            { CONFIG_FLOAT, "Update Frequency",
                            "Frequency to update the Plot Windows\n\nIgnored in post processing mode",
                            { "0", "50", "200" } },
            { CONFIG_STRING_BOX, "Start", "Gnuplot Commands to execute when starting the application.", { "set autoscale xy\nset grid ytics lc rgb \"#bbbbbb\" lw 1 lt 0\nset grid xtics lc rgb \"#bbbbbb\" lw 1 lt 0\n" } },
            { CONFIG_STRING_BOX, "Update", "Gnuplot Commands called every time to update the view.", { "" } },
            { CONFIG_N_INPUT_PORTS, "Input Ports",
                                    "Amount of Input Ports",
                                    { "1", "1", "30", "2" } },
            { CONFIG_LIST, "Port Type",
                           "Select the type of the message to receive on this port",
                           { std::string(VectorNavObs().type()),
                             "[" + std::string(RtklibPosObs().type()) + "]" } },
            { CONFIG_VARIANT, "", "",
                { ConfigOptionsBase(CONFIG_LIST_LIST_MULTI, "Data to plot",
                                                            "Specify what data should be plotted.",
                                                            { "[GPS time of week]",
                                                              "|",
                                                              "Time since startup",
                                                              "Quaternion W", "Quaternion X", "Quaternion Y", "Quaternion Z",
                                                              "[Yaw]", "Pitch", "Roll",
                                                              "Mag uncomp X", "Mag uncomp Y", "Mag uncomp Z",
                                                              "Accel uncomp X", "Accel uncomp Y", "Accel uncomp Z",
                                                              "Gyro uncomp X", "Gyro uncomp Y", "Gyro uncomp Z",
                                                              "Mag comp X", "Mag comp Y", "Mag comp Z",
                                                              "Accel comp X", "Accel comp Y", "Accel comp Z",
                                                              "Gyro comp X", "Gyro comp Y", "Gyro comp Z",
                                                              "Sync In Count",
                                                              "dtime",
                                                              "dtheta X", "dtheta Y", "dtheta Z",
                                                              "dvel X", "dvel Y", "dvel Z",
                                                              "Vpe Status",
                                                              "Temperature",
                                                              "Pressure",
                                                              "Mag comp N", "Mag comp E", "Mag comp D",
                                                              "Accel comp N", "Accel comp E", "Accel comp D",
                                                              "Gyro comp N", "Gyro comp E", "Gyro comp D",
                                                              "Linear Accel X", "Linear Accel Y", "Linear Accel Z",
                                                              "Linear Accel N", "Linear Accel E", "Linear Accel D",
                                                              "Yaw Uncertainty", "Pitch Uncertainty", "Roll Uncertainty" }),
                  ConfigOptionsBase(CONFIG_LIST_LIST_MULTI, "Data to plot",
                                                            "Specify what data should be plotted.",
                                                            { "GPS time of week",
                                                              "Latitude", "[Longitude]", "Height",
                                                              "X-ECEF", "Y-ECEF", "Z-ECEF",
                                                              "|",
                                                              "GPS time of week",
                                                              "[Latitude]", "Longitude", "Height",
                                                              "X-ECEF", "Y-ECEF", "Z-ECEF",
                                                              "Q",
                                                              "ns",
                                                              "sdn", "sde", "sdu",
                                                              "sdx", "sdy", "sdz",
                                                              "sdne", "sdeu", "sdun",
                                                              "sdxy", "sdyz", "sdzx",
                                                              "Age",
                                                              "Ratio" })
                } },
        };
        // clang-format on
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
            return nInputPorts;
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
            if (inputPortDataTypes.contains(portIndex))
            {
                return inputPortDataTypes.at(portIndex);
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
        if (inputPortDataTypes.contains(portIndex))
        {
            if (inputPortDataTypes.at(portIndex) == VectorNavObs().type())
            {
                auto obs = std::static_pointer_cast<VectorNavObs>(data);
                handleVectorNavObs(obs, portIndex);
            }
            else if (inputPortDataTypes.at(portIndex) == RtklibPosObs().type())
            {
                auto obs = std::static_pointer_cast<RtklibPosObs>(data);
                handleRtklibPosObs(obs, portIndex);
            }
        }
    }

    [[nodiscard]] bool update() const;

    void requestUpdate() const;

  private:
    /// Handle a VectorNav Observation
    void handleVectorNavObs(std::shared_ptr<NAV::VectorNavObs>& obs, size_t portIndex);
    /// Handle a RTKLIB Pos File Observation
    void handleRtklibPosObs(std::shared_ptr<NAV::RtklibPosObs>& obs, size_t portIndex);

    /// Number of input ports
    uint8_t nInputPorts = 1;
    /// Input Data Types
    std::map<size_t, std::string> inputPortDataTypes;

    class GnuPlotData
    {
      public:
        /**
         * @brief Construct a new Gnu Plot Data object
         * 
         * @param[in] xData Identifier for x Data
         * @param[in] yData Identifier for y Data
         */
        GnuPlotData(std::string xData, std::string yData)
            : xData(std::move(xData)), yData(std::move(yData)) {}
        /// xData specifier
        std::string xData;
        /// yData specifier
        std::string yData;
        /// x and y data which can be passed to the plot stream
        std::deque<std::pair<double, double>> xy;
    };

    /// Data to plot
    std::map<size_t, std::vector<GnuPlotData>> plotData;

    /// gnuplot object pointer
    gnuplotio::Gnuplot* gp = nullptr;

    double xDisplayScope = 10.0;
    double updateFrequency = 10.0;
};

} // namespace NAV
