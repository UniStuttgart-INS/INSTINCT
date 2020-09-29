/// @file GnuPlot.hpp
/// @brief Abstract class for Gnuplotting
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-04-14

#pragma once

#include "../Node.hpp"

#include "gnuplot-iostream.h"
#include <deque>
#include <vector>
#include <map>

#include "NodeData/State/StateData.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/VectorNavObs.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"

namespace NAV
{
/// Abstract class for Gnuplotting
class GnuPlot final : public Node
{
  public:
    /// @brief Constructor
    /// @param[in] name Name of the Node
    /// @param[in] options Program options string map
    GnuPlot(const std::string& name, const std::map<std::string, std::string>& options);

    /// @brief Default constructor
    GnuPlot() = default;
    /// @brief Destructor
    ~GnuPlot() final;
    /// @brief Copy constructor
    GnuPlot(const GnuPlot&) = delete;
    /// @brief Move constructor
    GnuPlot(GnuPlot&&) = delete;
    /// @brief Copy assignment operator
    GnuPlot& operator=(const GnuPlot&) = delete;
    /// @brief Move assignment operator
    GnuPlot& operator=(GnuPlot&&) = delete;

    /// @brief Returns the String representation of the Class Type
    /// @return The class type
    [[nodiscard]] constexpr std::string_view type() const final
    {
        return std::string_view("GnuPlot");
    }

    /// @brief Returns the String representation of the Class Category
    /// @return The class category
    [[nodiscard]] constexpr std::string_view category() const final
    {
        return std::string_view("Plot");
    }

    /// @brief Returns Gui Configuration options for the class
    /// @return The gui configuration
    [[nodiscard]] std::vector<ConfigOptions> guiConfig() const final
    {
        // clang-format off
        return {
            { CONFIG_INT, "Clear after x data",
                            "If more than x data are collected, the oldest one is discarded.\n\nIgnored in post processing mode",
                            { "0", "1000", "10000" } },
            { CONFIG_INT, "Update Frequency",
                            "Frequency to update the Plot Windows\n\nIgnored in post processing mode",
                            { "1", "50", "2000" } },
            { CONFIG_STRING_BOX, "Start", "Gnuplot Commands to execute when starting the application.", { "set autoscale xy\nset grid ytics lc rgb \"#bbbbbb\" lw 1 lt 0\nset grid xtics lc rgb \"#bbbbbb\" lw 1 lt 0\n" } },
            { CONFIG_N_INPUT_PORTS, "Input Ports",
                                    "Amount of Input Ports",
                                    { "1", "1", "30", "3" } },
            { CONFIG_LIST, "Port Type",
                           "Select the type of the message to receive on this port",
                           { "[" + std::string(ImuObs().type()) + "]",
                             std::string(VectorNavObs().type()),
                             std::string(RtklibPosObs().type()),
                             std::string(KvhObs().type()),
                             std::string(StateData().type()) } },
            { CONFIG_VARIANT, "", "",
                  // ImuObs
                { ConfigOptionsBase(CONFIG_LIST_MULTI, "Data to plot",
                                                            "Specify what data should be plotted.",
                                                            { "[Time]", "GPS time of week",
                                                              "Time since startup",
                                                              "Mag uncomp X", "Mag uncomp Y", "Mag uncomp Z",
                                                              "Accel uncomp X", "Accel uncomp Y", "Accel uncomp Z",
                                                              "Gyro uncomp X", "Gyro uncomp Y", "Gyro uncomp Z",
                                                              "Temperature" }),
                  // VectorNavObs
                  ConfigOptionsBase(CONFIG_LIST_MULTI, "Data to plot",
                                                            "Specify what data should be plotted.",
                                                            { "[Time]", "GPS time of week",
                                                              "Time since startup",
                                                              "Quaternion W", "Quaternion X", "Quaternion Y", "Quaternion Z",
                                                              "Yaw", "Pitch", "Roll",
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
                  // RtklibPosObs
                  ConfigOptionsBase(CONFIG_LIST_MULTI, "Data to plot",
                                                            "Specify what data should be plotted.",
                                                            { "[Time]", "GPS time of week",
                                                              "Latitude", "Longitude", "Height",
                                                              "X-ECEF", "Y-ECEF", "Z-ECEF",
                                                              "North [m]", "East [m]",
                                                              "Q",
                                                              "ns",
                                                              "sdn", "sde", "sdu",
                                                              "sdx", "sdy", "sdz",
                                                              "sdne", "sdeu", "sdun",
                                                              "sdxy", "sdyz", "sdzx",
                                                              "Age",
                                                              "Ratio" }),
                  // KvhObs
                  ConfigOptionsBase(CONFIG_LIST_MULTI, "Data to plot",
                                                            "Specify what data should be plotted.",
                                                            { "[Time]", "GPS time of week",
                                                              "Time since startup",
                                                              "Temperature",
                                                              "Sequence Number",
                                                              "Mag uncomp X", "Mag uncomp Y", "Mag uncomp Z",
                                                              "Accel uncomp X", "Accel uncomp Y", "Accel uncomp Z",
                                                              "Gyro uncomp X", "Gyro uncomp Y", "Gyro uncomp Z" }),
                  // StateData
                  ConfigOptionsBase(CONFIG_LIST_MULTI, "Data to plot",
                                                            "Specify what data should be plotted.",
                                                            { "[Time]", "GPS time of week",
                                                              "Latitude", "Longitude", "Height",
                                                              "X-ECEF", "Y-ECEF", "Z-ECEF",
                                                              "North [m]", "East [m]",
                                                              "Velocity North", "Velocity East", "Velocity Down",
                                                              "Roll", "Pitch", "Yaw",
                                                              "Quaternion W", "Quaternion X", "Quaternion Y", "Quaternion Z" })
                } },
            { CONFIG_STRING_BOX, "Update", "Gnuplot Commands called every time to update the view.\nUse [~x], where x is a number, to plot data selected below", { "plot [~1,2,3~] using 1:2 with lines title 'MyTitle'" } },

        };
        // clang-format on
    }

    /// @brief Returns the context of the class
    /// @return The class context
    [[nodiscard]] constexpr NodeContext context() const final
    {
        return NodeContext::ALL;
    }

    /// @brief Returns the number of Ports
    /// @param[in] portType Specifies the port type
    /// @return The number of ports
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

    /// @brief Returns the data types provided by this class
    /// @param[in] portType Specifies the port type
    /// @param[in] portIndex Port index on which the data is sent
    /// @return The data type
    [[nodiscard]] constexpr std::string_view dataType(PortType portType, uint8_t portIndex) const final
    {
        switch (portType)
        {
        case PortType::In:
            if (inputPortDataTypes.count(portIndex))
            {
                return inputPortDataTypes.at(portIndex);
            }
            break;
        case PortType::Out:
            break;
        }

        return std::string_view("");
    }

    /// @brief Handles the data sent on the input port
    /// @param[in] portIndex The input port index
    /// @param[in, out] data The data send on the input port
    void handleInputData(uint8_t portIndex, std::shared_ptr<NodeData> data) final
    {
        if (inputPortDataTypes.count(portIndex))
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
            else if (inputPortDataTypes.at(portIndex) == KvhObs().type())
            {
                auto obs = std::static_pointer_cast<KvhObs>(data);
                handleKvhObs(obs, portIndex);
            }
            else if (inputPortDataTypes.at(portIndex) == ImuObs().type())
            {
                auto obs = std::static_pointer_cast<ImuObs>(data);
                handleImuObs(obs, portIndex);
            }
            else if (inputPortDataTypes.at(portIndex) == StateData().type())
            {
                auto state = std::static_pointer_cast<StateData>(data);
                handleStateData(state, portIndex);
            }
        }
    }

    /// @brief Updates the gnuplot windows
    /// @return
    [[nodiscard]] bool update();

    /// @brief Checks if an update is necessary and then performs it
    void requestUpdate();

  private:
    /// Handle a VectorNav Observation
    void handleVectorNavObs(std::shared_ptr<NAV::VectorNavObs>& obs, size_t portIndex);
    /// Handle a RTKLIB Pos File Observation
    void handleRtklibPosObs(std::shared_ptr<NAV::RtklibPosObs>& obs, size_t portIndex);
    /// Handle a KVH IMU Observation
    void handleKvhObs(std::shared_ptr<NAV::KvhObs>& obs, size_t portIndex);
    /// Handle a IMU Observation
    void handleImuObs(std::shared_ptr<NAV::ImuObs>& obs, size_t portIndex);
    /// Handle State data
    void handleStateData(std::shared_ptr<NAV::StateData>& state, size_t portIndex);

    /// Number of input ports
    uint8_t nInputPorts = 1;
    /// Input Data Types
    std::map<size_t, std::string> inputPortDataTypes;

    /// Data handling class
    class GnuPlotData
    {
      public:
        /// @brief Constructor
        /// @param[in] dataIdentifier Identifier for the Data
        explicit GnuPlotData(std::string dataIdentifier)
            : dataIdentifier(std::move(dataIdentifier)) {}

        double startValue = std::nan("");
        /// Data identifier
        std::string dataIdentifier;
        /// x and y data which can be passed to the plot stream
        std::deque<double> data;
    };

    /// Data to plot
    std::map<size_t, std::vector<GnuPlotData>> plotData;

    /// gnuplot object
    gnuplotio::Gnuplot gp{ "gnuplot -persist > /dev/null 2>&1" };

    /// The amount of x data to plot
    uint32_t xDisplayScope = 10.0;
    /// Update Frequency
    uint32_t updateFrequency = 10;

    std::vector<std::string> portUpdateStrings;
};

} // namespace NAV
