/// @file Plot.hpp
/// @brief Plots data into ImPlot Windows
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include "Nodes/Node.hpp"

#include <map>

#include "util/ScrollingBuffer.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/GNSS/SkydelObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

namespace NAV
{
class Plot : public Node
{
  public:
    /// @brief Default constructor
    Plot();
    /// @brief Destructor
    ~Plot() override;
    /// @brief Copy constructor
    Plot(const Plot&) = delete;
    /// @brief Move constructor
    Plot(Plot&&) = delete;
    /// @brief Copy assignment operator
    Plot& operator=(const Plot&) = delete;
    /// @brief Move assignment operator
    Plot& operator=(Plot&&) = delete;

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

    /// @brief Called when a new link was established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterCreateLink(Pin* startPin, Pin* endPin) override;

    /// @brief Called when a link is to be deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void onDeleteLink(Pin* startPin, Pin* endPin) override;

    struct PinData
    {
        struct PlotData
        {
            /// @brief Default constructor
            PlotData() = default;

            /// @brief Constructor
            /// @param[in] displayName Display name of the contained data
            /// @param[in] size Size of the buffer
            explicit PlotData(std::string displayName, size_t size)
                : displayName(std::move(displayName)), buffer(size) {}

            /// Display name of the contained data
            std::string displayName;
            /// Buffer for the data
            ScrollingBuffer<double> buffer;
            /// Flag if data was received, as the buffer contains std::nan("") otherwise
            bool hasData = false;
            /// Key: PlotIndex; Value: yAxis
            std::map<size_t, int> plotOnAxis;
        };

        enum class PlotStyle : int
        {
            Scatter,
            Line,
        };

        enum class PinType : int
        {
            Flow,   ///< NodeData Trigger
            Bool,   ///< Boolean
            Int,    ///< Integer Number
            Float,  ///< Floating Point Number
            Matrix, ///< Matrix Object
        };

        /// @brief Adds a plotData Element to the list
        /// @param[in] displayName Display name of the contained data
        void addPlotDataItem(const std::string& displayName)
        {
            if (std::find_if(plotData.begin(),
                             plotData.end(),
                             [displayName](const PlotData& plotData) { return plotData.displayName == displayName; })
                == plotData.end())
            {
                plotData.emplace_back(displayName, static_cast<size_t>(size));
            }
        }
        /// Size of all buffers of the plotData elements
        int size = 2000;
        /// Data Identifier of the connected pin
        std::string dataIdentifier;
        /// List with all the data
        std::vector<PlotData> plotData;
        /// Plot style for all data on the pin
        PlotStyle plotStyle = PlotStyle::Scatter;
        /// Pin Type
        PinType pinType = PinType::Flow;
    };

    struct PlotInfo
    {
        /// @brief Default constructor
        PlotInfo() = default;

        /// @brief Constructor
        /// @param[in] title Title of the ImPlot
        /// @param[in] nInputPins Amount of inputPins
        PlotInfo(const std::string& title, size_t nInputPins)
            : title(title), headerText(title), selectedXdata(nInputPins, 0) {}

        /// Title of the ImPlot
        std::string title;
        /// Title of the CollapsingHeader
        std::string headerText;
        /// Selected pin in the GUI for the Drag & Drop Data
        int selectedPin = 0;
        /// Flags which are passed to the plot
        int plotFlags = 0;
        /// Flag whether to automaticaly set the x-Axis limits
        bool autoLimitXaxis = true;
        /// Flag whether to automaticaly set the y-Axis limits
        bool autoLimitYaxis = true;
        /// @brief Key: PinIndex, Value: plotData to use for x-Axis
        std::vector<size_t> selectedXdata;

        /// Width of plot Data content
        float leftPaneWidth = 180.0F;
        /// Width of the plot
        float rightPaneWidth = 400.0F;
    };

  private:
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Input Pins depending on the variable nInputPins
    void updateNumberOfInputPins();

    /// @brief Adds/Deletes Plots depending on the variable nPlots
    void updateNumberOfPlots();

    /// @brief Add Data to the buffer of the pin
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param[in] dataIndex Index of the data to insert
    /// @param[in] value The value to insert
    void addData(size_t pinIndex, size_t dataIndex, double value);

    /// @brief Plots the data on this port
    /// @param[in] linkId Id of the link over which the data is received
    void plotBoolean(ax::NodeEditor::LinkId linkId);

    /// @brief Plots the data on this port
    /// @param[in] linkId Id of the link over which the data is received
    void plotInteger(ax::NodeEditor::LinkId linkId);

    /// @brief Plots the data on this port
    /// @param[in] linkId Id of the link over which the data is received
    void plotFloat(ax::NodeEditor::LinkId linkId);

    /// @brief Plots the data on this port
    /// @param[in] linkId Id of the link over which the data is received
    void plotMatrix(ax::NodeEditor::LinkId linkId);

    /// @brief Plot the data on this port
    /// @param[in] nodeData Data to plot
    /// @param[in] linkId Id of the link over which the data is received
    void plotData(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotPosVelAtt(const std::shared_ptr<PosVelAtt>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotRtklibPosObs(const std::shared_ptr<RtklibPosObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotUbloxObs(const std::shared_ptr<UbloxObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotImuObs(const std::shared_ptr<ImuObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotKvhObs(const std::shared_ptr<KvhObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotImuObsWDeltaObs(const std::shared_ptr<ImuObsWDelta>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotVectorNavBinaryObs(const std::shared_ptr<VectorNavBinaryOutput>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotSkydelObs(const std::shared_ptr<SkydelObs>& obs, size_t pinIndex);

    /// Data storage for each pin
    std::vector<PinData> data;

    /// Info for each plot window
    std::vector<PlotInfo> plotInfos;

    /// Amount of input pins (should equal data.size())
    int nInputPins = 5;
    /// Amount of plot windows (should equal plotInfos.size())
    int nPlots = 0;
    /// Possible data identifiers to connect
    std::vector<std::string> dataIdentifier;

    /// Start Time for calculation of relative time with the GPS ToW
    double startValue_Time = std::nan("");
    /// Start Latitude [rad] for calculation of relative North-South
    double startValue_North = std::nan("");
    /// Start Longitude [rad] for calculation of relative East-West
    double startValue_East = std::nan("");
};

} // namespace NAV
