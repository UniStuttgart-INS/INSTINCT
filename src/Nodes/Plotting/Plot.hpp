/// @file Plot.hpp
/// @brief Plots data into ImPlot Windows
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include <implot.h>

#include <map>

#include "internal/Node/Node.hpp"

#include "util/Container/ScrollingBuffer.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/State/PVAError.hpp"
#include "NodeData/State/ImuBiases.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/KvhObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

namespace NAV
{
/// @brief Plot node which plots all kind of observations
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
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
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

    /// @brief Specifying the look of a certain line in the plot
    struct PlotStyle
    {
        /// @brief Possible line types
        enum class LineType : int
        {
            Scatter, ///< Scatter plot (only markers)
            Line,    ///< Line plot
        };

        /// Display name in the legend (if not set falls back to PlotData::displayName)
        std::string legendName;
        /// Legend name which gets changed in the gui
        std::string legendNameGui;
        /// Line type
        LineType lineType = LineType::Line;
        /// Line Color
        ImVec4 color = IMPLOT_AUTO_COL;
        /// Line thickness
        float thickness = 1.0F;

        /// Amount of points to skip for plotting
        int stride = 0;

        /// Display markers for the line plot (no effect for scatter type)
        bool markers = false;
        /// Style of the marker to display
        ImPlotMarker markerStyle = ImPlotMarker_Cross;
        /// Size of the markers (makes the marker smaller/bigger)
        float markerSize = 1.0F;
        /// Weight of the markers (increases thickness of marker lines)
        float markerWeight = 1.0F;
        /// Fill color for markers
        ImVec4 markerFillColor = IMPLOT_AUTO_COL;
        /// Outline/Border color for markers
        ImVec4 markerOutlineColor = IMPLOT_AUTO_COL;
    };

    /// @brief Information needed to plot the data on a certain pin
    struct PinData
    {
        /// @brief Stores the actual data coming from a pin
        struct PlotData
        {
            /// @brief Default constructor
            PlotData() = default;

            /// @brief Constructor
            /// @param[in] displayName Display name of the contained data
            /// @param[in] size Size of the buffer
            explicit PlotData(std::string displayName, size_t size)
                : displayName(std::move(displayName)), buffer(size, PinData::SCROLLING_BUFFER_PADDING) {}

            /// Display name of the contained data
            std::string displayName;
            /// Buffer for the data
            ScrollingBuffer<double> buffer;
            /// Flag if data was received, as the buffer contains std::nan("") otherwise
            bool hasData = false;
            /// Key: PlotIndex; Value: <yAxisIndex, plotStyle>
            std::map<size_t, std::pair<int, PlotStyle>> plotOnAxis;

            /// When connecting a new link. All data is flagged for delete and only those who are also present in the new link are kept
            bool markedForDelete = false;
        };

        /// @brief Possible Pin types
        enum class PinType : int
        {
            Flow,   ///< NodeData Trigger
            Bool,   ///< Boolean
            Int,    ///< Integer Number
            Float,  ///< Floating Point Number
            Matrix, ///< Matrix Object
        };

        /// @brief Adds a plotData Element to the list
        /// @param[in] dataIndex Index where to add the data to
        /// @param[in] displayName Display name of the contained data
        void addPlotDataItem(size_t dataIndex, const std::string& displayName)
        {
            if (plotData.size() > dataIndex)
            {
                if (plotData.at(dataIndex).displayName == displayName) // Item was restored already at this position
                {
                    plotData.at(dataIndex).markedForDelete = false;
                    return;
                }

                // Some other item was restored at this position
                if (!plotData.at(dataIndex).markedForDelete)
                {
                    LOG_WARN("Adding PlotData item '{}' at position {}, but at this position exists already the item '{}'. Reordering the items to match the data. Consider resaving the flow file.",
                             displayName, dataIndex, plotData.at(dataIndex).displayName);
                }
                auto searchIter = std::find_if(plotData.begin(),
                                               plotData.end(),
                                               [displayName](const PlotData& plotData) { return plotData.displayName == displayName; });
                auto iter = plotData.begin();
                std::advance(iter, dataIndex);
                if (searchIter == plotData.end()) // Item does not exist yet. Developer added a new item to the list
                {
                    plotData.insert(iter, PlotData{ displayName, static_cast<size_t>(size) });
                }
                else // Item exists already. Developer reordered the items in the list
                {
                    auto tmpPlotData = *searchIter;     // Copy the oldelement
                    plotData.erase(searchIter);         // Delete the old element
                    plotData.insert(iter, tmpPlotData); // Put the found element at the right position
                }
                iter->markedForDelete = false;
            }
            else if (std::find_if(plotData.begin(),
                                  plotData.end(),
                                  [displayName](const PlotData& plotData) { return plotData.displayName == displayName; })
                     != plotData.end())
            {
                LOG_ERROR("Adding the PlotData item {} at position {}, but this plot item was found at another position already",
                          displayName, dataIndex);
            }
            else // Item not there yet. Add to the end of the list
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
        /// Pin Type
        PinType pinType = PinType::Flow;
        /// Amount of points to skip for plotting
        int stride = 1;

        /// Extra data points for the buffer to avoid connecting the first and last point
        static constexpr size_t SCROLLING_BUFFER_PADDING = 0;
    };

    /// @brief Information specifying the look of each plot
    struct PlotInfo
    {
        /// @brief Default constructor
        PlotInfo() = default;

        /// @brief Constructor
        /// @param[in] title Title of the ImPlot
        /// @param[in] nInputPins Amount of inputPins
        PlotInfo(const std::string& title, size_t nInputPins)
            : title(title), headerText(title), selectedXdata(nInputPins, 0) {}

        /// Size of the plot
        ImVec2 size{ -1, 300 };

        /// Title of the ImPlot
        std::string title;
        /// Title of the CollapsingHeader
        std::string headerText;
        /// Flag, whether to override the x axis label
        bool overrideXAxisLabel = false;
        /// X axis label
        std::string xAxisLabel;
        /// Y1 axis label
        std::string y1AxisLabel;
        /// Y2 axis label
        std::string y2AxisLabel;
        /// Y3 axis label
        std::string y3AxisLabel;
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

    /// @brief Adds/Deletes Input Pins depending on the variable _nInputPins
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
    void plotData(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotPosVelAtt(const std::shared_ptr<const PosVelAtt>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotPVAError(const std::shared_ptr<const PVAError>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotImuBiases(const std::shared_ptr<const ImuBiases>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotRtklibPosObs(const std::shared_ptr<const RtklibPosObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotUbloxObs(const std::shared_ptr<const UbloxObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotImuObs(const std::shared_ptr<const ImuObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotKvhObs(const std::shared_ptr<const KvhObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotImuObsWDeltaObs(const std::shared_ptr<const ImuObsWDelta>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotVectorNavBinaryObs(const std::shared_ptr<const VectorNavBinaryOutput>& obs, size_t pinIndex);

    /// Data storage for each pin
    std::vector<PinData> _pinData;

    /// Info for each plot window
    std::vector<PlotInfo> _plotInfos;

    /// Amount of input pins (should equal data.size())
    int _nInputPins = 5;
    /// Amount of plot windows (should equal _plotInfos.size())
    int _nPlots = 0;
    /// Possible data identifiers to connect
    std::vector<std::string> _dataIdentifier;

    /// Start Time for calculation of relative time with the GPS ToW
    double _startValue_Time = std::nan("");
    /// Start Latitude [rad] for calculation of relative North-South
    double _startValue_North = std::nan("");
    /// Start Longitude [rad] for calculation of relative East-West
    double _startValue_East = std::nan("");

    /// Flag, whether to override the North/East startValues in the GUI
    bool _overridePositionStartValues = false;
};

} // namespace NAV
