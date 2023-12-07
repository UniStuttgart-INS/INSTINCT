// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Plot.hpp
/// @brief Plots data into ImPlot Windows
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include <implot.h>

#include <map>
#include <mutex>

#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/PositionInput.hpp"

#include "util/Container/ScrollingBuffer.hpp"
#include "util/Container/Vector.hpp"
#include "util/Plot/Colormap.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/GNSS/GnssCombination.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/SppSolution.hpp"
#include "NodeData/State/LcKfInsGnssErrors.hpp"
#include "NodeData/State/TcKfInsGnssErrors.hpp"
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include "NodeData/GNSS/UbloxObs.hpp"
#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsSimulated.hpp"
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
    void afterCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Information needed to plot the data on a certain pin
    struct PinData
    {
        /// @brief Stores the actual data coming from a pin
        struct PlotData
        {
            /// @brief Default constructor (needed to make serialization with json working)
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

            /// When connecting a new link. All data is flagged for delete and only those who are also present in the new link are kept
            bool markedForDelete = false;
            /// Bool to show if dynamic data
            bool isDynamic = false;
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

        /// @brief Constructor
        PinData() = default;
        /// @brief Destructor
        ~PinData() = default;
        /// @brief Copy constructor
        /// @param[in] other The other element to copy
        PinData(const PinData& other)
            : size(other.size),
              dataIdentifier(other.dataIdentifier),
              plotData(other.plotData),
              pinType(other.pinType),
              stride(other.stride) {}

        /// @brief Move constructor
        /// @param[in] other The other element to move
        PinData(PinData&& other) noexcept
            : size(other.size),
              dataIdentifier(std::move(other.dataIdentifier)),
              plotData(std::move(other.plotData)),
              pinType(other.pinType),
              stride(other.stride) {}

        /// @brief Copy assignment operator
        /// @param[in] rhs The other element to copy
        PinData& operator=(const PinData& rhs)
        {
            if (&rhs != this)
            {
                size = rhs.size;
                dataIdentifier = rhs.dataIdentifier;
                plotData = rhs.plotData;
                pinType = rhs.pinType;
                stride = rhs.stride;
            }

            return *this;
        }
        /// @brief Move assignment operator
        /// @param[in] rhs The other element to move
        PinData& operator=(PinData&& rhs) noexcept
        {
            if (&rhs != this)
            {
                size = rhs.size;
                dataIdentifier = std::move(rhs.dataIdentifier);
                plotData = std::move(rhs.plotData);
                pinType = rhs.pinType;
                stride = rhs.stride;
            }

            return *this;
        }

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
                iter->markedForDelete = false;
                if (searchIter == plotData.end()) // Item does not exist yet. Developer added a new item to the list
                {
                    plotData.insert(iter, PlotData{ displayName, static_cast<size_t>(size) });
                }
                else // Item exists already. Developer reordered the items in the list
                {
                    move(plotData, static_cast<size_t>(searchIter - plotData.begin()), dataIndex);
                }
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
        /// Mutex to lock the buffer so that the GUI thread and the calculation threads don't cause a data race
        std::mutex mutex;
        /// Dynamic data start index
        int dynamicDataStartIndex = -1;
        /// Events with relative time, absolute time and tooltip text
        std::vector<std::tuple<double, InsTime, std::string>> events;
    };

    /// @brief Information specifying the look of each plot
    struct PlotInfo
    {
        /// Info needed to draw a data set
        struct PlotItem
        {
            /// @brief Specifying the look of a certain line in the plot
            struct Style
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
                /// Colormap mask (pair: type and id)
                std::pair<ColormapMaskType, int64_t> colormapMask = { ColormapMaskType::None, -1 };
                /// Index of the plot data to compare for the color
                size_t colormapMaskDataCmpIdx = 0;
                /// Line thickness
                float thickness = 1.0F;

                /// Amount of points to skip for plotting
                int stride = 0;

                /// Colormap mask (pair: type and id)
                std::pair<ColormapMaskType, int64_t> markerColormapMask = { ColormapMaskType::None, -1 };
                /// Index of the plot data to compare for the color
                size_t markerColormapMaskDataCmpIdx = 0;
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

                /// Show events on this data
                bool eventsEnabled = false;
                /// Style of the marker to display
                ImPlotMarker eventMarkerStyle = ImPlotMarker_Cross;
                /// Size of the markers (makes the marker smaller/bigger)
                float eventMarkerSize = 1.0F;
                /// Weight of the markers (increases thickness of marker lines)
                float eventMarkerWeight = 1.0F;
                /// Fill color for markers
                ImVec4 eventMarkerFillColor = IMPLOT_AUTO_COL;
                /// Outline/Border color for markers
                ImVec4 eventMarkerOutlineColor = IMPLOT_AUTO_COL;
                /// Tooltip search regex
                std::string eventTooltipFilterRegex;
            };

            /// @brief Default constructor (needed to make serialization with json working)
            PlotItem() = default;

            /// @brief Constructor
            /// @param[in] pinIndex Index of the pin where the data came in
            /// @param[in] dataIndex Index of the data on the pin
            /// @param[in] displayName Display name of the data
            PlotItem(size_t pinIndex, size_t dataIndex, std::string displayName)
                : pinIndex(pinIndex), dataIndex(dataIndex), displayName(std::move(displayName))
            {
                style.colormapMaskDataCmpIdx = dataIndex;
                style.markerColormapMaskDataCmpIdx = dataIndex;
            }

            /// @brief Constructor
            /// @param[in] pinIndex Index of the pin where the data came in
            /// @param[in] dataIndex Index of the data on the pin
            /// @param[in] displayName Display name of the data
            /// @param[in] axis Axis to plot the data on (Y1, Y2, Y3)
            PlotItem(size_t pinIndex, size_t dataIndex, std::string displayName, ImAxis axis)
                : pinIndex(pinIndex), dataIndex(dataIndex), displayName(std::move(displayName)), axis(axis)
            {
                style.colormapMaskDataCmpIdx = dataIndex;
                style.markerColormapMaskDataCmpIdx = dataIndex;
            }

            /// @brief Equal comparison operator (needed to search the vector with std::find)
            /// @param[in] rhs Right-hand-side of the operator
            /// @return True if the pin and data indices match
            constexpr bool operator==(const PlotItem& rhs) const
            {
                return pinIndex == rhs.pinIndex && dataIndex == rhs.dataIndex && displayName == rhs.displayName;
            }

            size_t pinIndex{};        ///< Index of the pin where the data came in
            size_t dataIndex{};       ///< Index of the data on the pin
            std::string displayName;  ///< Display name of the data (not changing and unique)
            ImAxis axis{ ImAxis_Y1 }; ///< Axis to plot the data on (Y1, Y2, Y3)
            Style style{};            ///< Defines how the data should be plotted
            /// Buffer for the colormap mask
            ScrollingBuffer<ImU32> colormapMaskColors = ScrollingBuffer<ImU32>(0);
            /// Colormap version (to track updates of the colormap)
            size_t colormapMaskVersion = 0;
            /// Buffer for the colormap mask
            ScrollingBuffer<ImU32> markerColormapMaskColors = ScrollingBuffer<ImU32>(0);
            /// Colormap version (to track updates of the colormap)
            size_t markerColormapMaskVersion = 0;

            /// Buffer for event markers
            ScrollingBuffer<double> eventMarker = ScrollingBuffer<double>(0);

            /// Tooltip info
            struct Tooltip
            {
                InsTime time;                   ///< Time of the event
                std::vector<std::string> texts; ///< List of event texts
            };
            /// List of tooltips (x,y, tooltip)
            std::vector<std::tuple<double, double, Tooltip>> eventTooltips;
        };

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
        size_t selectedPin = 0;
        /// Flags which are passed to the plot
        int plotFlags = 0;
        /// Flags for the x-Axis
        ImPlotAxisFlags xAxisFlags = ImPlotAxisFlags_AutoFit;
        /// Flags for the y-Axes
        ImPlotAxisFlags yAxisFlags = ImPlotAxisFlags_AutoFit;
        /// Scale for the x-Axis
        ImPlotScale xAxisScale = ImPlotScale_Linear;
        /// Scale for the y-Axes
        std::array<ImPlotScale, 3> yAxesScale = { ImPlotScale_Linear, ImPlotScale_Linear, ImPlotScale_Linear };

        /// @brief Key: PinIndex, Value: plotData to use for x-Axis
        std::vector<size_t> selectedXdata;

        /// List containing all elements which should be plotted
        std::vector<PlotItem> plotItems;

        /// Width of plot Data content
        float leftPaneWidth = 180.0F;
        /// Width of the plot
        float rightPaneWidth = 400.0F;

        /// Flag whether the whole plot is visible. If not, then it should be deleted
        bool visible = true;
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

    /// @brief Adds a event to a certain point in time
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param relTime Relative time in [s]
    /// @param insTime Absolute time
    /// @param text Text to display
    void addEvent(size_t pinIndex, double relTime, InsTime insTime, const std::string& text);

    /// @brief Add Data to the buffer of the pin
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param[in] dataIndex Index of the data to insert
    /// @param[in] value The value to insert
    void addData(size_t pinIndex, size_t dataIndex, double value);

    /// @brief Add Data to the buffer of the pin
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param[in] displayName Display name of the data
    /// @param[in] value The value to insert
    void addData(size_t pinIndex, std::string displayName, double value);

    /// @brief Plots the data on this port
    /// @param[in] insTime Time the data was received
    /// @param[in] pinIdx Index of the pin the data is received on
    void plotBoolean(const InsTime& insTime, size_t pinIdx);

    /// @brief Plots the data on this port
    /// @param[in] insTime Time the data was received
    /// @param[in] pinIdx Index of the pin the data is received on
    void plotInteger(const InsTime& insTime, size_t pinIdx);

    /// @brief Plots the data on this port
    /// @param[in] insTime Time the data was received
    /// @param[in] pinIdx Index of the pin the data is received on
    void plotFloat(const InsTime& insTime, size_t pinIdx);

    /// @brief Plots the data on this port
    /// @param[in] insTime Time the data was received
    /// @param[in] pinIdx Index of the pin the data is received on
    void plotMatrix(const InsTime& insTime, size_t pinIdx);

    /// @brief Plot the data on this port
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void plotData(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotPos(const std::shared_ptr<const Pos>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotPosVel(const std::shared_ptr<const PosVel>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotPosVelAtt(const std::shared_ptr<const PosVelAtt>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotLcKfInsGnssErrors(const std::shared_ptr<const LcKfInsGnssErrors>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotTcKfInsGnssErrors(const std::shared_ptr<const TcKfInsGnssErrors>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotGnssCombination(const std::shared_ptr<const GnssCombination>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotGnssObs(const std::shared_ptr<const GnssObs>& obs, size_t pinIndex);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotSppSolution(const std::shared_ptr<const SppSolution>& obs, size_t pinIndex);

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
    void plotImuObsSimulated(const std::shared_ptr<const ImuObsSimulated>& obs, size_t pinIndex);

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
    std::vector<PlotInfo> _plots;

    /// Amount of input pins (should equal data.size())
    size_t _nInputPins = 5;
    /// Amount of plot windows (should equal _plots.size())
    size_t _nPlots = 0;
    /// Possible data identifiers to connect
    std::vector<std::string> _dataIdentifier;

    /// Index of the Collapsible Header currently being dragged
    int _dragAndDropHeaderIndex = -1;
    /// Index of the Pin currently being dragged
    int _dragAndDropPinIndex = -1;

    /// Start Time for calculation of relative time with the GPS ToW
    InsTime _startTime;
    /// Start position for the calculation of relative North-South and East-West
    std::optional<gui::widgets::PositionWithFrame> _originPosition;

    /// Flag, whether to override the North/East startValues in the GUI
    bool _overridePositionStartValues = false;
};

} // namespace NAV
