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

// <boost/asio.hpp> needs to be included before <winsock.h> (even though not used in this file)
// https://stackoverflow.com/questions/9750344/boostasio-winsock-and-winsock-2-compatibility-issue
#ifdef _WIN32
    // Set the proper SDK version before including boost/Asio
    #include <SDKDDKVer.h>
    // Note boost/ASIO includes Windows.h.
    #include <boost/asio.hpp>
#endif //_WIN32

#include <array>
#include <imgui.h>
#include <implot.h>

#include <map>
#include <memory>
#include <mutex>
#include <unordered_set>

#include "NodeData/NodeData.hpp"
#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"
#include "internal/gui/widgets/PositionInput.hpp"

#include "util/Container/ScrollingBuffer.hpp"
#include "util/Container/Vector.hpp"
#include "util/Plot/PlotEventTooltip.hpp"
#include "util/Plot/PlotItemStyle.hpp"
#include "util/Plot/PlotTooltip.hpp"
#include "util/Logger/CommonLog.hpp"

#include "NodeData/General/DynamicData.hpp"
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
#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InsGnssTCKFSolution.hpp"
#include "NodeData/WiFi/WiFiPositioningSolution.hpp"

namespace NAV
{
/// @brief Plot node which plots all kind of observations
class Plot : public Node, public CommonLog
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
            PlotData(std::string displayName, size_t size);

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
        enum class PinType : uint8_t
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
        PinData(const PinData& other);

        /// @brief Move constructor
        /// @param[in] other The other element to move
        PinData(PinData&& other) noexcept;

        /// @brief Copy assignment operator
        /// @param[in] rhs The other element to copy
        PinData& operator=(const PinData& rhs);
        /// @brief Move assignment operator
        /// @param[in] rhs The other element to move
        PinData& operator=(PinData&& rhs) noexcept;

        /// @brief Adds a plotData Element to the list
        /// @param[in] dataIndex Index where to add the data to
        /// @param[in] displayName Display name of the contained data
        void addPlotDataItem(size_t dataIndex, const std::string& displayName);

        /// Size of all buffers of the plotData elements
        int size = 0;
        /// Data Identifier of the connected pin
        std::string dataIdentifier;
        /// List with all the data
        std::vector<PlotData> plotData;
        /// List with the raw data received
        ScrollingBuffer<std::shared_ptr<const NodeData>> rawNodeData;
        /// Pin Type
        PinType pinType = PinType::Flow;
        /// Amount of points to skip for plotting
        int stride = 1;
        /// Mutex to lock the buffer so that the GUI thread and the calculation threads don't cause a data race
        std::mutex mutex;
        /// Dynamic data start index
        int dynamicDataStartIndex = -1;
        /// Events with relative time, absolute time, tooltip text and data Index (-1 means all)
        std::vector<std::tuple<double, InsTime, std::string, int32_t>> events;
    };

    /// @brief Information specifying the look of each plot
    struct PlotInfo
    {
        /// Info needed to draw a data set
        struct PlotItem
        {
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
                : PlotItem(pinIndex, dataIndex, std::move(displayName))
            {
                this->axis = axis; // NOLINT(cppcoreguidelines-prefer-member-initializer)
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
            PlotItemStyle style{};    ///< Defines how the data should be plotted
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

            /// Error bounds lower and upper data
            std::array<ScrollingBuffer<double>, 2> errorBoundsData;

            /// List of tooltips (x,y, tooltip)
            std::vector<std::tuple<double, double, PlotEventTooltip>> eventTooltips;
        };

        /// @brief Default constructor
        PlotInfo() = default;

        /// @brief Constructor
        /// @param[in] title Title of the ImPlot
        /// @param[in] nInputPins Amount of inputPins
        PlotInfo(const std::string& title, size_t nInputPins)
            : title(title), headerText(title), selectedXdata(nInputPins, 1) {}

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
        /// Line Flags for all items (each item can override the selection)
        ImPlotLineFlags lineFlags = ImPlotLineFlags_NoClip | ImPlotLineFlags_SkipNaN;

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

        /// List of tooltip windows to show
        std::vector<PlotTooltip> tooltips;
    };

  private:
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Plots depending on the variable nPlots
    void updateNumberOfPlots();

    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// Index of the GPST data (unix timestamp)
    size_t GPST_PLOT_IDX = 1;

    /// Data storage for each pin
    std::vector<PinData> _pinData;

    /// Info for each plot window
    std::vector<PlotInfo> _plots;

    /// Amount of plot windows (should equal _plots.size())
    size_t _nPlots = 0;
    /// Possible data identifiers to connect
    std::vector<std::string> _dataIdentifier = {
        // General
        DynamicData::type(),
        // GNSS
        GnssCombination::type(),
        GnssObs::type(),
        RtklibPosObs::type(),
        SppSolution::type(),
        // IMU
        ImuObs::type(),
        ImuObsSimulated::type(),
        ImuObsWDelta::type(),
        KvhObs::type(),
        VectorNavBinaryOutput::type(),
        // State
        InsGnssLCKFSolution::type(),
        Pos::type(),
        PosVel::type(),
        PosVelAtt::type(),
        InsGnssTCKFSolution::type(),
        // WiFi
        WiFiPositioningSolution::type(),
    };

    /// Index of the Collapsible Header currently being dragged
    int _dragAndDropHeaderIndex = -1;

    size_t _screenshotFrameCnt = 0; ///< Frame counter for taking screenshots (> 0 when screenshot in progress)
    size_t _screenShotPlotIdx = 0;  ///< Plot index a screenshot is taken of

    /// Values to force the x axis range to and a set of plotIdx to force
    std::pair<std::unordered_set<size_t>, ImPlotRange> _forceXaxisRange;

    /// Start position for the calculation of relative North-South and East-West
    std::optional<gui::widgets::PositionWithFrame> _originPosition;

    /// Flag, whether to override the North/East startValues in the GUI
    bool _overridePositionStartValues = false;

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ 0, this, pinAddCallback, pinDeleteCallback, 1 };

    /// @brief Adds a event to a certain point in time
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param insTime Absolute time
    /// @param text Text to display
    /// @param dataIndex Data Index to add the event for (-1 means all)
    void addEvent(size_t pinIndex, InsTime insTime, const std::string& text, int32_t dataIndex);

    /// @brief Add Data to the buffer of the pin
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param[in] dataIndex Index of the data to insert
    /// @param[in] value The value to insert
    void addData(size_t pinIndex, size_t dataIndex, double value);

    /// @brief Add Data to the buffer of the pin
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param[in] displayName Display name of the data
    /// @param[in] value The value to insert
    /// @return Data Index where data were inserted
    size_t addData(size_t pinIndex, std::string displayName, double value);

    /// @brief Calculate the local position offset from the plot origin
    /// @param[in] lla_position [𝜙, λ, h] Latitude, Longitude, Altitude in [rad, rad, m]
    /// @return Local positions in north/south and east/west directions in [m]
    CommonLog::LocalPosition calcLocalPosition(const Eigen::Vector3d& lla_position);

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
    void plotFlowData(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    /// @param[in, out] plotIndex Index for inserting the data into the plot data vector
    /// @param[in] startIndex Data descriptor start index
    template<typename T>
    void plotData(const std::shared_ptr<const T>& obs, size_t pinIndex, size_t& plotIndex, size_t startIndex = 0)
    {
        for (size_t i = startIndex; i < T::GetStaticDescriptorCount(); ++i)
        {
            addData(pinIndex, plotIndex++, obs->getValueAtOrNaN(i));
        }
    }
};

} // namespace NAV
