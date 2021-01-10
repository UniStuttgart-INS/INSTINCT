/// @file Plot.hpp
/// @brief Plots data into ImPlot Windows
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-09

#pragma once

#include "Nodes/Node.hpp"

#include <map>

#include "util/ScrollingBuffer.hpp"

#include "NodeData/IMU/VectorNavObs.hpp"

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

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(Pin* startPin, Pin* endPin) override;

    /// @brief Called when a link is to be deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void onDeleteLink(Pin* startPin, Pin* endPin) override;

  private:
    constexpr static size_t OutputPortIndex_Plot = 0; ///< @brief Delegate

    /// @brief Plot the data on this port
    /// @param[in] nodeData Data to plot
    /// @param[in] linkId Id of the link over which the data is received
    void plotData(std::shared_ptr<NodeData> nodeData, ax::NodeEditor::LinkId linkId);

    /// @brief Plot the data
    /// @param[in] obs Observation to plot
    /// @param[in] pinIndex Index of the input pin where the data was received
    void plotVectorNavObs(std::shared_ptr<VectorNavObs> obs, size_t pinIndex);

    /// @brief Checks if the given data type is supported for link creation
    /// @param[in] dataIdentifier Type to check
    /// @return True if supported, false if not
    static bool isDataTypeSupported(std::string_view dataIdentifier);

    /// @brief Adds Input Pins depending on the variable nInputPins
    void updateNumberOfInputPins();

    struct PinData
    {
        struct PlotData
        {
            explicit PlotData(std::string displayName, size_t size)
                : displayName(std::move(displayName)), buffer(size) {}

            std::string displayName;
            ScrollingBuffer<double> buffer;
            bool show = false;
        };

        void addPlotDataItem(std::string displayName)
        {
            plotData.emplace_back(displayName, size);
            allDisplayNames.push_back(displayName);
        }

        size_t size = 2000;
        std::vector<PlotData> plotData;
        std::vector<std::string> allDisplayNames;
    };

    std::vector<PinData> data;

    struct PlotInfo
    {
        struct DataToPlot
        {
            size_t dataIndex = 0;
            size_t yAxis = 0;
        };

        PlotInfo(const std::string& title, size_t nInputPins)
            : title(title), headerText(title), selectedXdata(nInputPins, 0)
        {
            while (dataToPlot.size() < nInputPins)
            {
                dataToPlot[dataToPlot.size()] = {};
            }
        }

        std::string title;
        std::string headerText;
        int selectedPin = 0;
        int plotFlags = 0;
        /// @brief Key: PinIndex, Value: plotData to use for x-Axis
        std::vector<size_t> selectedXdata;
        /// @brief Key: PinIndex, Value: Data To Plot
        std::map<size_t, std::vector<DataToPlot>> dataToPlot;
    };

    std::vector<PlotInfo> plotInfos;

    int nInputPins = 1;
    int nPlots = 0;

    double startValue_Time = std::nan("");
    // double startValue_North = std::nan("");
    // double startValue_East = std::nan("");
};

} // namespace NAV
