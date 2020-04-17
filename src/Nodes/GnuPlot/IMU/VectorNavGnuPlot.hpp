/**
 * @file VectorNavGnuPlot.hpp
 * @brief Plots VectorNav Imu Data
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-14
 */

#pragma once

#include "../GnuPlot.hpp"

namespace NAV
{
/// Plots VectorNav Imu Data
class VectorNavGnuPlot : public GnuPlot
{
  public:
    /**
     * @brief Construct a new VectorNav Gnu Plot object
     * 
     * @param[in] name Name of the Node
     * @param[in, out] options Program options string list
     */
    VectorNavGnuPlot(std::string name, std::deque<std::string>& options);

    /// Default Destructor
    ~VectorNavGnuPlot();

    /**
     * @brief Plots an VectorNav Observation
     * 
     * @param[in] observation The received observation
     * @param[in] userData User data specified when registering the callback
     * @retval NavStatus Indicates whether the plot was successfull.
     */
    static NavStatus plotVectorNavObs(std::shared_ptr<NodeData> observation, std::shared_ptr<Node> userData);

  private:
    uint64_t timeStart = 0;
};

} // namespace NAV
