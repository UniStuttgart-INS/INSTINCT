/**
 * @file NodeCreator.hpp
 * @brief Defines routines to automatically create nodes from program options
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-03
 */

#pragma once

#include <string>

#include "util/Common.hpp"
#include "util/Config.hpp"

namespace NAV
{
/// Static Class for creating Nodes from program options
class NodeCreator
{
  public:
    /**
     * @brief Creates Nodes from the program options
     * 
     * @param[in] pConfig The program options
     * @retval NavStatus Indicates if the creation was successfull
     */
    static NavStatus createNodes(NAV::Config* pConfig);

    /**
     * @brief Links Node callbacks depending on the program options
     * 
     * @param[in] pConfig The program options
     * @retval NavStatus Indicates if the linking was successfull
     */
    static NavStatus createLinks(NAV::Config* pConfig);

    /// Constructor is deleted
    NodeCreator() = delete;
};

} // namespace NAV
