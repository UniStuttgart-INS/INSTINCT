#pragma once

#include <cstddef>
#include "kvh/xplat/int.hpp"
#include <utility>

namespace kvh::util
{
/// \brief Helpful class for working checksums.
class kvhChecksum
{
    // Public Methods /////////////////////////////////////////////////////////

  public:
    /*******************************************************************************
     *                  ui32CalcImuCRC()
     *
     *       Author: KVH
     *
     *   Start Date: circa 2012
     *
     *  Description: CRC calculator for IMU data packet
     *
     *       http://en.wikipedia.org/wiki/Cyclic_redundancy_check#Commonly_used_and_standardized_CRCs
     *
     *       Inputs: pui8Data  pointer to data to CRC
     *               iLength   the size of the data in bytes
     *
     *      Returns: 32-bit CRC
     *
     * Side Effects: n/a
     ******************************************************************************/
    static uint32_t ui32CalcImuCRC(uint8_t* pui8Data, int iLength);
};

} // namespace kvh::util