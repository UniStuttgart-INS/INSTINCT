#pragma once

#include <cstddef>
#include "uart/xplat/int.hpp"
#include <utility>

namespace uart::util
{
/// \brief Helpful class for working checksums.
class Checksum
{
    // Public Methods /////////////////////////////////////////////////////////

  public:
    /// \brief Computes the binary checksums of the provided data.
    ///
    /// \param[in] data The data array to compute the checksums for.
    /// \param[in] length The length of data bytes from the array to compute the checksum over.
    /// \return The computed checksum.
    static bool checksumBinary(const unsigned char data[], size_t length);

    /// \brief Computes the ASCII checksum of the provided data.
    ///
    /// \param[in] data The data array to compute the checksum for.
    /// \param[in] length The length of data bytes from the array to compute the checksum over.
    /// \return The computed checksum.
    static bool checksumASCII(const unsigned char data[], size_t length);
};

} // namespace uart::util