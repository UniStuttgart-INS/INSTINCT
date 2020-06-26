#pragma once

#include <cstddef>
#include "er/xplat/int.hpp"
#include <utility>

namespace er::util
{
/// \brief Helpful class for working checksums.
class emlidChecksum
{
    // Public Methods /////////////////////////////////////////////////////////

  public:
    /// \brief Computes the 2 UBX checksums of the provided data.
    ///
    /// \param[in] data The data array to compute the checksums for.
    /// \param[in] length The length of data bytes from the array to compute
    ///     the checksum over.
    /// \return The computed checksums CK_A and CK_B.
    static std::pair<unsigned char, unsigned char> checksumUBX(const unsigned char data[], size_t length);
};

} // namespace er::util