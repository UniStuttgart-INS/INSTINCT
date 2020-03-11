#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>

namespace ub::util
{
/// \brief Helpful class for working checksums.
class ubloxChecksum
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

    /// \brief Computes the NMEA checksum of the provided data.
    ///
    /// \param[in] data The data array to compute the 16-bit checksum for.
    /// \param[in] length The length of data bytes from the array to compute
    ///     the checksum over.
    /// \return The computed checksum.
    static uint8_t checksumNMEA(const char data[], size_t length);
};

} // namespace ub::util