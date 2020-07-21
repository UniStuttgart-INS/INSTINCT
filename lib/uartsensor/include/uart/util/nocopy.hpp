#pragma once

#include "uart/xplat/export.hpp"

namespace uart::util
{
/// \brief Identifies a derived class as being unable to be copied and prevents copy attempts.
class proglib_DLLEXPORT NoCopy
{
  protected:
    /// \brief Allows construction of derived objects.
    NoCopy() = default;

    /// \brief Allows destruction of derived objects.
    ~NoCopy() = default;

  public:
    /// \brief Prevent copying of derived objects.
    NoCopy(const NoCopy&) = delete;
    /// \brief Prevent assignment copying of derived objects.
    NoCopy& operator=(const NoCopy&) = delete;

    /// \brief Move constructor
    NoCopy(NoCopy&&) = default;
    /// \brief Move assignment operator
    NoCopy& operator=(NoCopy&&) = default;
};

} // namespace uart::util