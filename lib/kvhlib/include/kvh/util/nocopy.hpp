#pragma once

#include "kvh/xplat/export.hpp"

namespace kvh::util
{
/// \brief Identifies a derived class as being unable to be copied and prevents copy attempts.
class kvh_proglib_DLLEXPORT NoCopy
{
  protected:
    /// \brief Allows construction of derived objects.
    NoCopy() {}

    /// \brief Allows destruction of derived objects.
    ~NoCopy() {}

  private:
    /// \brief Prevent copying of derived objects.
    NoCopy(const NoCopy&);

    /// \brief Prevent assignment copying of derived objects.
    NoCopy& operator=(const NoCopy&);
};

} // namespace kvh::util