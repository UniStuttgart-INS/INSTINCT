#pragma once

namespace ub::util
{
/// \brief Identifies a derived class as being unable to be copied and prevents copy attempts.
class NoCopy
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

} // namespace ub::util