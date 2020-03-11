#pragma once

#include "ub/util/nocopy.hpp"

namespace ub::xplat
{
/// \brief Represents a cross-platform critical section.
class CriticalSection : private util::NoCopy
{
    // Constructors ///////////////////////////////////////////////////////////

  public:
    /// \brief Creates a new critical section.
    CriticalSection();

    ~CriticalSection();

    // Public Methods /////////////////////////////////////////////////////////

  public:
    /// \brief Requests and signals that a critical section is being entered.
    void enter();

    /// \brief Signals that a critical section is being left.
    void leave();

    // Private Members ////////////////////////////////////////////////////////

  private:
    // Contains internal data, mainly stuff that is required for cross-platform
    // support.
    struct Impl;
    Impl* _pi;
};

} // namespace ub::xplat