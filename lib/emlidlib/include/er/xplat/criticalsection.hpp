#pragma once

#include "er/util/nocopy.hpp"
#include "export.hpp"

namespace er::xplat
{
/// \brief Represents a cross-platform critical section.
class er_proglib_DLLEXPORT CriticalSection : private util::NoCopy
{
  public: // Constructors ///////////////////////////////////////////////////////////
    /// \brief Creates a new critical section.
    CriticalSection();

    ~CriticalSection();

  public: // Public Methods /////////////////////////////////////////////////////////
    /// \brief Requests and signals that a critical section is being entered.
    void enter();

    /// \brief Signals that a critical section is being left.
    void leave();

  private: // Private Members ////////////////////////////////////////////////////////
    // Contains internal data, mainly stuff that is required for cross-platform
    // support.
    struct Impl;
    Impl* _pi;
};

} // namespace er::xplat