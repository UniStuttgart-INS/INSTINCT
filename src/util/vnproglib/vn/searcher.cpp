// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "vn/searcher.h"

#ifndef HAS_VECTORNAV_LIBRARY

// NOLINTBEGIN

namespace vn
{
namespace sensors
{

bool Searcher::search(const std::string& /* portName */, int32_t* /* foundBaudrate */)
{
    return false;
}

std::vector<std::pair<std::string, uint32_t>> Searcher::search()
{
    return {};
}

std::vector<std::pair<std::string, uint32_t>> Searcher::search(std::vector<std::string>& /* portsToCheck */)
{
    return {};
}

bool Searcher::test(std::string /* portName */, uint32_t /* baudrate */)
{
    return false;
}

} // namespace sensors
} // namespace vn

// NOLINTEND

#endif