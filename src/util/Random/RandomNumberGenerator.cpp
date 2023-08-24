/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RandomNumberGenerator.cpp
/// @brief Random Number generator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-24

#include "RandomNumberGenerator.hpp"

namespace NAV
{

void to_json(json& j, const RandomNumberGenerator& rng)
{
    j = json{
        { "useSeed", rng.useSeed },
        { "seed", rng.seed },
    };
}
void from_json(const json& j, RandomNumberGenerator& rng)
{
    if (j.contains("useSeed"))
    {
        j.at("useSeed").get_to(rng.useSeed);
    }
    if (j.contains("seed"))
    {
        j.at("seed").get_to(rng.seed);
    }
}

} // namespace NAV