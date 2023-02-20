// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Code.hpp"

#include <array>
#include <fmt/core.h>
#include <imgui.h>

#include "util/Logger.hpp"

namespace NAV
{

Code::operator std::string() const
{
    const std::string filler = " | ";
    std::string str;

    if (*this & G1C) { str += (!str.empty() ? filler : "") + "G1C"; }
    if (*this & G1S) { str += (!str.empty() ? filler : "") + "G1S"; }
    if (*this & G1L) { str += (!str.empty() ? filler : "") + "G1L"; }
    if (*this & G1X) { str += (!str.empty() ? filler : "") + "G1X"; }
    if (*this & G1P) { str += (!str.empty() ? filler : "") + "G1P"; }
    if (*this & G1W) { str += (!str.empty() ? filler : "") + "G1W"; }
    if (*this & G1Y) { str += (!str.empty() ? filler : "") + "G1Y"; }
    if (*this & G1M) { str += (!str.empty() ? filler : "") + "G1M"; }
    if (*this & G1N) { str += (!str.empty() ? filler : "") + "G1N"; }
    if (*this & G2C) { str += (!str.empty() ? filler : "") + "G2C"; }
    if (*this & G2D) { str += (!str.empty() ? filler : "") + "G2D"; }
    if (*this & G2S) { str += (!str.empty() ? filler : "") + "G2S"; }
    if (*this & G2L) { str += (!str.empty() ? filler : "") + "G2L"; }
    if (*this & G2X) { str += (!str.empty() ? filler : "") + "G2X"; }
    if (*this & G2P) { str += (!str.empty() ? filler : "") + "G2P"; }
    if (*this & G2W) { str += (!str.empty() ? filler : "") + "G2W"; }
    if (*this & G2Y) { str += (!str.empty() ? filler : "") + "G2Y"; }
    if (*this & G2M) { str += (!str.empty() ? filler : "") + "G2M"; }
    if (*this & G2N) { str += (!str.empty() ? filler : "") + "G2N"; }
    if (*this & G5I) { str += (!str.empty() ? filler : "") + "G5I"; }
    if (*this & G5Q) { str += (!str.empty() ? filler : "") + "G5Q"; }
    if (*this & G5X) { str += (!str.empty() ? filler : "") + "G5X"; }

    if (*this & E1A) { str += (!str.empty() ? filler : "") + "E1A"; }
    if (*this & E1B) { str += (!str.empty() ? filler : "") + "E1B"; }
    if (*this & E1C) { str += (!str.empty() ? filler : "") + "E1C"; }
    if (*this & E1X) { str += (!str.empty() ? filler : "") + "E1X"; }
    if (*this & E1Z) { str += (!str.empty() ? filler : "") + "E1Z"; }
    if (*this & E5I) { str += (!str.empty() ? filler : "") + "E5I"; }
    if (*this & E5Q) { str += (!str.empty() ? filler : "") + "E5Q"; }
    if (*this & E5X) { str += (!str.empty() ? filler : "") + "E5X"; }
    if (*this & E6A) { str += (!str.empty() ? filler : "") + "E6A"; }
    if (*this & E6B) { str += (!str.empty() ? filler : "") + "E6B"; }
    if (*this & E6C) { str += (!str.empty() ? filler : "") + "E6C"; }
    if (*this & E6X) { str += (!str.empty() ? filler : "") + "E6X"; }
    if (*this & E6Z) { str += (!str.empty() ? filler : "") + "E6Z"; }
    if (*this & E7I) { str += (!str.empty() ? filler : "") + "E7I"; }
    if (*this & E7Q) { str += (!str.empty() ? filler : "") + "E7Q"; }
    if (*this & E7X) { str += (!str.empty() ? filler : "") + "E7X"; }
    if (*this & E8I) { str += (!str.empty() ? filler : "") + "E8I"; }
    if (*this & E8Q) { str += (!str.empty() ? filler : "") + "E8Q"; }
    if (*this & E8X) { str += (!str.empty() ? filler : "") + "E8X"; }

    if (*this & R1C) { str += (!str.empty() ? filler : "") + "R1C"; }
    if (*this & R1P) { str += (!str.empty() ? filler : "") + "R1P"; }
    if (*this & R2C) { str += (!str.empty() ? filler : "") + "R2C"; }
    if (*this & R2P) { str += (!str.empty() ? filler : "") + "R2P"; }
    if (*this & R3I) { str += (!str.empty() ? filler : "") + "R3I"; }
    if (*this & R3Q) { str += (!str.empty() ? filler : "") + "R3Q"; }
    if (*this & R3X) { str += (!str.empty() ? filler : "") + "R3X"; }
    if (*this & R4A) { str += (!str.empty() ? filler : "") + "R4A"; }
    if (*this & R4B) { str += (!str.empty() ? filler : "") + "R4B"; }
    if (*this & R4X) { str += (!str.empty() ? filler : "") + "R4X"; }
    if (*this & R6A) { str += (!str.empty() ? filler : "") + "R6A"; }
    if (*this & R6B) { str += (!str.empty() ? filler : "") + "R6B"; }
    if (*this & R6X) { str += (!str.empty() ? filler : "") + "R6X"; }

    if (*this & B1D) { str += (!str.empty() ? filler : "") + "B1D"; }
    if (*this & B1P) { str += (!str.empty() ? filler : "") + "B1P"; }
    if (*this & B1X) { str += (!str.empty() ? filler : "") + "B1X"; }
    if (*this & B2I) { str += (!str.empty() ? filler : "") + "B2I"; }
    if (*this & B2Q) { str += (!str.empty() ? filler : "") + "B2Q"; }
    if (*this & B2X) { str += (!str.empty() ? filler : "") + "B2X"; }
    if (*this & B5D) { str += (!str.empty() ? filler : "") + "B5D"; }
    if (*this & B5P) { str += (!str.empty() ? filler : "") + "B5P"; }
    if (*this & B5X) { str += (!str.empty() ? filler : "") + "B5X"; }
    if (*this & B6I) { str += (!str.empty() ? filler : "") + "B6I"; }
    if (*this & B6Q) { str += (!str.empty() ? filler : "") + "B6Q"; }
    if (*this & B6X) { str += (!str.empty() ? filler : "") + "B6X"; }
    if (*this & B6A) { str += (!str.empty() ? filler : "") + "B6A"; }
    if (*this & B7I) { str += (!str.empty() ? filler : "") + "B7I"; }
    if (*this & B7Q) { str += (!str.empty() ? filler : "") + "B7Q"; }
    if (*this & B7X) { str += (!str.empty() ? filler : "") + "B7X"; }
    if (*this & B7D) { str += (!str.empty() ? filler : "") + "B7D"; }
    if (*this & B7P) { str += (!str.empty() ? filler : "") + "B7P"; }
    if (*this & B7Z) { str += (!str.empty() ? filler : "") + "B7Z"; }
    if (*this & B8D) { str += (!str.empty() ? filler : "") + "B8D"; }
    if (*this & B8P) { str += (!str.empty() ? filler : "") + "B8P"; }
    if (*this & B8X) { str += (!str.empty() ? filler : "") + "B8X"; }

    if (*this & J1C) { str += (!str.empty() ? filler : "") + "J1C"; }
    if (*this & J1S) { str += (!str.empty() ? filler : "") + "J1S"; }
    if (*this & J1L) { str += (!str.empty() ? filler : "") + "J1L"; }
    if (*this & J1X) { str += (!str.empty() ? filler : "") + "J1X"; }
    if (*this & J1Z) { str += (!str.empty() ? filler : "") + "J1Z"; }
    if (*this & J2S) { str += (!str.empty() ? filler : "") + "J2S"; }
    if (*this & J2L) { str += (!str.empty() ? filler : "") + "J2L"; }
    if (*this & J2X) { str += (!str.empty() ? filler : "") + "J2X"; }
    if (*this & J5I) { str += (!str.empty() ? filler : "") + "J5I"; }
    if (*this & J5Q) { str += (!str.empty() ? filler : "") + "J5Q"; }
    if (*this & J5X) { str += (!str.empty() ? filler : "") + "J5X"; }
    if (*this & J5D) { str += (!str.empty() ? filler : "") + "J5D"; }
    if (*this & J5P) { str += (!str.empty() ? filler : "") + "J5P"; }
    if (*this & J5Z) { str += (!str.empty() ? filler : "") + "J5Z"; }
    if (*this & J6S) { str += (!str.empty() ? filler : "") + "J6S"; }
    if (*this & J6L) { str += (!str.empty() ? filler : "") + "J6L"; }
    if (*this & J6X) { str += (!str.empty() ? filler : "") + "J6X"; }
    if (*this & J6E) { str += (!str.empty() ? filler : "") + "J6E"; }
    if (*this & J6Z) { str += (!str.empty() ? filler : "") + "J6Z"; }

    if (*this & I5A) { str += (!str.empty() ? filler : "") + "I5A"; }
    if (*this & I5B) { str += (!str.empty() ? filler : "") + "I5B"; }
    if (*this & I5C) { str += (!str.empty() ? filler : "") + "I5C"; }
    if (*this & I5X) { str += (!str.empty() ? filler : "") + "I5X"; }
    if (*this & I9A) { str += (!str.empty() ? filler : "") + "I9A"; }
    if (*this & I9B) { str += (!str.empty() ? filler : "") + "I9B"; }
    if (*this & I9C) { str += (!str.empty() ? filler : "") + "I9C"; }
    if (*this & I9X) { str += (!str.empty() ? filler : "") + "I9X"; }

    if (*this & S1C) { str += (!str.empty() ? filler : "") + "S1C"; }
    if (*this & S5I) { str += (!str.empty() ? filler : "") + "S5I"; }
    if (*this & S5Q) { str += (!str.empty() ? filler : "") + "S5Q"; }
    if (*this & S5X) { str += (!str.empty() ? filler : "") + "S5X"; }

    if (!str.empty())
    {
        return str;
    }
    return "None";
}

Code Code::fromFreqAttr(Frequency freq, char attribute)
{
    switch (Frequency_(freq))
    {
    case G01: // GPS L1 (1575.42 MHz).
        if (attribute == 'C') { return G1C; }
        if (attribute == 'S') { return G1S; }
        if (attribute == 'L') { return G1L; }
        if (attribute == 'X') { return G1X; }
        if (attribute == 'P') { return G1P; }
        if (attribute == 'W') { return G1W; }
        if (attribute == 'Y') { return G1Y; }
        if (attribute == 'M') { return G1M; }
        if (attribute == 'N') { return G1N; }
        break;
    case G02: // GPS L2 (1227.6 MHz).
        if (attribute == 'C') { return G2C; }
        if (attribute == 'D') { return G2D; }
        if (attribute == 'S') { return G2S; }
        if (attribute == 'L') { return G2L; }
        if (attribute == 'X') { return G2X; }
        if (attribute == 'P') { return G2P; }
        if (attribute == 'W') { return G2W; }
        if (attribute == 'Y') { return G2Y; }
        if (attribute == 'M') { return G2M; }
        if (attribute == 'N') { return G2N; }
        break;
    case G05: // GPS L5 (1176.45 MHz).
        if (attribute == 'I') { return G5I; }
        if (attribute == 'Q') { return G5Q; }
        if (attribute == 'X') { return G5X; }
        break;
    case E01: // Galileo, "E1" (1575.42 MHz).
        if (attribute == 'A') { return E1A; }
        if (attribute == 'B') { return E1B; }
        if (attribute == 'C') { return E1C; }
        if (attribute == 'X') { return E1X; }
        if (attribute == 'Z') { return E1Z; }
        break;
    case E05: // Galileo E5a (1176.45 MHz).
        if (attribute == 'I') { return E5I; }
        if (attribute == 'Q') { return E5Q; }
        if (attribute == 'X') { return E5X; }
        break;
    case E06: // Galileo E6 (1278.75 MHz).
        if (attribute == 'A') { return E6A; }
        if (attribute == 'B') { return E6B; }
        if (attribute == 'C') { return E6C; }
        if (attribute == 'X') { return E6X; }
        if (attribute == 'Z') { return E6Z; }
        break;
    case E07: // Galileo E5b (1207.14 MHz).
        if (attribute == 'I') { return E7I; }
        if (attribute == 'Q') { return E7Q; }
        if (attribute == 'X') { return E7X; }
        break;
    case E08: // Galileo E5 (E5a + E5b) (1191.795MHz).
        if (attribute == 'I') { return E8I; }
        if (attribute == 'Q') { return E8Q; }
        if (attribute == 'X') { return E8X; }
        break;

    case R01: // GLONASS, "G1" (1602 MHZ).
        if (attribute == 'C') { return R1C; }
        if (attribute == 'P') { return R1P; }
        break;
    case R02: // GLONASS, "G2" (1246 MHz).
        if (attribute == 'C') { return R2C; }
        if (attribute == 'P') { return R2P; }
        break;
    case R03: // GLONASS, "G3" (1202.025 MHz).
        if (attribute == 'I') { return R3I; }
        if (attribute == 'Q') { return R3Q; }
        if (attribute == 'X') { return R3X; }
        break;
    case R04: // GLONASS, "G1a" (1600.995 MHZ).
        if (attribute == 'A') { return R4A; }
        if (attribute == 'B') { return R4B; }
        if (attribute == 'X') { return R4X; }
        break;
    case R06: // GLONASS, "G2a" (1248.06 MHz).
        if (attribute == 'A') { return R6A; }
        if (attribute == 'B') { return R6B; }
        if (attribute == 'X') { return R6X; }
        break;

    case B01: // Beidou B1 (1575.42 MHz).
        if (attribute == 'D') { return B1D; }
        if (attribute == 'P') { return B1P; }
        if (attribute == 'X') { return B1X; }
        break;
    case B02: // Beidou B1-2 (1561.098 MHz).
        if (attribute == 'I') { return B2I; }
        if (attribute == 'Q') { return B2Q; }
        if (attribute == 'X') { return B2X; }
        break;
    case B05: // Beidou B2a (1176.45 MHz).
        if (attribute == 'D') { return B5D; }
        if (attribute == 'P') { return B5P; }
        if (attribute == 'X') { return B5X; }
        break;
    case B06: // Beidou B3 (1268.52 MHz).
        if (attribute == 'I') { return B6I; }
        if (attribute == 'Q') { return B6Q; }
        if (attribute == 'X') { return B6X; }
        if (attribute == 'A') { return B6A; }
        break;
    case B07: // Beidou B2b (1207.14 MHz).
        if (attribute == 'I') { return B7I; }
        if (attribute == 'Q') { return B7Q; }
        if (attribute == 'X') { return B7X; }
        if (attribute == 'D') { return B7D; }
        if (attribute == 'P') { return B7P; }
        if (attribute == 'Z') { return B7Z; }
        break;
    case B08: // Beidou B2 (B2a + B2b) (1191.795MHz).
        if (attribute == 'D') { return B8D; }
        if (attribute == 'P') { return B8P; }
        if (attribute == 'X') { return B8X; }
        break;

    case J01: // QZSS L1 (1575.42 MHz).
        if (attribute == 'C') { return J1C; }
        if (attribute == 'S') { return J1S; }
        if (attribute == 'L') { return J1L; }
        if (attribute == 'X') { return J1X; }
        if (attribute == 'Z') { return J1Z; }
        break;
    case J02: // QZSS L2 (1227.6 MHz).
        if (attribute == 'S') { return J2S; }
        if (attribute == 'L') { return J2L; }
        if (attribute == 'X') { return J2X; }
        break;
    case J05: // QZSS L5 (1176.45 MHz).
        if (attribute == 'I') { return J5I; }
        if (attribute == 'Q') { return J5Q; }
        if (attribute == 'X') { return J5X; }
        if (attribute == 'D') { return J5D; }
        if (attribute == 'P') { return J5P; }
        if (attribute == 'Z') { return J5Z; }
        break;
    case J06: // QZSS L6 / LEX (1278.75 MHz).
        if (attribute == 'S') { return J6S; }
        if (attribute == 'L') { return J6L; }
        if (attribute == 'X') { return J6X; }
        if (attribute == 'E') { return J6E; }
        if (attribute == 'Z') { return J6Z; }
        break;

    case I05: // IRNSS L5 (1176.45 MHz).
        if (attribute == 'A') { return I5A; }
        if (attribute == 'B') { return I5B; }
        if (attribute == 'C') { return I5C; }
        if (attribute == 'X') { return I5X; }
        break;
    case I09: // IRNSS S (2492.028 MHz).
        if (attribute == 'A') { return I9A; }
        if (attribute == 'B') { return I9B; }
        if (attribute == 'C') { return I9C; }
        if (attribute == 'X') { return I9X; }
        break;

    case S01: // SBAS L1 (1575.42 MHz).
        if (attribute == 'C') { return S1C; }
        break;
    case S05: // SBAS L5 (1176.45 MHz).
        if (attribute == 'I') { return S5I; }
        if (attribute == 'Q') { return S5Q; }
        if (attribute == 'X') { return S5X; }
        break;

    case Freq_None:
        break;
    }

    LOG_WARN("Can't convert frequency '{}' and attribute '{}'. Unkown code.", freq, attribute);

    return Code(Set());
}

const char* Code::GetCodeDescription(Code code)
{
    if (code == G1C) { return "GPS L1 - C/A-code"; }
    if (code == G1S) { return "GPS L1 - L1C-D (data)"; }
    if (code == G1L) { return "GPS L1 - L1C-P (pilot)"; }
    if (code == G1X) { return "GPS L1 - L1C-(D+P) (combined)"; }
    if (code == G1P) { return "GPS L1 - P-code (unencrypted)"; }
    if (code == G1W) { return "GPS L1 - Semicodeless P(Y) tracking (Z-tracking)"; }
    if (code == G1Y) { return "GPS L1 - Y-code (with decryption)"; }
    if (code == G1M) { return "GPS L1 - M-code"; }
    if (code == G1N) { return "GPS L1 - codeless"; }
    if (code == G2C) { return "GPS L2 - C/A-code"; }
    if (code == G2D) { return "GPS L2 - Semi-codeless P(Y) tracking (L1 C/A + (P2-P1))"; }
    if (code == G2S) { return "GPS L2 - L2C(M) (medium)"; }
    if (code == G2L) { return "GPS L2 - L2C(L) (long)"; }
    if (code == G2X) { return "GPS L2 - L2C(M+L) (combined)"; }
    if (code == G2P) { return "GPS L2 - P-code (unencrypted)"; }
    if (code == G2W) { return "GPS L2 - Semicodeless P(Y) tracking (Z-tracking)"; }
    if (code == G2Y) { return "GPS L2 - Y-code (with decryption)"; }
    if (code == G2M) { return "GPS L2 - M-code"; }
    if (code == G2N) { return "GPS L2 - codeless"; }
    if (code == G5I) { return "GPS L5 - Data"; }
    if (code == G5Q) { return "GPS L5 - Pilot"; }
    if (code == G5X) { return "GPS L5 - Combined"; }

    if (code == E1A) { return "GAL E1 - PRS signal"; }
    if (code == E1B) { return "GAL E1 - OS (data)"; }
    if (code == E1C) { return "GAL E1 - OS (pilot)"; }
    if (code == E1X) { return "GAL E1 - OS(B+C) (combined)"; }
    if (code == E1Z) { return "GAL E1 - PRS + OS (data + pilot)"; }
    if (code == E5I) { return "GAL E5a - Data"; }
    if (code == E5Q) { return "GAL E5a - Pilot"; }
    if (code == E5X) { return "GAL E5a - Combined"; }
    if (code == E6A) { return "GAL E6 - PRS signal"; }
    if (code == E6B) { return "GAL E6 - Data"; }
    if (code == E6C) { return "GAL E6 - Pilot"; }
    if (code == E6X) { return "GAL E6 - Combined (B+C)"; }
    if (code == E6Z) { return "GAL E6 - PRS + OS (A+B+C)"; }
    if (code == E7I) { return "GAL E5b - Data"; }
    if (code == E7Q) { return "GAL E5b - Pilot"; }
    if (code == E7X) { return "GAL E5b - Combined"; }
    if (code == E8I) { return "GAL E5(a+b) - AltBOC (data)"; }
    if (code == E8Q) { return "GAL E5(a+b) - AltBOC (pilot)"; }
    if (code == E8X) { return "GAL E5(a+b) - AltBOC (combined)"; }

    if (code == R1C) { return "GLO L1 - C/A-code"; }
    if (code == R1P) { return "GLO L1 - P-code"; }
    if (code == R2C) { return "GLO L2 - C/A-code"; }
    if (code == R2P) { return "GLO L2 - P-code"; }
    if (code == R3I) { return "GLO L3 - Data"; }
    if (code == R3Q) { return "GLO L3 - Pilot"; }
    if (code == R3X) { return "GLO L3 - Combined"; }
    if (code == R4A) { return "GLO G1a - L1OCd (data)"; }
    if (code == R4B) { return "GLO G1a - L1OCp (pilot)"; }
    if (code == R4X) { return "GLO G1a - L1OCd+L1OCp (combined)"; }
    if (code == R6A) { return "GLO G2a - L2CSI (data)"; }
    if (code == R6B) { return "GLO G2a - L2OCp (pilot)"; }
    if (code == R6X) { return "GLO G2a - L2CSI+L2OCp (combined)"; }

    if (code == B1D) { return "BeiDou B1 - Data (D)"; }
    if (code == B1P) { return "BeiDou B1 - Pilot(P)"; }
    if (code == B1X) { return "BeiDou B1 - D+P"; }
    if (code == B2I) { return "BeiDou B1-2 - B1I(OS)"; }
    if (code == B2Q) { return "BeiDou B1-2 - B1Q"; }
    if (code == B2X) { return "BeiDou B1-2 - B1I(OS), B1Q, combined"; }
    if (code == B5D) { return "BeiDou B2a - Data (D)"; }
    if (code == B5P) { return "BeiDou B2a - Pilot(P)"; }
    if (code == B5X) { return "BeiDou B2a - D+P"; }
    if (code == B6I) { return "BeiDou B3 - B3I"; }
    if (code == B6Q) { return "BeiDou B3 - B3Q"; }
    if (code == B6X) { return "BeiDou B3 - B3I, B3Q, combined"; }
    if (code == B6A) { return "BeiDou B3 - B3A"; }
    if (code == B7I) { return "BeiDou B2b (BDS-2) - B2I(OS)"; }
    if (code == B7Q) { return "BeiDou B2b (BDS-2) - B2Q"; }
    if (code == B7X) { return "BeiDou B2b (BDS-2) - B2I(OS), B2Q, combined"; }
    if (code == B7D) { return "BeiDou B2b (BDS-3) - Data (D)"; }
    if (code == B7P) { return "BeiDou B2b (BDS-3) - Pilot(P)"; }
    if (code == B7Z) { return "BeiDou B2b (BDS-3) - D+P"; }
    if (code == B8D) { return "BeiDou B2 (B2a+B2b) - Data (D)"; }
    if (code == B8P) { return "BeiDou B2 (B2a+B2b) - Pilot(P)"; }
    if (code == B8X) { return "BeiDou B2 (B2a+B2b) - D+P"; }

    if (code == J1C) { return "QZSS L1 - C/A-code"; }
    if (code == J1S) { return "QZSS L1 - L1C (data)"; }
    if (code == J1L) { return "QZSS L1 - L1C (pilot)"; }
    if (code == J1X) { return "QZSS L1 - L1C (combined)"; }
    if (code == J1Z) { return "QZSS L1 - L1-SAIF signal"; }
    if (code == J2S) { return "QZSS L2 - L2C-code (medium)"; }
    if (code == J2L) { return "QZSS L2 - L2C-code (long)"; }
    if (code == J2X) { return "QZSS L2 - L2C-code (combined)"; }
    if (code == J5I) { return "QZSS L5 - Data"; }
    if (code == J5Q) { return "QZSS L5 - Pilot"; }
    if (code == J5X) { return "QZSS L5 - Combined"; }
    if (code == J5D) { return "QZSS L5S - I"; }
    if (code == J5P) { return "QZSS L5S - Q"; }
    if (code == J5Z) { return "QZSS L5S - I+Q"; }
    if (code == J6S) { return "QZSS L6 - L6D LEX signal (short)"; }
    if (code == J6L) { return "QZSS L6 - L6P LEX signal (long)"; }
    if (code == J6X) { return "QZSS L6 - L6(D+P) LEX signal (combined)"; }
    if (code == J6E) { return "QZSS L6 - L6E"; }
    if (code == J6Z) { return "QZSS L6 - L6(D+E)"; }

    if (code == I5A) { return "IRNSS L5 - SPS Signal"; }
    if (code == I5B) { return "IRNSS L5 - RS (data)"; }
    if (code == I5C) { return "IRNSS L5 - RS (pilot)"; }
    if (code == I5X) { return "IRNSS L5 - RS (combined)"; }
    if (code == I9A) { return "IRNSS S - SPS signal"; }
    if (code == I9B) { return "IRNSS S - RS (data)"; }
    if (code == I9C) { return "IRNSS S - RS (pilot)"; }
    if (code == I9X) { return "IRNSS S - RS (combined)"; }

    if (code == S1C) { return "SBAS L1 - C/A-code"; }
    if (code == S5I) { return "SBAS L5 - Data"; }
    if (code == S5Q) { return "SBAS L5 - Pilot"; }
    if (code == S5X) { return "SBAS L5 - Combined"; }

    return "Unknown code.";
}

const char* Code::getDescription() const
{
    return GetCodeDescription(*this);
}

Frequency Code::GetCodeFequency(Code code)
{
    if (code == G1C) { return G01; } // GPS L1 - C/A-code
    if (code == G1S) { return G01; } // GPS L1 - L1C-D (data)
    if (code == G1L) { return G01; } // GPS L1 - L1C-P (pilot)
    if (code == G1X) { return G01; } // GPS L1 - L1C-(D+P) (combined)
    if (code == G1P) { return G01; } // GPS L1 - P-code (unencrypted)
    if (code == G1W) { return G01; } // GPS L1 - Semicodeless P(Y) tracking (Z-tracking)
    if (code == G1Y) { return G01; } // GPS L1 - Y-code (with decryption)
    if (code == G1M) { return G01; } // GPS L1 - M-code
    if (code == G1N) { return G01; } // GPS L1 - codeless
    if (code == G2C) { return G02; } // GPS L2 - C/A-code
    if (code == G2D) { return G02; } // GPS L2 - Semi-codeless P(Y) tracking (L1 C/A + (P2-P1))
    if (code == G2S) { return G02; } // GPS L2 - L2C(M) (medium)
    if (code == G2L) { return G02; } // GPS L2 - L2C(L) (long)
    if (code == G2X) { return G02; } // GPS L2 - L2C(M+L) (combined)
    if (code == G2P) { return G02; } // GPS L2 - P-code (unencrypted)
    if (code == G2W) { return G02; } // GPS L2 - Semicodeless P(Y) tracking (Z-tracking)
    if (code == G2Y) { return G02; } // GPS L2 - Y-code (with decryption)
    if (code == G2M) { return G02; } // GPS L2 - M-code
    if (code == G2N) { return G02; } // GPS L2 - codeless
    if (code == G5I) { return G05; } // GPS L5 - Data
    if (code == G5Q) { return G05; } // GPS L5 - Pilot
    if (code == G5X) { return G05; } // GPS L5 - Combined

    if (code == E1A) { return E01; } // GAL E1 - PRS signal
    if (code == E1B) { return E01; } // GAL E1 - OS (data)
    if (code == E1C) { return E01; } // GAL E1 - OS (pilot)
    if (code == E1X) { return E01; } // GAL E1 - OS(B+C) (combined)
    if (code == E1Z) { return E01; } // GAL E1 - PRS + OS (data + pilot)
    if (code == E5I) { return E05; } // GAL E5a - Data
    if (code == E5Q) { return E05; } // GAL E5a - Pilot
    if (code == E5X) { return E05; } // GAL E5a - Combined
    if (code == E6A) { return E06; } // GAL E6 - PRS signal
    if (code == E6B) { return E06; } // GAL E6 - Data
    if (code == E6C) { return E06; } // GAL E6 - Pilot
    if (code == E6X) { return E06; } // GAL E6 - Combined (B+C)
    if (code == E6Z) { return E06; } // GAL E6 - PRS + OS (A+B+C)
    if (code == E7I) { return E07; } // GAL E5b - Data
    if (code == E7Q) { return E07; } // GAL E5b - Pilot
    if (code == E7X) { return E07; } // GAL E5b - Combined
    if (code == E8I) { return E08; } // GAL E5(a+b) - AltBOC (data)
    if (code == E8Q) { return E08; } // GAL E5(a+b) - AltBOC (pilot)
    if (code == E8X) { return E08; } // GAL E5(a+b) - AltBOC (combined)

    if (code == R1C) { return R01; } // GLO L1 - C/A-code
    if (code == R1P) { return R01; } // GLO L1 - P-code
    if (code == R2C) { return R02; } // GLO L2 - C/A-code
    if (code == R2P) { return R02; } // GLO L2 - P-code
    if (code == R3I) { return R03; } // GLO L3 - Data
    if (code == R3Q) { return R03; } // GLO L3 - Pilot
    if (code == R3X) { return R03; } // GLO L3 - Combined
    if (code == R4A) { return R04; } // GLO G1a - L1OCd (data)
    if (code == R4B) { return R04; } // GLO G1a - L1OCp (pilot)
    if (code == R4X) { return R04; } // GLO G1a - L1OCd+L1OCp (combined)
    if (code == R6A) { return R06; } // GLO G2a - L2CSI (data)
    if (code == R6B) { return R06; } // GLO G2a - L2OCp (pilot)
    if (code == R6X) { return R06; } // GLO G2a - L2CSI+L2OCp (combined)

    if (code == B1D) { return B01; } // BeiDou B1 - Data (D)
    if (code == B1P) { return B01; } // BeiDou B1 - Pilot(P)
    if (code == B1X) { return B01; } // BeiDou B1 - D+P
    if (code == B2I) { return B02; } // BeiDou B1-2 - B1I(OS)
    if (code == B2Q) { return B02; } // BeiDou B1-2 - B1Q
    if (code == B2X) { return B02; } // BeiDou B1-2 - B1I(OS), B1Q, combined
    if (code == B5D) { return B05; } // BeiDou B2a - Data (D)
    if (code == B5P) { return B05; } // BeiDou B2a - Pilot(P)
    if (code == B5X) { return B05; } // BeiDou B2a - D+P
    if (code == B6I) { return B06; } // BeiDou B3 - B3I
    if (code == B6Q) { return B06; } // BeiDou B3 - B3Q
    if (code == B6X) { return B06; } // BeiDou B3 - B3I, B3Q, combined
    if (code == B6A) { return B06; } // BeiDou B3 - B3A
    if (code == B7I) { return B07; } // BeiDou B2b (BDS-2) - B2I(OS)
    if (code == B7Q) { return B07; } // BeiDou B2b (BDS-2) - B2Q
    if (code == B7X) { return B07; } // BeiDou B2b (BDS-2) - B2I(OS), B2Q, combined
    if (code == B7D) { return B07; } // BeiDou B2b (BDS-3) - Data (D)
    if (code == B7P) { return B07; } // BeiDou B2b (BDS-3) - Pilot(P)
    if (code == B7Z) { return B07; } // BeiDou B2b (BDS-3) - D+P
    if (code == B8D) { return B08; } // BeiDou B2 (B2a+B2b) - Data (D)
    if (code == B8P) { return B08; } // BeiDou B2 (B2a+B2b) - Pilot(P)
    if (code == B8X) { return B08; } // BeiDou B2 (B2a+B2b) - D+P

    if (code == J1C) { return J01; } // QZSS L1 - C/A-code
    if (code == J1S) { return J01; } // QZSS L1 - L1C (data)
    if (code == J1L) { return J01; } // QZSS L1 - L1C (pilot)
    if (code == J1X) { return J01; } // QZSS L1 - L1C (combined)
    if (code == J1Z) { return J01; } // QZSS L1 - L1-SAIF signal
    if (code == J2S) { return J02; } // QZSS L2 - L2C-code (medium)
    if (code == J2L) { return J02; } // QZSS L2 - L2C-code (long)
    if (code == J2X) { return J02; } // QZSS L2 - L2C-code (combined)
    if (code == J5I) { return J05; } // QZSS L5 - Data
    if (code == J5Q) { return J05; } // QZSS L5 - Pilot
    if (code == J5X) { return J05; } // QZSS L5 - Combined
    if (code == J5D) { return J05; } // QZSS L5S - I
    if (code == J5P) { return J05; } // QZSS L5S - Q
    if (code == J5Z) { return J05; } // QZSS L5S - I+Q
    if (code == J6S) { return J06; } // QZSS L6 - L6D LEX signal (short)
    if (code == J6L) { return J06; } // QZSS L6 - L6P LEX signal (long)
    if (code == J6X) { return J06; } // QZSS L6 - L6(D+P) LEX signal (combined)
    if (code == J6E) { return J06; } // QZSS L6 - L6E
    if (code == J6Z) { return J06; } // QZSS L6 - L6(D+E)

    if (code == I5A) { return I05; } // IRNSS L5 - SPS Signal
    if (code == I5B) { return I05; } // IRNSS L5 - RS (data)
    if (code == I5C) { return I05; } // IRNSS L5 - RS (pilot)
    if (code == I5X) { return I05; } // IRNSS L5 - RS (combined)
    if (code == I9A) { return I09; } // IRNSS S - SPS signal
    if (code == I9B) { return I09; } // IRNSS S - RS (data)
    if (code == I9C) { return I09; } // IRNSS S - RS (pilot)
    if (code == I9X) { return I09; } // IRNSS S - RS (combined)

    if (code == S1C) { return S01; } // SBAS L1 - C/A-code
    if (code == S5I) { return S05; } // SBAS L5 - Data
    if (code == S5Q) { return S05; } // SBAS L5 - Pilot
    if (code == S5X) { return S05; } // SBAS L5 - Combined

    return Freq_None;
}

Frequency Code::getFrequency() const
{
    return GetCodeFequency(*this);
}

bool Code::IsCodeCombined(Code first, Code second)
{
    return ((first & Code_G1S_G1L_G1X) && (second & Code_G1S_G1L_G1X))
           || ((first & Code_G2S_G2L_G2X) && (second & Code_G2S_G2L_G2X))
           || ((first & Code_G5I_G5Q_G5X) && (second & Code_G5I_G5Q_G5X))
           || ((first & Code_E1B_E1C_E1X) && (second & Code_E1B_E1C_E1X))
           || ((first & Code_E5I_E5Q_E5X) && (second & Code_E5I_E5Q_E5X))
           || ((first & Code_E6B_E6C_E6X) && (second & Code_E6B_E6C_E6X))
           || ((first & Code_E7I_E7Q_E7X) && (second & Code_E7I_E7Q_E7X))
           || ((first & Code_E8I_E8Q_E8X) && (second & Code_E8I_E8Q_E8X))
           || ((first & Code_R3I_R3Q_R3X) && (second & Code_R3I_R3Q_R3X))
           || ((first & Code_R4A_R4B_R4X) && (second & Code_R4A_R4B_R4X))
           || ((first & Code_R6A_R6B_R6X) && (second & Code_R6A_R6B_R6X))
           || ((first & Code_B1D_B1P_B1X) && (second & Code_B1D_B1P_B1X))
           || ((first & Code_B2I_B2Q_B2X) && (second & Code_B2I_B2Q_B2X))
           || ((first & Code_B5D_B5P_B5X) && (second & Code_B5D_B5P_B5X))
           || ((first & Code_B6I_B6Q_B6X) && (second & Code_B6I_B6Q_B6X))
           || ((first & Code_B7I_B7Q_B7X) && (second & Code_B7I_B7Q_B7X))
           || ((first & Code_B7D_B7P_B7Z) && (second & Code_B7D_B7P_B7Z))
           || ((first & Code_B8D_B8P_B8X) && (second & Code_B8D_B8P_B8X))
           || ((first & Code_J1S_J1L_J1X) && (second & Code_J1S_J1L_J1X))
           || ((first & Code_J2S_J2L_J2X) && (second & Code_J2S_J2L_J2X))
           || ((first & Code_J5I_J5Q_J5X) && (second & Code_J5I_J5Q_J5X))
           || ((first & Code_J5D_J5P_J5Z) && (second & Code_J5D_J5P_J5Z))
           || ((first & Code_J6S_J6L_J6X) && (second & Code_J6S_J6L_J6X))
           || ((first & Code_I5B_I5C_I5X) && (second & Code_I5B_I5C_I5X))
           || ((first & Code_I9B_I9C_I9X) && (second & Code_I9B_I9C_I9X))
           || ((first & Code_S5I_S5Q_S5X) && (second & Code_S5I_S5Q_S5X));
}

// #########################################################################################################################################

Code operator|(const Code& lhs, const Code& rhs)
{
    return Code(lhs.value | rhs.value);
}
Code operator|(const Code& lhs, const Code::Enum& rhs)
{
    return Code(lhs.value | Code(rhs).value);
}
Code operator|(const Code::Enum& lhs, const Code& rhs)
{
    return Code(Code::Set().set(lhs, true) | rhs.value);
}

Code& Code::operator|=(const Code& rhs)
{
    *this = *this | rhs;
    return *this;
}
Code& Code::operator|=(const Enum& rhs)
{
    *this = *this | rhs;
    return *this;
}

Code operator&(const Code& lhs, const Code& rhs)
{
    return Code(lhs.value & rhs.value);
}
Code operator&(const Code& lhs, const Code::Enum& rhs)
{
    return Code(lhs.value & Code::Set().set(rhs, true));
}
Code operator&(const Code::Enum& lhs, const Code& rhs)
{
    return Code(Code::Set().set(lhs, true) & rhs.value);
}

Code operator|(const Code::Enum& lhs, const Code::Enum& rhs)
{
    return Code(lhs) | Code(rhs);
}
Code operator&(const Code::Enum& lhs, const Code::Enum& rhs)
{
    return Code(lhs) & Code(rhs);
}

Code& Code::operator&=(const Code& rhs)
{
    *this = *this & rhs;
    return *this;
}
Code& Code::operator&=(const Enum& rhs)
{
    *this = *this & rhs;
    return *this;
}

Code Code::operator~() const
{
    return Code(~value);
}

// #########################################################################################################################################

bool operator==(const Code& lhs, const Code& rhs)
{
    return lhs.value == rhs.value;
}

bool operator==(const Code& lhs, const Code::Enum& rhs)
{
    return lhs.value == Code(rhs).value;
}

bool operator==(const Code::Enum& lhs, const Code& rhs)
{
    return Code(lhs).value == rhs.value;
}

bool operator!=(const Code& lhs, const Code& rhs)
{
    return !(lhs == rhs);
}

bool operator!=(const Code& lhs, const Code::Enum& rhs)
{
    return !(lhs == rhs);
}

bool operator!=(const Code::Enum& lhs, const Code& rhs)
{
    return !(lhs == rhs);
}

// #########################################################################################################################################

Code operator&(Code lhs, SatelliteSystem_ rhs)
{
    if (rhs == SatSys_None)
    {
        return Code(Code::Set());
    }
    if (rhs & GPS)
    {
        for (int i = Code::_GPS_START; i <= Code::_GPS_END; i++) { lhs &= Code(Code::Enum(i)); }
    }
    if (rhs & GAL)
    {
        for (int i = Code::_GAL_START; i <= Code::_GAL_END; i++) { lhs &= Code(Code::Enum(i)); }
    }
    if (rhs & GLO)
    {
        for (int i = Code::_GLO_START; i <= Code::_GLO_END; i++) { lhs &= Code(Code::Enum(i)); }
    }
    if (rhs & BDS)
    {
        for (int i = Code::_BDS_START; i <= Code::_BDS_END; i++) { lhs &= Code(Code::Enum(i)); }
    }
    if (rhs & QZSS)
    {
        for (int i = Code::_QZSS_START; i <= Code::_QZSS_END; i++) { lhs &= Code(Code::Enum(i)); }
    }
    if (rhs & IRNSS)
    {
        for (int i = Code::_IRNSS_START; i <= Code::_IRNSS_END; i++) { lhs &= Code(Code::Enum(i)); }
    }
    if (rhs & SBAS)
    {
        for (int i = Code::_SBAS_START; i <= Code::_SBAS_END; i++) { lhs &= Code(Code::Enum(i)); }
    }

    return lhs;
}
Code operator&(SatelliteSystem_ lhs, Code rhs)
{
    return rhs & lhs;
}
Code& Code::operator&=(const SatelliteSystem_& rhs)
{
    *this = *this & rhs;
    return *this;
}
Code operator&(Code lhs, SatelliteSystem rhs)
{
    return lhs & SatelliteSystem_(rhs);
}
Code operator&(SatelliteSystem lhs, Code rhs)
{
    return SatelliteSystem_(lhs) & rhs;
}
Code& Code::operator&=(const SatelliteSystem& rhs)
{
    *this = *this & rhs;
    return *this;
}

Code operator|(Code lhs, SatelliteSystem_ rhs)
{
    if (rhs == SatSys_None)
    {
        return lhs;
    }
    if (rhs & GPS)
    {
        for (int i = Code::_GPS_START; i <= Code::_GPS_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & GAL)
    {
        for (int i = Code::_GAL_START; i <= Code::_GAL_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & GLO)
    {
        for (int i = Code::_GLO_START; i <= Code::_GLO_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & BDS)
    {
        for (int i = Code::_BDS_START; i <= Code::_BDS_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & QZSS)
    {
        for (int i = Code::_QZSS_START; i <= Code::_QZSS_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & IRNSS)
    {
        for (int i = Code::_IRNSS_START; i <= Code::_IRNSS_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & SBAS)
    {
        for (int i = Code::_SBAS_START; i <= Code::_SBAS_END; i++) { lhs |= Code(Code::Enum(i)); }
    }

    return lhs;
}
Code operator|(SatelliteSystem_ lhs, Code rhs)
{
    return rhs | lhs;
}
Code& Code::operator|=(const SatelliteSystem_& rhs)
{
    *this = *this | rhs;
    return *this;
}
Code operator|(Code lhs, SatelliteSystem rhs)
{
    return lhs | SatelliteSystem_(rhs);
}
Code operator|(SatelliteSystem lhs, Code rhs)
{
    return SatelliteSystem_(lhs) | rhs;
}
Code& Code::operator|=(const SatelliteSystem& rhs)
{
    *this = *this | rhs;
    return *this;
}

// #########################################################################################################################################

Code operator&(Code lhs, Frequency_ rhs)
{
    LOG_DATA("Before ({}) & ({})", Frequency(rhs), lhs);
    if (rhs == Freq_None)
    {
        return Code(Code::Set());
    }
    if (!(rhs & Frequency_::G01))
    {
        for (int i = Code::_G01_START; i <= Code::_G01_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::G02))
    {
        for (int i = Code::_G02_START; i <= Code::_G02_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::G05))
    {
        for (int i = Code::_G05_START; i <= Code::_G05_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::E01))
    {
        for (int i = Code::_E01_START; i <= Code::_E01_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::E05))
    {
        for (int i = Code::_E05_START; i <= Code::_E05_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::E06))
    {
        for (int i = Code::_E06_START; i <= Code::_E06_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::E07))
    {
        for (int i = Code::_E07_START; i <= Code::_E07_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::E08))
    {
        for (int i = Code::_E08_START; i <= Code::_E08_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!((rhs & Frequency_::R01) || (rhs & Frequency_::R04)))
    {
        for (int i = Code::_R01_START; i <= Code::_R01_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!((rhs & Frequency_::R02) || (rhs & Frequency_::R06)))
    {
        for (int i = Code::_R02_START; i <= Code::_R02_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::R03))
    {
        for (int i = Code::_R03_START; i <= Code::_R03_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::R04))
    {
        for (int i = Code::_R04_START; i <= Code::_R04_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::R06))
    {
        for (int i = Code::_R06_START; i <= Code::_R06_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::B01))
    {
        for (int i = Code::_B01_START; i <= Code::_B01_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::B02))
    {
        for (int i = Code::_B02_START; i <= Code::_B02_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::B05))
    {
        for (int i = Code::_B05_START; i <= Code::_B05_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::B06))
    {
        for (int i = Code::_B06_START; i <= Code::_B06_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::B07))
    {
        for (int i = Code::_B07_START; i <= Code::_B07_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::B08))
    {
        for (int i = Code::_B08_START; i <= Code::_B08_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::J01))
    {
        for (int i = Code::_J01_START; i <= Code::_J01_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::J02))
    {
        for (int i = Code::_J02_START; i <= Code::_J02_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::J05))
    {
        for (int i = Code::_J05_START; i <= Code::_J05_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::J06))
    {
        for (int i = Code::_J06_START; i <= Code::_J06_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::I05))
    {
        for (int i = Code::_I05_START; i <= Code::_I05_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::I09))
    {
        for (int i = Code::_I09_START; i <= Code::_I09_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::S01))
    {
        for (int i = Code::_S01_START; i <= Code::_S01_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    if (!(rhs & Frequency_::S05))
    {
        for (int i = Code::_S05_START; i <= Code::_S05_END; i++) { lhs &= ~Code(Code::Enum(i)); }
    }
    LOG_DATA("After  ({}) & ({})", Frequency(rhs), lhs);

    return lhs;
}
Code operator&(Frequency_ lhs, Code rhs)
{
    return rhs & lhs;
}
Code& Code::operator&=(const Frequency_& rhs)
{
    *this = *this & rhs;
    return *this;
}
Code operator&(Code lhs, Frequency rhs)
{
    return lhs & Frequency_(rhs);
}
Code operator&(Frequency lhs, Code rhs)
{
    return Frequency_(lhs) & rhs;
}
Code& Code::operator&=(const Frequency& rhs)
{
    *this = *this & rhs;
    return *this;
}

Code operator|(Code lhs, Frequency_ rhs)
{
    LOG_DATA("Before ({}) & ({})", Frequency(rhs), lhs);
    if (rhs == Freq_None)
    {
        return lhs;
    }
    if (rhs & Frequency_::G01)
    {
        for (int i = Code::_G01_START; i <= Code::_G01_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::G02)
    {
        for (int i = Code::_G02_START; i <= Code::_G02_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::G05)
    {
        for (int i = Code::_G05_START; i <= Code::_G05_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::E01)
    {
        for (int i = Code::_E01_START; i <= Code::_E01_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::E05)
    {
        for (int i = Code::_E05_START; i <= Code::_E05_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::E06)
    {
        for (int i = Code::_E06_START; i <= Code::_E06_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::E07)
    {
        for (int i = Code::_E07_START; i <= Code::_E07_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::E08)
    {
        for (int i = Code::_E08_START; i <= Code::_E08_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if ((rhs & Frequency_::R01) || (rhs & Frequency_::R04))
    {
        for (int i = Code::_R01_START; i <= Code::_R01_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if ((rhs & Frequency_::R02) || (rhs & Frequency_::R06))
    {
        for (int i = Code::_R02_START; i <= Code::_R02_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::R03)
    {
        for (int i = Code::_R03_START; i <= Code::_R03_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::R04)
    {
        for (int i = Code::_R04_START; i <= Code::_R04_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::R06)
    {
        for (int i = Code::_R06_START; i <= Code::_R06_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::B01)
    {
        for (int i = Code::_B01_START; i <= Code::_B01_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::B02)
    {
        for (int i = Code::_B02_START; i <= Code::_B02_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::B05)
    {
        for (int i = Code::_B05_START; i <= Code::_B05_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::B06)
    {
        for (int i = Code::_B06_START; i <= Code::_B06_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::B07)
    {
        for (int i = Code::_B07_START; i <= Code::_B07_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::B08)
    {
        for (int i = Code::_B08_START; i <= Code::_B08_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::J01)
    {
        for (int i = Code::_J01_START; i <= Code::_J01_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::J02)
    {
        for (int i = Code::_J02_START; i <= Code::_J02_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::J05)
    {
        for (int i = Code::_J05_START; i <= Code::_J05_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::J06)
    {
        for (int i = Code::_J06_START; i <= Code::_J06_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::I05)
    {
        for (int i = Code::_I05_START; i <= Code::_I05_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::I09)
    {
        for (int i = Code::_I09_START; i <= Code::_I09_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::S01)
    {
        for (int i = Code::_S01_START; i <= Code::_S01_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    if (rhs & Frequency_::S05)
    {
        for (int i = Code::_S05_START; i <= Code::_S05_END; i++) { lhs |= Code(Code::Enum(i)); }
    }
    LOG_DATA("After  ({}) & ({})", Frequency(rhs), lhs);

    return lhs;
}
Code operator|(Frequency_ lhs, Code rhs)
{
    return rhs | lhs;
}
Code& Code::operator|=(const Frequency_& rhs)
{
    *this = *this | rhs;
    return *this;
}
Code operator|(Code lhs, Frequency rhs)
{
    return lhs | Frequency_(rhs);
}
Code operator|(Frequency lhs, Code rhs)
{
    return Frequency_(lhs) | rhs;
}
Code& Code::operator|=(const Frequency& rhs)
{
    *this = *this | rhs;
    return *this;
}

void to_json(json& j, const Code& data)
{
    j = Code::Set(data).to_string();
}
void from_json(const json& j, Code& data)
{
    data = Code(Code::Set(j.get<std::string>()));
}

bool ShowCodeSelector(const char* label, Code& code, const Frequency& filterFreq)
{
    bool valueChanged = false;
    if (ImGui::BeginCombo(label, std::string(code).c_str(), ImGuiComboFlags_HeightLargest))
    {
        if (ImGui::BeginTable(fmt::format("{} Table", label).c_str(), 7, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY))
        {
            ImGui::TableSetupScrollFreeze(0, 1);
            for (uint64_t satSys = 0xFF; satSys < 0xFFUL << (7 * 8); satSys = satSys << 8UL)
            {
                ImGui::TableSetupColumn(std::string(SatelliteSystem(SatelliteSystem_(satSys))).c_str());
            }
            ImGui::TableHeadersRow();
            ImGui::TableNextRow();
            for (int satSys = 0; satSys < 7; satSys++)
            {
                ImGui::TableSetColumnIndex(satSys);
                for (uint64_t f = 0; f < 8; f++)
                {
                    uint64_t flag = (1UL << (f + static_cast<uint64_t>(satSys) * 8));
                    auto frequency = Frequency(Frequency_(flag));
                    auto text = std::string(frequency);
                    if (text == "None")
                    {
                        continue;
                    }
                    bool hasCode = false;
                    for (int c = 0; c < Code::COUNT; c++)
                    {
                        auto co = Code(Code::Enum(c));
                        if (co.getFrequency() == frequency)
                        {
                            hasCode = true;
                            break;
                        }
                    }
                    if (hasCode)
                    {
                        ImGui::Text("%s", text.c_str());
                        if (!(frequency & filterFreq))
                        {
                            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.6F);
                        }
                        ImGui::Indent();
                        for (int c = 0; c < Code::COUNT; c++)
                        {
                            auto co = Code(Code::Enum(c));
                            if (co.getFrequency() == frequency)
                            {
                                auto checked = bool(co & code);
                                if (c > 0 && Code::IsCodeCombined(Code(Code::Enum(c - 1)), co))
                                {
                                    ImGui::SameLine();
                                }
                                if (ImGui::Checkbox(std::string(co).c_str(), &checked) && (frequency & filterFreq))
                                {
                                    if (checked)
                                    {
                                        code |= co;
                                    }
                                    else
                                    {
                                        code &= ~co;
                                    }
                                    valueChanged = true;
                                }
                                if (ImGui::IsItemHovered())
                                {
                                    ImGui::SetTooltip("%s", co.getDescription());
                                }
                            }
                        }
                        ImGui::Unindent();
                        if (!(frequency & filterFreq))
                        {
                            ImGui::PopStyleVar();
                        }
                    }
                }
            }

            ImGui::EndTable();
        }
        ImGui::EndCombo();
    }
    return valueChanged;
}

} // namespace NAV