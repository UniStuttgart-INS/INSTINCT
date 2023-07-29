// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Code.hpp
/// @brief Code definitions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-01

#pragma once

#include <bitset>
#include <fmt/format.h>

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"

namespace NAV
{

/// @brief Enumerate for GNSS Codes
///
/// <table>
///   <tr><th> System                                <th> Band                         <th> Code          <th> Description
///   <tr><td rowspan="16" valign="top"> GPS         <td rowspan="7" valign="top"> L1  <td> L1C           <td> C/A-code
///   <tr>                                                                             <td> L1S, L1L, L1X <td> L1C (data, pilot, combined)
///   <tr>                                                                             <td> L1P           <td> P-code (unencrypted)
///   <tr>                                                                             <td> L1W           <td> Semicodeless P(Y) tracking (Z-tracking)
///   <tr>                                                                             <td> L1Y           <td> Y-code (with decryption)
///   <tr>                                                                             <td> L1M           <td> M-code
///   <tr>                                                                             <td> L1N           <td> codeless
///   <tr>                                           <td rowspan="8" valign="top"> L2  <td> L2C           <td> C/A-code
///   <tr>                                                                             <td> L2D           <td> Semi-codeless P(Y) tracking (L1 C/A+(P2-P1))
///   <tr>                                                                             <td> L2S, L2L, L2X <td> L2C-code (medium, long, combined)
///   <tr>                                                                             <td> L2P           <td> P-code (unencrypted)
///   <tr>                                                                             <td> L2W           <td> Semicodeless P(Y) tracking (Z-tracking)
///   <tr>                                                                             <td> L2Y           <td> Y-code (with decryption)
///   <tr>                                                                             <td> L2M           <td> M-code
///   <tr>                                                                             <td> L2N           <td> codeless
///   <tr>                                                                    <td> L5  <td> L5I, L5Q, L5X <td> L5 (data, pilot, combined)
///   <tr><td rowspan= "7" valign="top"> Galileo     <td rowspan="3" valign="top"> E1  <td> L1A           <td> PRS signal
///   <tr>                                                                             <td> L1B, L1C, L1X <td> OS (data, pilot, combined)
///   <tr>                                                                             <td> L1Z           <td> PRS + OS(data+pilot)
///   <tr>                                                                    <td> E5a <td> L5I, L5Q, L5X <td> E5a (data, pilot, combined)
///   <tr>                                           <td rowspan="3" valign="top"> E6  <td> L6A           <td> E6 PRS signal
///   <tr>                                                                             <td> L6B, L6C, L6X <td> E6 (data, pilot, combined)
///   <tr>                                                                             <td> L6Z           <td> E6 PRS + OS(data+pilot)
///   <tr>                                                                    <td> E5b <td> L7I, L7Q, L7X <td> E5b (data, pilot, combined)
///   <tr>                                                                    <td> E5  <td> L8I, L8Q, L8X <td> E5 AltBOC (data, pilot, combined)
///   <tr><td rowspan= "7" valign="top"> GLONASS     <td rowspan="2" valign="top"> L1  <td> L1C           <td> C/A-code
///   <tr>                                                                             <td> L1P           <td> P-code
///   <tr>                                           <td rowspan="2" valign="top"> L2  <td> L2C           <td> C/A-code
///   <tr>                                                                             <td> L2P           <td> P-code
///   <tr>                                                                    <td> L3  <td> L3I, L3Q, L3X <td> L3 (data, pilot, combined)
///   <tr>                                                                    <td> G1a <td> L4A, L4B, L4X <td> G1a (data, pilot, combined)
///   <tr>                                                                    <td> G2b <td> L6A, L6B, L6X <td> G2a (data, pilot, combined)
///   <tr><td rowspan= "8" valign="top"> BeiDou (BDS-2/3)                     <td> B1  <td> L1D, L1P, L1X <td> B1 (data, pilot, combined)
///   <tr>                                                                    <td> B1-2<td> L2I, L2Q, L2X <td> B1I(OS), B1Q, combined
///   <tr>                                                                    <td> B2a <td> L5D, L5P, L5X <td> B2a (data, pilot, combined)
///   <tr>                                           <td rowspan="2" valign="top"> B3  <td> L6I, L6Q, L6X <td> B3I, B3Q, combined
///   <tr>                                                                             <td> L6A           <td> B3A
///   <tr>                                           <td rowspan="2" valign="top"> B2b <td> L7I, L7Q, L7X <td> BDS-2: B2I(OS), B2Q, combined
///   <tr>                                                                             <td> L7D, L7P, L7Z <td> BDS-3: data, pilot, combined
///   <tr>                                                                    <td> B2  <td> L8D, L8P, L8X <td> B2 (B2a+B2b): data, pilot, combined
///   <tr><td rowspan= "9" valign="top"> QZSS        <td rowspan="3" valign="top"> L1  <td> L1C           <td> C/A-code
///   <tr>                                                                             <td> L1S, L1L, L1X <td> L1C (data, pilot, combined)
///   <tr>                                                                             <td> L1Z           <td> L1-SAIF signal
///   <tr>                                                                    <td> L2  <td> L2S, L2L, L2X <td> L2C-code (medium, long, combined)
///   <tr>                                           <td rowspan="2" valign="top"> L5  <td> L5I, L5Q, L5X <td> L5 (data, pilot, combined)
///   <tr>                                                                             <td> L5D, L5P, L5Z <td> I, Q, combined
///   <tr>                                           <td rowspan="3" valign="top"> L6  <td> L6S, L6L, L6X <td> LEX signal (short, long, combined)
///   <tr>                                                                             <td> L6E           <td> L6E
///   <tr>                                                                             <td> L6Z           <td> L6(D+E)
///   <tr><td rowspan= "4" valign="top"> IRNSS/NavIC <td rowspan="2" valign="top"> L5  <td> L5A           <td> SPS Signal
///   <tr>                                                                             <td> L5B, L5C, L5X <td> RS (data, pilot, combined)
///   <tr>                                           <td rowspan="2" valign="top"> S   <td> L9A           <td> SPS signal
///   <tr>                                                                             <td> L9B, L9C, L9X <td> RS (data, pilot, combined)
///   <tr><td rowspan= "2" valign="top"> SBAS                                 <td> L1  <td> L1C           <td> C/A-code
///   <tr>                                                                    <td> L5  <td> L5I, L5Q, L5X <td> L5 (data, pilot, combined)
/// </table>
/// @note See \cite SpringerHandbookGNSS2017 Springer Handbook GNSS Annex A, Table A.5, p. 1211
/// @note See \cite RINEX-3.04 RINEX 3.04 Appendix A23, p. A43-A46 (97-100)
class Code
{
  public:
    /// @brief Enumeration of all Codes
    enum Enum
    {
        None, ///< None

        G1C, ///< GPS L1 - C/A-code
        G1S, ///< GPS L1 - L1C-D (data)
        G1L, ///< GPS L1 - L1C-P (pilot)
        G1X, ///< GPS L1 - L1C-(D+P) (combined)
        G1P, ///< GPS L1 - P-code (unencrypted)
        G1W, ///< GPS L1 - Semicodeless P(Y) tracking (Z-tracking)
        G1Y, ///< GPS L1 - Y-code (with decryption)
        G1M, ///< GPS L1 - M-code
        G1N, ///< GPS L1 - codeless
        G2C, ///< GPS L2 - C/A-code
        G2D, ///< GPS L2 - Semi-codeless P(Y) tracking (L1 C/A + (P2-P1))
        G2S, ///< GPS L2 - L2C(M) (medium)
        G2L, ///< GPS L2 - L2C(L) (long)
        G2X, ///< GPS L2 - L2C(M+L) (combined)
        G2P, ///< GPS L2 - P-code (unencrypted)
        G2W, ///< GPS L2 - Semicodeless P(Y) tracking (Z-tracking)
        G2Y, ///< GPS L2 - Y-code (with decryption)
        G2M, ///< GPS L2 - M-code
        G2N, ///< GPS L2 - codeless
        G5I, ///< GPS L5 - Data
        G5Q, ///< GPS L5 - Pilot
        G5X, ///< GPS L5 - Combined

        E1A, ///< GAL E1 - PRS signal
        E1B, ///< GAL E1 - OS (data)
        E1C, ///< GAL E1 - OS (pilot)
        E1X, ///< GAL E1 - OS(B+C) (combined)
        E1Z, ///< GAL E1 - PRS + OS (data + pilot)
        E5I, ///< GAL E5a - Data
        E5Q, ///< GAL E5a - Pilot
        E5X, ///< GAL E5a - Combined
        E6A, ///< GAL E6 - PRS signal
        E6B, ///< GAL E6 - Data
        E6C, ///< GAL E6 - Pilot
        E6X, ///< GAL E6 - Combined (B+C)
        E6Z, ///< GAL E6 - PRS + OS (A+B+C)
        E7I, ///< GAL E5b - Data
        E7Q, ///< GAL E5b - Pilot
        E7X, ///< GAL E5b - Combined
        E8I, ///< GAL E5(a+b) - AltBOC (data)
        E8Q, ///< GAL E5(a+b) - AltBOC (pilot)
        E8X, ///< GAL E5(a+b) - AltBOC (combined)

        R1C, ///< GLO L1 - C/A-code
        R1P, ///< GLO L1 - P-code
        R2C, ///< GLO L2 - C/A-code
        R2P, ///< GLO L2 - P-code
        R3I, ///< GLO L3 - Data
        R3Q, ///< GLO L3 - Pilot
        R3X, ///< GLO L3 - Combined
        R4A, ///< GLO G1a - L1OCd (data)
        R4B, ///< GLO G1a - L1OCp (pilot)
        R4X, ///< GLO G1a - L1OCd+L1OCp (combined)
        R6A, ///< GLO G2a - L2CSI (data)
        R6B, ///< GLO G2a - L2OCp (pilot)
        R6X, ///< GLO G2a - L2CSI+L2OCp (combined)

        B1D, ///< BeiDou B1 - Data (D)
        B1P, ///< BeiDou B1 - Pilot(P)
        B1X, ///< BeiDou B1 - D+P
        B2I, ///< BeiDou B1-2 - B1I(OS)
        B2Q, ///< BeiDou B1-2 - B1Q
        B2X, ///< BeiDou B1-2 - B1I(OS), B1Q, combined
        B5D, ///< BeiDou B2a - Data (D)
        B5P, ///< BeiDou B2a - Pilot(P)
        B5X, ///< BeiDou B2a - D+P
        B6I, ///< BeiDou B3 - B3I
        B6Q, ///< BeiDou B3 - B3Q
        B6X, ///< BeiDou B3 - B3I, B3Q, combined
        B6A, ///< BeiDou B3 - B3A
        B7I, ///< BeiDou B2b (BDS-2) - B2I(OS)
        B7Q, ///< BeiDou B2b (BDS-2) - B2Q
        B7X, ///< BeiDou B2b (BDS-2) - B2I(OS), B2Q, combined
        B7D, ///< BeiDou B2b (BDS-3) - Data (D)
        B7P, ///< BeiDou B2b (BDS-3) - Pilot(P)
        B7Z, ///< BeiDou B2b (BDS-3) - D+P
        B8D, ///< BeiDou B2 (B2a+B2b) - Data (D)
        B8P, ///< BeiDou B2 (B2a+B2b) - Pilot(P)
        B8X, ///< BeiDou B2 (B2a+B2b) - D+P

        J1C, ///< QZSS L1 - C/A-code
        J1S, ///< QZSS L1 - L1C (data)
        J1L, ///< QZSS L1 - L1C (pilot)
        J1X, ///< QZSS L1 - L1C (combined)
        J1Z, ///< QZSS L1 - L1-SAIF signal
        J2S, ///< QZSS L2 - L2C-code (medium)
        J2L, ///< QZSS L2 - L2C-code (long)
        J2X, ///< QZSS L2 - L2C-code (combined)
        J5I, ///< QZSS L5 - Data
        J5Q, ///< QZSS L5 - Pilot
        J5X, ///< QZSS L5 - Combined
        J5D, ///< QZSS L5S - I
        J5P, ///< QZSS L5S - Q
        J5Z, ///< QZSS L5S - I+Q
        J6S, ///< QZSS L6 - L6D LEX signal (short)
        J6L, ///< QZSS L6 - L6P LEX signal (long)
        J6X, ///< QZSS L6 - L6(D+P) LEX signal (combined)
        J6E, ///< QZSS L6 - L6E
        J6Z, ///< QZSS L6 - L6(D+E)

        I5A, ///< IRNSS L5 - SPS Signal
        I5B, ///< IRNSS L5 - RS (data)
        I5C, ///< IRNSS L5 - RS (pilot)
        I5X, ///< IRNSS L5 - RS (combined)
        I9A, ///< IRNSS S - SPS signal
        I9B, ///< IRNSS S - RS (data)
        I9C, ///< IRNSS S - RS (pilot)
        I9X, ///< IRNSS S - RS (combined)

        S1C, ///< SBAS L1 - C/A-code
        S5I, ///< SBAS L5 - Data
        S5Q, ///< SBAS L5 - Pilot
        S5X, ///< SBAS L5 - Combined
    };
    /// @brief Helper variables
    enum
    {
        _GPS_START = G1C,   ///< GPS start index in the Enum
        _GPS_END = G5X,     ///< GPS end index in the Enum
        _GAL_START = E1A,   ///< GAL start index in the Enum
        _GAL_END = E8X,     ///< GAL end index in the Enum
        _GLO_START = R1C,   ///< GLO start index in the Enum
        _GLO_END = R6X,     ///< GLO end index in the Enum
        _BDS_START = B1D,   ///< BDS start index in the Enum
        _BDS_END = B8X,     ///< BDS end index in the Enum
        _QZSS_START = J1C,  ///< QZSS start index in the Enum
        _QZSS_END = J6Z,    ///< QZSS end index in the Enum
        _IRNSS_START = I5A, ///< IRNSS start index in the Enum
        _IRNSS_END = I9X,   ///< IRNSS end index in the Enum
        _SBAS_START = S1C,  ///< SBAS start index in the Enum
        _SBAS_END = S5X,    ///< SBAS end index in the Enum

        _G01_START = G1C, ///< GPS L1 start index in the Enum
        _G01_END = G1N,   ///< GPS L1 end index in the Enum
        _G02_START = G2C, ///< GPS L2 start index in the Enum
        _G02_END = G2N,   ///< GPS L2 end index in the Enum
        _G05_START = G5I, ///< GPS L5 start index in the Enum
        _G05_END = G5X,   ///< GPS L5 end index in the Enum

        _E01_START = E1A, ///< GAL E1 start index in the Enum
        _E01_END = E1Z,   ///< GAL E1 end index in the Enum
        _E05_START = E5I, ///< GAL E5a start index in the Enum
        _E05_END = E5X,   ///< GAL E5a end index in the Enum
        _E06_START = E6B, ///< GAL E6 start index in the Enum
        _E06_END = E6X,   ///< GAL E6 end index in the Enum
        _E07_START = E7I, ///< GAL E5b start index in the Enum
        _E07_END = E7X,   ///< GAL E5b end index in the Enum
        _E08_START = E8I, ///< GAL E5(a+b) start index in the Enum
        _E08_END = E8X,   ///< GAL E5(a+b) end index in the Enum

        _R01_START = R1C, ///< GLO L1 start index in the Enum
        _R01_END = R1P,   ///< GLO L1 end index in the Enum
        _R02_START = R2C, ///< GLO L2 start index in the Enum
        _R02_END = R2P,   ///< GLO L2 end index in the Enum
        _R03_START = R3I, ///< GLO L3 start index in the Enum
        _R03_END = R3X,   ///< GLO L3 end index in the Enum
        _R04_START = R4A, ///< GLO G1a start index in the Enum
        _R04_END = R4X,   ///< GLO G1a end index in the Enum
        _R06_START = R6A, ///< GLO G2a start index in the Enum
        _R06_END = R6X,   ///< GLO G2a end index in the Enum

        _B01_START = B1D, ///< BDS B1 start index in the Enum
        _B01_END = B1X,   ///< BDS B1 end index in the Enum
        _B02_START = B2I, ///< BDS B1-2 start index in the Enum
        _B02_END = B2X,   ///< BDS B1-2 end index in the Enum
        _B05_START = B5D, ///< BDS B2a start index in the Enum
        _B05_END = B5X,   ///< BDS B2a end index in the Enum
        _B06_START = B6I, ///< BDS B3 start index in the Enum
        _B06_END = B6A,   ///< BDS B3 end index in the Enum
        _B07_START = B7I, ///< BDS B2b start index in the Enum
        _B07_END = B7Z,   ///< BDS B2b end index in the Enum
        _B08_START = B8D, ///< BDS B2 (B2a+B2b) start index in the Enum
        _B08_END = B8X,   ///< BDS B2 (B2a+B2b) end index in the Enum

        _J01_START = J1C, ///< QZSS L1 start index in the Enum
        _J01_END = J1Z,   ///< QZSS L1 end index in the Enum
        _J02_START = J2S, ///< QZSS L2 start index in the Enum
        _J02_END = J2X,   ///< QZSS L2 end index in the Enum
        _J05_START = J5I, ///< QZSS L5 start index in the Enum
        _J05_END = J5Z,   ///< QZSS L5 end index in the Enum
        _J06_START = J6S, ///< QZSS L6 start index in the Enum
        _J06_END = J6Z,   ///< QZSS L6 end index in the Enum

        _I05_START = I5A, ///< IRNSS L5 start index in the Enum
        _I05_END = I5X,   ///< IRNSS L5 end index in the Enum
        _I09_START = I9A, ///< IRNSS S start index in the Enum
        _I09_END = I9X,   ///< IRNSS S end index in the Enum

        _S01_START = S1C, ///< SBAS L start index in the Enum
        _S01_END = S1C,   ///< SBAS L end index in the Enum
        _S05_START = S5I, ///< SBAS L start index in the Enum
        _S05_END = S5X,   ///< SBAS L end index in the Enum

        COUNT = S5X + 1 ///< Amount of Codes in the Enum
    };

    /// Typedef for the bitset with size of COUNT
    using Set = std::bitset<COUNT>;

    /// Default constructor for an empty code
    constexpr Code() = default;

    /// @brief Constructor from a biset
    /// @param[in] set Bitset with values to construct the Code from
    constexpr explicit Code(const Set& set) : value(set) {}

    /// @brief Constructor from a frequency
    /// @param[in] freq Frequency to set all codes for
    explicit Code(Frequency_ freq);

    /// @brief Constructor from a single code value
    /// @param[in] e Code enum value to construct the code from
    constexpr Code(Enum e) { value.set(e, true); } // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)

    /// Implicit bool conversion operator. Allows if(...)
    constexpr explicit operator bool() const { return value.any(); }

    /// @brief std::bitset conversion operator
    /// @return The bitset representation of the type
    constexpr explicit operator Set() const { return value; }

    /// @brief std::string conversion operator
    /// @return A std::string representation of the type
    explicit operator std::string() const;

    /// @brief Generates a Code from frequency and attribute
    /// @param[in] freq Frequency of the code
    /// @param[in] attribute Attribute letter of the code
    /// @return Code representation
    static Code fromFreqAttr(Frequency freq, char attribute);

    /// @brief Returns a short description for the code
    /// @param[in] code GNSS signal code
    static const char* GetCodeDescription(Code code);

    /// @brief Returns a short description for the code
    [[nodiscard]] const char* getDescription() const;

    /// @brief Returns the frequency for the code
    /// @param[in] code GNSS signal code
    static Frequency GetCodeFequency(Code code);

    /// @brief Returns the frequency for the code
    [[nodiscard]] Frequency getFrequency() const;

    /// @brief Checks if the codes are part of a combined code
    /// @param[in] first First GNSS signal code
    /// @param[in] second Second GNSS signal code
    static bool IsCodeCombined(Code first, Code second);

    /// @brief Returns a list with all possible codes
    static std::vector<Code> GetAll();

    /// @brief Returns the enum value for the code (only one must be set)
    /// @param code GNSS signal code
    static Enum GetCodeEnumValue(Code code);

    /// @brief Returns the enum value for the code (only one must be set)
    [[nodiscard]] Enum getEnumValue() const;

    // #####################################################################################################################################

    /// @brief Allows combining flags of the Code enum.
    /// @param[in] lhs Left-hand side value.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ORed value.
    friend Code operator|(const Code& lhs, const Code& rhs);
    /// @brief Allows combining flags of the Code enum.
    /// @param[in] lhs Left-hand side value.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ORed value.
    friend Code operator|(const Code& lhs, const Enum& rhs);
    /// @brief Allows combining flags of the Code enum.
    /// @param[in] lhs Left-hand side value.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ORed value.
    friend Code operator|(const Enum& lhs, const Code& rhs);

    /// @brief Allows combining flags of the Code enum.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ORed value.
    Code& operator|=(const Code& rhs);
    /// @brief Allows combining flags of the Code enum.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ORed value.
    Code& operator|=(const Enum& rhs);

    /// @brief Allows combining flags of the Code enum.
    /// @param[in] lhs Left-hand side value.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ANDed value.
    friend Code operator&(const Code& lhs, const Code& rhs);
    /// @brief Allows combining flags of the Code enum.
    /// @param[in] lhs Left-hand side value.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ANDed value.
    friend Code operator&(const Code& lhs, const Enum& rhs);
    /// @brief Allows combining flags of the Code enum.
    /// @param[in] lhs Left-hand side value.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ANDed value.
    friend Code operator&(const Enum& lhs, const Code& rhs);

    /// @brief Allows combining flags of the Code enum.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ANDed value.
    Code& operator&=(const Code& rhs);
    /// @brief Allows combining flags of the Code enum.
    /// @param[in] rhs Right-hand side value.
    /// @return The binary ANDed value.
    Code& operator&=(const Enum& rhs);

    /// @brief Allows negating flags of the Code enum.
    Code operator~() const;

    // #####################################################################################################################################

    /// @brief Equal compares values
    /// @param[in] lhs Left-hand side of the operator
    /// @param[in] rhs Right-hand side of the operator
    /// @return Whether the comparison was successful
    friend bool operator==(const Code& lhs, const Code& rhs);

    /// @brief Equal compares values
    /// @param[in] lhs Left-hand side of the operator
    /// @param[in] rhs Right-hand side of the operator
    /// @return Whether the comparison was successful
    friend bool operator==(const Code& lhs, const Enum& rhs);

    /// @brief Equal compares values
    /// @param[in] lhs Left-hand side of the operator
    /// @param[in] rhs Right-hand side of the operator
    /// @return Whether the comparison was successful
    friend bool operator==(const Enum& lhs, const Code& rhs);

    /// @brief Inequal compares values
    /// @param[in] lhs Left-hand side of the operator
    /// @param[in] rhs Right-hand side of the operator
    /// @return Whether the comparison was successful
    friend bool operator!=(const Code& lhs, const Code& rhs);
    /// @brief Inequal compares values
    /// @param[in] lhs Left-hand side of the operator
    /// @param[in] rhs Right-hand side of the operator
    /// @return Whether the comparison was successful
    friend bool operator!=(const Code& lhs, const Enum& rhs);
    /// @brief Inequal compares values
    /// @param[in] lhs Left-hand side of the operator
    /// @param[in] rhs Right-hand side of the operator
    /// @return Whether the comparison was successful
    friend bool operator!=(const Enum& lhs, const Code& rhs);

    // #####################################################################################################################################

    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(Code lhs, SatelliteSystem_ rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(SatelliteSystem_ lhs, Code rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    Code& operator&=(const SatelliteSystem_& rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(Code lhs, SatelliteSystem rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(SatelliteSystem lhs, Code rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    Code& operator&=(const SatelliteSystem& rhs);

    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(Code lhs, SatelliteSystem_ rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(SatelliteSystem_ lhs, Code rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    Code& operator|=(const SatelliteSystem_& rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(Code lhs, SatelliteSystem rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(SatelliteSystem lhs, Code rhs);
    /// @brief Allows filtering Code with SatelliteSystem.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    Code& operator|=(const SatelliteSystem& rhs);

    // #####################################################################################################################################

    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(Code lhs, Frequency_ rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(Frequency_ lhs, Code rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    Code& operator&=(const Frequency_& rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(Code lhs, Frequency rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    friend Code operator&(Frequency lhs, Code rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ANDed value.
    Code& operator&=(const Frequency& rhs);

    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(Code lhs, Frequency_ rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(Frequency_ lhs, Code rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    Code& operator|=(const Frequency_& rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(Code lhs, Frequency rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] lhs Left-hand side enum value.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    friend Code operator|(Frequency lhs, Code rhs);
    /// @brief Allows filtering Code with Frequency.
    /// @param[in] rhs Right-hand side enum value.
    /// @return The binary ORed value.
    Code& operator|=(const Frequency& rhs);

    // #####################################################################################################################################

  private:
    /// Code bitset
    Set value{};
};

// #########################################################################################################################################

/// @brief Converts the provided link into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const Code& data);
/// @brief Converts the provided json object into a link object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, Code& data);

// #########################################################################################################################################

/// @brief Allows combining flags of the Code enum.
/// @param[in] lhs Left-hand side value.
/// @param[in] rhs Right-hand side value.
/// @return The binary ORed value.
Code operator|(const Code::Enum& lhs, const Code::Enum& rhs);
/// @brief Allows combining flags of the Code enum.
/// @param[in] lhs Left-hand side value.
/// @param[in] rhs Right-hand side value.
/// @return The binary ANDed value.
Code operator&(const Code::Enum& lhs, const Code::Enum& rhs);

const Code Code_G1S_G1L_G1X = Code::G1S | Code::G1L | Code::G1X; ///< L1C (data, pilot, combined)
const Code Code_G2S_G2L_G2X = Code::G2S | Code::G2L | Code::G2X; ///< L2C-code (medium, long, combined)
const Code Code_G5I_G5Q_G5X = Code::G5I | Code::G5Q | Code::G5X; ///< L5 (data, pilot, combined)

const Code Code_E1B_E1C_E1X = Code::E1B | Code::E1C | Code::E1X; ///< OS (data, pilot, combined)
const Code Code_E5I_E5Q_E5X = Code::E5I | Code::E5Q | Code::E5X; ///< E5a (data, pilot, combined)
const Code Code_E6B_E6C_E6X = Code::E6B | Code::E6C | Code::E6X; ///< E6 (data, pilot, combined)
const Code Code_E7I_E7Q_E7X = Code::E7I | Code::E7Q | Code::E7X; ///< E5b (data, pilot, combined)
const Code Code_E8I_E8Q_E8X = Code::E8I | Code::E8Q | Code::E8X; ///< E5 AltBOC (data, pilot, combined)

const Code Code_R3I_R3Q_R3X = Code::R3I | Code::R3Q | Code::R3X; ///< L3 (data, pilot, combined)
const Code Code_R4A_R4B_R4X = Code::R4A | Code::R4B | Code::R4X; ///< G1a (data, pilot, combined)
const Code Code_R6A_R6B_R6X = Code::R6A | Code::R6B | Code::R6X; ///< G2a (data, pilot, combined)

const Code Code_B1D_B1P_B1X = Code::B1D | Code::B1P | Code::B1X; ///< B1 (data, pilot, combined)
const Code Code_B2I_B2Q_B2X = Code::B2I | Code::B2Q | Code::B2X; ///< B1I(OS), B1Q, combined
const Code Code_B5D_B5P_B5X = Code::B5D | Code::B5P | Code::B5X; ///< B2a (data, pilot, combined)
const Code Code_B6I_B6Q_B6X = Code::B6I | Code::B6Q | Code::B6X; ///< B3I, B3Q, combined
const Code Code_B7I_B7Q_B7X = Code::B7I | Code::B7Q | Code::B7X; ///< B2I(OS), B2Q, combined
const Code Code_B7D_B7P_B7Z = Code::B7D | Code::B7P | Code::B7Z; ///< B2b (data, pilot, combined)
const Code Code_B8D_B8P_B8X = Code::B8D | Code::B8P | Code::B8X; ///< B2 (B2a+B2b) (data, pilot, combined)

const Code Code_J1S_J1L_J1X = Code::J1S | Code::J1L | Code::J1X; ///< L1C (data, pilot, combined)
const Code Code_J2S_J2L_J2X = Code::J2S | Code::J2L | Code::J2X; ///< L2C-code (medium, long, combined)
const Code Code_J5I_J5Q_J5X = Code::J5I | Code::J5Q | Code::J5X; ///< L5 (data, pilot, combined)
const Code Code_J5D_J5P_J5Z = Code::J5D | Code::J5P | Code::J5Z; ///< L5 (data, pilot, combined)
const Code Code_J6S_J6L_J6X = Code::J6S | Code::J6L | Code::J6X; ///< LEX signal (short, long, combined)

const Code Code_I5B_I5C_I5X = Code::I5B | Code::I5C | Code::I5X; ///< RS (data, pilot, combined)
const Code Code_I9B_I9C_I9X = Code::I9B | Code::I9C | Code::I9X; ///< RS (data, pilot, combined)

const Code Code_S5I_S5Q_S5X = Code::S5I | Code::S5Q | Code::S5X; ///< L5 (data, pilot, combined)

const Code Code_ALL = Code(Code::Set().set()); ///< All codes set

/// @brief Shows a ComboBox to select signal codes
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in, out] code Reference to the code object to select
/// @param[in] filterFreq Frequencies to select codes for. Other Frequencies will be diabled.
bool ShowCodeSelector(const char* label, Code& code, const Frequency& filterFreq);

} // namespace NAV

namespace std
{

/// @brief Hash function for Frequency (needed for unordered_map)
template<>
struct hash<NAV::Code>
{
    /// @brief Hash function for signal code
    /// @param[in] c Signal code
    /// @return Has value for the signal code
    std::size_t operator()(const NAV::Code& c) const
    {
        return static_cast<size_t>(c.getEnumValue());
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for Code
template<>
struct fmt::formatter<NAV::Code>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format Code structs
    /// @param[in] code Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Code& code, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "{0}", std::string(code));
    }
};

#endif