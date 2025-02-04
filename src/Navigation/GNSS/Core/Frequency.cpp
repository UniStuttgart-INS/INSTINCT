// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Frequency.hpp"

#include <algorithm>
#include <cmath>
#include <imgui.h>
#include <imgui_internal.h>

#include "util/Assert.h"
#include "util/Logger.hpp"

namespace NAV
{

Frequency Frequency::fromString(const std::string& typeString)
{
    if (typeString == "B1" || typeString == "B01" || typeString == "C01") { return B01; }
    if (typeString == "B2" || typeString == "B08" || typeString == "C08") { return B08; }
    if (typeString == "B3" || typeString == "B06" || typeString == "C06") { return B06; }
    if (typeString == "B1-2" || typeString == "B02" || typeString == "C02") { return B02; }
    if (typeString == "B2a" || typeString == "B05" || typeString == "C05") { return B05; }
    if (typeString == "B2b" || typeString == "B07" || typeString == "C07") { return B07; }
    if (typeString == "E1" || typeString == "E01") { return E01; }
    if (typeString == "E5a" || typeString == "E05") { return E05; }
    if (typeString == "E6" || typeString == "E06") { return E06; }
    if (typeString == "E5b" || typeString == "E07") { return E07; }
    if (typeString == "E5" || typeString == "E08") { return E08; }
    if (typeString == "L1" || typeString == "G01") { return G01; }
    if (typeString == "L2" || typeString == "G02") { return G02; }
    if (typeString == "L5" || typeString == "G05") { return G05; }
    if (typeString == "I5" || typeString == "I05") { return I05; }
    if (typeString == "IS" || typeString == "I09") { return I09; }
    if (typeString == "Q1" || typeString == "J01") { return J01; }
    if (typeString == "Q2" || typeString == "J02") { return J02; }
    if (typeString == "Q5" || typeString == "J05") { return J05; }
    if (typeString == "Q6" || typeString == "QLEX" || typeString == "J06") { return J06; }
    if (typeString == "G1" || typeString == "R01") { return R01; }
    if (typeString == "G2" || typeString == "R02") { return R02; }
    if (typeString == "G3" || typeString == "R03") { return R03; }
    if (typeString == "G1a" || typeString == "R04") { return R04; }
    if (typeString == "G2a" || typeString == "R06") { return R06; }
    if (typeString == "S1" || typeString == "S01") { return S01; }
    if (typeString == "S5" || typeString == "S05") { return S05; }

    return Freq_None;
}

Frequency Frequency::fromEnum(Frequency::Enum enumeration)
{
    switch (enumeration)
    {
    case Frequency::Enum_G01:
        return G01;
    case Frequency::Enum_G02:
        return G02;
    case Frequency::Enum_G05:
        return G05;
    case Frequency::Enum_E01:
        return E01;
    case Frequency::Enum_E05:
        return E05;
    case Frequency::Enum_E06:
        return E06;
    case Frequency::Enum_E07:
        return E07;
    case Frequency::Enum_E08:
        return E08;
    case Frequency::Enum_R01:
        return R01;
    case Frequency::Enum_R02:
        return R02;
    case Frequency::Enum_R03:
        return R03;
    case Frequency::Enum_R04:
        return R04;
    case Frequency::Enum_R06:
        return R06;
    case Frequency::Enum_B01:
        return B01;
    case Frequency::Enum_B02:
        return B02;
    case Frequency::Enum_B05:
        return B05;
    case Frequency::Enum_B06:
        return B06;
    case Frequency::Enum_B07:
        return B07;
    case Frequency::Enum_B08:
        return B08;
    case Frequency::Enum_J01:
        return J01;
    case Frequency::Enum_J02:
        return J02;
    case Frequency::Enum_J05:
        return J05;
    case Frequency::Enum_J06:
        return J06;
    case Frequency::Enum_I05:
        return I05;
    case Frequency::Enum_I09:
        return I09;
    case Frequency::Enum_S01:
        return S01;
    case Frequency::Enum_S05:
        return S05;
    case Frequency::Enum_COUNT:
    case Frequency::Enum_None:
        return Freq_None;
    }
    return Freq_None;
}

Frequency::operator std::string() const
{
    const std::string filler = " | ";
    std::string str;
    if (value & G01)
    {
        str += (!str.empty() ? filler : "") + "L1";
    }
    if (value & G02)
    {
        str += (!str.empty() ? filler : "") + "L2";
    }
    if (value & G05)
    {
        str += (!str.empty() ? filler : "") + "L5";
    }
    if (value & E01)
    {
        str += (!str.empty() ? filler : "") + "E1";
    }
    if (value & E05)
    {
        str += (!str.empty() ? filler : "") + "E5a";
    }
    if (value & E06)
    {
        str += (!str.empty() ? filler : "") + "E6";
    }
    if (value & E07)
    {
        str += (!str.empty() ? filler : "") + "E5b";
    }
    if (value & E08)
    {
        str += (!str.empty() ? filler : "") + "E5";
    }
    if (value & R01)
    {
        str += (!str.empty() ? filler : "") + "G1";
    }
    if (value & R02)
    {
        str += (!str.empty() ? filler : "") + "G2";
    }
    if (value & R03)
    {
        str += (!str.empty() ? filler : "") + "G3";
    }
    if (value & R04)
    {
        str += (!str.empty() ? filler : "") + "G1a";
    }
    if (value & R06)
    {
        str += (!str.empty() ? filler : "") + "G2a";
    }
    if (value & B01)
    {
        str += (!str.empty() ? filler : "") + "B1";
    }
    if (value & B08)
    {
        str += (!str.empty() ? filler : "") + "B2";
    }
    if (value & B06)
    {
        str += (!str.empty() ? filler : "") + "B3";
    }
    if (value & B02)
    {
        str += (!str.empty() ? filler : "") + "B1-2";
    }
    if (value & B05)
    {
        str += (!str.empty() ? filler : "") + "B2a";
    }
    if (value & B07)
    {
        str += (!str.empty() ? filler : "") + "B2b";
    }
    if (value & J01)
    {
        str += (!str.empty() ? filler : "") + "Q1";
    }
    if (value & J02)
    {
        str += (!str.empty() ? filler : "") + "Q2";
    }
    if (value & J05)
    {
        str += (!str.empty() ? filler : "") + "Q5";
    }
    if (value & J06)
    {
        str += (!str.empty() ? filler : "") + "Q6";
    }
    if (value & I05)
    {
        str += (!str.empty() ? filler : "") + "I5";
    }
    if (value & I09)
    {
        str += (!str.empty() ? filler : "") + "IS";
    }
    if (value & S01)
    {
        str += (!str.empty() ? filler : "") + "S1";
    }
    if (value & S05)
    {
        str += (!str.empty() ? filler : "") + "S5";
    }

    if (!str.empty())
    {
        return str;
    }
    return "None";
}

SatelliteSystem Frequency::GetSatelliteSystemForFrequency(Frequency freq)
{
    SatelliteSystem retVal = SatSys_None;
    for (auto satSys : SatelliteSystem::GetAll())
    {
        if (freq & satSys) { retVal |= satSys; }
    }
    return retVal;
}

double Frequency::GetFrequency(Frequency freq, int8_t num)
{
    switch (Frequency_(freq))
    {
    case B01: // Beidou B1 (1575.42 MHz)
        return 1575.42e6;
    case B02: // Beidou B1-2 (1561.098 MHz)
        return 1561.098e6;
    case B05: // Beidou B2a (1176.45 MHz)
        return 1176.45e6;
    case B06: // Beidou B3 (1268.52 MHz)
        return 1268.52e6;
    case B07: // Beidou B2b (1207.14 MHz)
        return 1207.14e6;
    case B08: // Beidou B2 (B2a + B2b) (1191.795 MHz)
        return 1191.795e6;
    case E01: // Galileo, "E1" (1575.42 MHz)
        return 1575.42e6;
    case E05: // Galileo E5a (1176.45 MHz)
        return 1176.45e6;
    case E06: // Galileo E6 (1278.75 MHz)
        return 1278.75e6;
    case E07: // Galileo E5b (1207.14 MHz)
        return 1207.14e6;
    case E08: // Galileo E5 (E5a + E5b) (1191.795 MHz)
        return 1191.795e6;
    case G01: // GPS L1 (1575.42 MHz)
        return 1575.42e6;
    case G02: // GPS L2 (1227.6 MHz)
        return 1227.6e6;
    case G05: // GPS L5 (1176.45 MHz)
        return 1176.45e6;
    case I05: // IRNSS L5 (1176.45 MHz)
        return 1176.45e6;
    case I09: // IRNSS S (2492.028 MHz)
        return 2492.028e6;
    case J01: // QZSS L1 (1575.42 MHz)
        return 1575.42e6;
    case J02: // QZSS L2 (1227.6 MHz)
        return 1227.6e6;
    case J05: // QZSS L5 (1176.45 MHz)
        return 1176.45e6;
    case J06: // QZSS L6 / LEX (1278.75 MHz)
        return 1278.75e6;
    case R01: // GLONASS, "G1" (1602 MHZ)
        INS_ASSERT_USER_ERROR(num >= -7 && num <= 6, "GLONASS G1 frequency numbers have to be in the range [-7, +6] (all satellites launched after 2005)");
        return (1602.0 + num * 9.0 / 16.0) * 1e6;
    case R02: // GLONASS, "G2" (1246 MHz)
        INS_ASSERT_USER_ERROR(num >= -7 && num <= 6, "GLONASS G1 frequency numbers have to be in the range [-7, +6] (all satellites launched after 2005)");
        return (1246.0 + num * 7.0 / 16.0) * 1e6;
    case R03: // GLONASS, "G3" (1202.025 MHz)
        return 1202.025e6;
    case R04: // GLONASS, "G1a" (1600.995 MHZ)
        return 1600.995e6;
    case R06: // GLONASS, "G2a" (1248.06 MHz)
        return 1248.06e6;
    case S01: // SBAS L1 (1575.42 MHz)
        return 1575.42e6;
    case S05: // SBAS L5 (1176.45 MHz)
        return 1176.45e6;
    case Freq_None:
        return std::nan("");
    }

    return std::nan("");
}

Frequency Frequency::GetL1(Frequency freq)
{
    switch (Frequency_(freq))
    {
    case B01: // Beidou B1 (1575.42 MHz)
    case B02: // Beidou B1-2 (1561.098 MHz)
    case B05: // Beidou B2a (1176.45 MHz)
    case B06: // Beidou B3 (1268.52 MHz)
    case B07: // Beidou B2b (1207.14 MHz)
    case B08: // Beidou B2 (B2a + B2b) (1191.795 MHz)
        return B01;
    case E01: // Galileo, "E1" (1575.42 MHz)
    case E05: // Galileo E5a (1176.45 MHz)
    case E06: // Galileo E6 (1278.75 MHz)
    case E07: // Galileo E5b (1207.14 MHz)
    case E08: // Galileo E5 (E5a + E5b) (1191.795 MHz)
        return E01;
    case G01: // GPS L1 (1575.42 MHz)
    case G02: // GPS L2 (1227.6 MHz)
    case G05: // GPS L5 (1176.45 MHz)
        return G01;
    case I05: // IRNSS L5 (1176.45 MHz)
    case I09: // IRNSS S (2492.028 MHz)
        return I05;
    case J01: // QZSS L1 (1575.42 MHz)
    case J02: // QZSS L2 (1227.6 MHz)
    case J05: // QZSS L5 (1176.45 MHz)
    case J06: // QZSS L6 / LEX (1278.75 MHz)
        return J01;
    case R01: // GLONASS, "G1" (1602 MHZ)
    case R02: // GLONASS, "G2" (1246 MHz)
    case R03: // GLONASS, "G3" (1202.025 MHz)
    case R04: // GLONASS, "G1a" (1600.995 MHZ)
    case R06: // GLONASS, "G2a" (1248.06 MHz)
        return R01;
    case S01: // SBAS L1 (1575.42 MHz)
    case S05: // SBAS L5 (1176.45 MHz)
        return S01;
    case Freq_None:
        return Freq_None;
    }

    return Freq_None;
}

size_t Frequency::count() const
{
    size_t num = 0;
    if (value & G01) { num += 1; }
    if (value & G02) { num += 1; }
    if (value & G05) { num += 1; }
    if (value & E01) { num += 1; }
    if (value & E05) { num += 1; }
    if (value & E06) { num += 1; }
    if (value & E07) { num += 1; }
    if (value & E08) { num += 1; }
    if (value & R01) { num += 1; }
    if (value & R02) { num += 1; }
    if (value & R03) { num += 1; }
    if (value & R04) { num += 1; }
    if (value & R06) { num += 1; }
    if (value & B01) { num += 1; }
    if (value & B02) { num += 1; }
    if (value & B05) { num += 1; }
    if (value & B06) { num += 1; }
    if (value & B07) { num += 1; }
    if (value & B08) { num += 1; }
    if (value & J01) { num += 1; }
    if (value & J02) { num += 1; }
    if (value & J05) { num += 1; }
    if (value & J06) { num += 1; }
    if (value & I05) { num += 1; }
    if (value & I09) { num += 1; }
    if (value & S01) { num += 1; }
    if (value & S05) { num += 1; }

    return num;
}

Frequency::Enum Frequency::ToEnumeration(Frequency freq)
{
    switch (Frequency_(freq))
    {
    case Freq_None:
        return Enum_None;
    case G01:
        return Enum_G01;
    case G02:
        return Enum_G02;
    case G05:
        return Enum_G05;
    case E01:
        return Enum_E01;
    case E05:
        return Enum_E05;
    case E06:
        return Enum_E06;
    case E07:
        return Enum_E07;
    case E08:
        return Enum_E08;
    case R01:
        return Enum_R01;
    case R02:
        return Enum_R02;
    case R03:
        return Enum_R03;
    case R04:
        return Enum_R04;
    case R06:
        return Enum_R06;
    case B01:
        return Enum_B01;
    case B02:
        return Enum_B02;
    case B05:
        return Enum_B05;
    case B06:
        return Enum_B06;
    case B07:
        return Enum_B07;
    case B08:
        return Enum_B08;
    case J01:
        return Enum_J01;
    case J02:
        return Enum_J02;
    case J05:
        return Enum_J05;
    case J06:
        return Enum_J06;
    case I05:
        return Enum_I05;
    case I09:
        return Enum_I09;
    case S01:
        return Enum_S01;
    case S05:
        return Enum_S05;
    }
    return Enum_None;
}

Frequency::Enum Frequency::toEnumeration() const
{
    return ToEnumeration(*this);
}

std::vector<Frequency> Frequency::ToVector(Frequency freq)
{
    std::vector<Frequency> v;
    for (const Frequency& f : GetAll())
    {
        if (f.value & freq.value) { v.push_back(f); }
    }
    return v;
}

std::vector<Frequency> Frequency::toVector() const
{
    return ToVector(value);
}

bool Frequency::IsFirstFrequency(const Frequency& freq, const Frequency& filter)
{
    return freq.isFirstFrequency(filter);
}

bool Frequency::isFirstFrequency(const Frequency& filter) const
{
    Frequency_ f = filter & getSatSys();
    f = Frequency_(f ^ value);

    if (f == Freq_None) { return true; }

    auto frequencies = Frequency(f).toVector();
    return std::ranges::all_of(frequencies,
                               [&](const Frequency& freq) { return value < Frequency_(freq); });
}

void to_json(json& j, const Frequency& data)
{
    j = std::string(data);
}
void from_json(const json& j, Frequency& data)
{
    data = Frequency::fromString(j.get<std::string>());
}

bool ShowFrequencySelector(const char* label, Frequency& frequency, bool singleSelect)
{
    bool valueChanged = false;
    if (ImGui::BeginCombo(label, std::string(frequency).c_str(), ImGuiComboFlags_HeightLargest))
    {
        if (ImGui::BeginTable(fmt::format("{} Table", label).c_str(), 7, ImGuiTableFlags_BordersInnerV))
        {
            for (uint64_t satSys = 0xFF; satSys < 0xFFUL << (7 * 8); satSys = satSys << 8UL)
            {
                ImGui::TableSetupColumn(std::string(SatelliteSystem(SatelliteSystem_(satSys))).c_str());
            }
            ImGui::TableHeadersRow();
            for (uint64_t f = 0; f < 8; f++)
            {
                ImGui::TableNextRow();
                for (int c = 0; c < 7; c++)
                {
                    uint64_t flag = (1UL << (f + static_cast<uint64_t>(c) * 8));
                    auto text = std::string(Frequency(Frequency_(flag)));
                    if (text == "None")
                    {
                        continue;
                    }
                    ImGui::TableSetColumnIndex(c);
                    if (c >= 5)
                    {
                        ImGui::BeginDisabled();
                    }
                    if (singleSelect)
                    {
                        bool selected = flag & Frequency_(frequency);
                        if (ImGui::Checkbox(text.c_str(), &selected) && frequency != Frequency_(flag))
                        {
                            frequency = Frequency_(flag);
                            valueChanged = true;
                        }
                    }
                    else
                    {
                        ImU64 value = Frequency_(frequency);
                        if (ImGui::CheckboxFlags(text.c_str(), &value, flag))
                        {
                            frequency = Frequency_(value);
                            valueChanged = true;
                        }
                    }
                    if (c >= 5)
                    {
                        ImGui::EndDisabled();
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

std::ostream& operator<<(std::ostream& os, const NAV::Frequency& obj)
{
    return os << fmt::format("{}", obj);
}