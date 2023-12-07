// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Errors.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

namespace NAV
{

double GnssMeasurementErrorModel::gnssMeasErrorVar(const SatelliteSystem& satSys, double elevation) const
{
    if (model == RTKLIB)
    {
        double ele = std::max(elevation, deg2rad(5));

        constexpr double EFACT_GPS = 1.0;  // Satellite system error factor GPS/GAL/QZS/BeiDou
        constexpr double EFACT_GLO = 1.5;  // Satellite system error factor GLONASS/IRNSS
        constexpr double EFACT_SBAS = 3.0; // Satellite system error factor SBAS
        double satSysErrFactor = satSys & (GLO | IRNSS)
                                     ? EFACT_GLO
                                     : (satSys == SBAS
                                            ? EFACT_SBAS
                                            : EFACT_GPS);

        return std::pow(satSysErrFactor, 2)
               * (std::pow(rtklibParams.carrierPhaseErrorAB[0], 2)
                  + std::pow(rtklibParams.carrierPhaseErrorAB[1], 2) / std::pow(std::sin(ele), 2));
    }
    return 0.0;
}

double GnssMeasurementErrorModel::psrMeasErrorVar(const SatelliteSystem& satSys, const Frequency& freq, double elevation) const
{
    double codeCarrierPhaseErrorRatio = 1.0;
    switch (static_cast<Frequency_>(freq))
    {
    case G01: // GPS L1 (1575.42 MHz).
    case E01: // Galileo, "E1" (1575.42 MHz).
    case R01: // GLONASS, "G1" (1602 MHZ).
    case B01: // Beidou B1 (1575.42 MHz).
    case J01: // QZSS L1 (1575.42 MHz).
    case S01: // SBAS L1 (1575.42 MHz).
        codeCarrierPhaseErrorRatio = rtklibParams.codeCarrierPhaseErrorRatio[0];
        break;
    case G02:       // GPS L2 (1227.6 MHz).
    case G05:       // GPS L5 (1176.45 MHz).
    case E05:       // Galileo E5a (1176.45 MHz).
    case E06:       // Galileo E6 (1278.75 MHz).
    case E07:       // Galileo E5b (1207.14 MHz).
    case E08:       // Galileo E5 (E5a + E5b) (1191.795MHz).
    case R02:       // GLONASS, "G2" (1246 MHz).
    case R03:       // GLONASS, "G3" (1202.025 MHz).
    case R04:       // GLONASS, "G1a" (1600.995 MHZ).
    case R06:       // GLONASS, "G2a" (1248.06 MHz).
    case B02:       // Beidou B1-2 (1561.098 MHz).
    case B05:       // Beidou B2a (1176.45 MHz).
    case B06:       // Beidou B3 (1268.52 MHz).
    case B07:       // Beidou B2b (1207.14 MHz).
    case B08:       // Beidou B2 (B2a + B2b) (1191.795MHz).
    case J02:       // QZSS L2 (1227.6 MHz).
    case J05:       // QZSS L5 (1176.45 MHz).
    case J06:       // QZSS L6 / LEX (1278.75 MHz).
    case I05:       // IRNSS L5 (1176.45 MHz).
    case I09:       // IRNSS S (2492.028 MHz).
    case S05:       // SBAS L5 (1176.45 MHz).
    case Freq_None: // None
        codeCarrierPhaseErrorRatio = rtklibParams.codeCarrierPhaseErrorRatio[1];
        break;
    }

    return std::pow(codeCarrierPhaseErrorRatio, 2) * gnssMeasErrorVar(satSys, elevation);
}

double GnssMeasurementErrorModel::carrierMeasErrorVar(const SatelliteSystem& satSys, double elevation) const
{
    return gnssMeasErrorVar(satSys, elevation);
}

double GnssMeasurementErrorModel::codeBiasErrorVar() const
{
    if (model == RTKLIB)
    {
        constexpr double ERR_CBIAS = 0.3; // Code bias error Std [m]

        return std::pow(ERR_CBIAS, 2);
    }
    return 0.0;
}

[[nodiscard]] double GnssMeasurementErrorModel::psrRateErrorVar(Frequency freq, int8_t num) const
{
    if (model == RTKLIB)
    {
        return std::pow(doppler2rangeRate(rtklibParams.dopplerFrequency, freq, num), 2);
    }
    return 0.0;
}

double GnssMeasurementErrorModel::dopplerErrorVar() const
{
    if (model == RTKLIB)
    {
        return std::pow(rtklibParams.dopplerFrequency, 2);
    }
    return 0.0;
}

double GnssMeasurementErrorModel::psrMeasErrorVar(double elevation, double cn0) const
{
    if (model == Groves2013)
    {
        return (1 / std::pow(std::sin(elevation), 2) * (std::pow(groves2013Params.sigmaRho.at(0), 2) + std::pow(groves2013Params.sigmaRho.at(1), 2) / cn0));
    }
    return 0.0;
}

double GnssMeasurementErrorModel::dopplerMeasErrorVar(double elevation, double cn0) const
{
    if (model == Groves2013)
    {
        return (1 / std::pow(std::sin(elevation), 2) * (std::pow(groves2013Params.sigmaR.at(0), 2) + std::pow(groves2013Params.sigmaR.at(1), 2) / cn0));
    }
    return 0.0;
}

bool GnssMeasurementErrorModel::ShowGuiWidgets(const char* id, float width)
{
    ImGui::SetNextItemWidth(width);
    bool changed = gui::widgets::EnumCombo(fmt::format("GNSS Measurement Error Model##", id).c_str(), model);

    if (model == GnssMeasurementErrorModel::RTKLIB)
    {
        ImGui::SetNextItemWidth(width);
        changed |= ImGui::InputDouble2(fmt::format("Code/Carrier-Phase Error Ratio L1/L2##", id).c_str(), rtklibParams.codeCarrierPhaseErrorRatio.data(), "%.1g");
        ImGui::SetNextItemWidth(width);
        changed |= ImGui::InputDouble2(fmt::format("Carrier-Phase Error a+b/sin(El)##", id).c_str(), rtklibParams.carrierPhaseErrorAB.data(), "%.3g m");
        ImGui::SetNextItemWidth(width);
        changed |= ImGui::InputDouble(fmt::format("Doppler Frequency##", id).c_str(), &(rtklibParams.dopplerFrequency), 0.0, 0.0, "%.3g Hz");
        ImGui::SameLine();
        ImGui::Text("= %.2g m/s (G1)", std::abs(doppler2rangeRate(rtklibParams.dopplerFrequency, G01)));
    }
    if (model == GnssMeasurementErrorModel::Groves2013)
    {
        ImGui::SetNextItemWidth(width);
        changed |= ImGui::InputDouble2(fmt::format("Pseudo-range error standard deviations (zenith | cn0-dependent)##", id).c_str(), groves2013Params.sigmaRho.data(), "%.3g m");
        ImGui::SetNextItemWidth(width);
        changed |= ImGui::InputDouble2(fmt::format("Pseudo-range rate error standard deviations (zenith | cn0-dependent)##", id).c_str(), groves2013Params.sigmaR.data(), "%.3g m");
    }
    return changed;
}

const char* to_string(GnssMeasurementErrorModel::Model model)
{
    switch (model)
    {
    case GnssMeasurementErrorModel::Model::None:
        return "None";
    case GnssMeasurementErrorModel::Model::RTKLIB:
        return "RTKLIB";
    case GnssMeasurementErrorModel::Model::Groves2013:
        return "Groves2013";
    case GnssMeasurementErrorModel::Model::COUNT:
        break;
    }
    return "";
}

void to_json(json& j, const GnssMeasurementErrorModel& obj)
{
    j = json{
        { "model", obj.model },
        { "rtklibParams", obj.rtklibParams },
        { "groves2013Params", obj.groves2013Params },
    };
}

void from_json(const json& j, GnssMeasurementErrorModel& obj)
{
    if (j.contains("model")) { j.at("model").get_to(obj.model); }
    if (j.contains("rtklibParams")) { j.at("rtklibParams").get_to(obj.rtklibParams); }
    if (j.contains("groves2013Params")) { j.at("groves2013Params").get_to(obj.groves2013Params); }
}

void to_json(json& j, const GnssMeasurementErrorModel::RtklibParameters& obj)
{
    j = json{
        { "codeCarrierPhaseErrorRatio", obj.codeCarrierPhaseErrorRatio },
        { "carrierPhaseErrorAB", obj.carrierPhaseErrorAB },
        { "dopplerFrequency", obj.dopplerFrequency },
    };
}

void from_json(const json& j, GnssMeasurementErrorModel::RtklibParameters& obj)
{
    if (j.contains("codeCarrierPhaseErrorRatio")) { j.at("codeCarrierPhaseErrorRatio").get_to(obj.codeCarrierPhaseErrorRatio); }
    if (j.contains("carrierPhaseErrorAB")) { j.at("carrierPhaseErrorAB").get_to(obj.carrierPhaseErrorAB); }
    if (j.contains("dopplerFrequency")) { j.at("dopplerFrequency").get_to(obj.dopplerFrequency); }
}

void to_json(json& j, const GnssMeasurementErrorModel::Groves2013Parameters& obj)
{
    j = json{
        { "sigmaRho", obj.sigmaRho },
        { "sigmaR", obj.sigmaR },
    };
}

void from_json(const json& j, GnssMeasurementErrorModel::Groves2013Parameters& obj)
{
    if (j.contains("sigmaRho")) { j.at("sigmaRho").get_to(obj.sigmaRho); }
    if (j.contains("sigmaR")) { j.at("sigmaR").get_to(obj.sigmaR); }
}

} // namespace NAV