// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Errors.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV
{

double gnssMeasErrorVar(const SatelliteSystem& satSys, const Frequency& freq, double elevation, bool pseudorangeMeasurement)
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

    double codeCarrierPhaseErrorRatio = 1.0;
    if (pseudorangeMeasurement)
    {
        switch (static_cast<Frequency_>(freq)) // TODO: This table needs to be adapted to get better result depending on frequency
        {
        case G01:       // GPS L1 (1575.42 MHz).
        case G02:       // GPS L2 (1227.6 MHz).
        case G05:       // GPS L5 (1176.45 MHz).
        case E01:       // Galileo, "E1" (1575.42 MHz).
        case E05:       // Galileo E5a (1176.45 MHz).
        case E06:       // Galileo E6 (1278.75 MHz).
        case E07:       // Galileo E5b (1207.14 MHz).
        case E08:       // Galileo E5 (E5a + E5b) (1191.795MHz).
        case R01:       // GLONASS, "G1" (1602 MHZ).
        case R02:       // GLONASS, "G2" (1246 MHz).
        case R03:       // GLONASS, "G3" (1202.025 MHz).
        case R04:       // GLONASS, "G1a" (1600.995 MHZ).
        case R06:       // GLONASS, "G2a" (1248.06 MHz).
        case B01:       // Beidou B1 (1575.42 MHz).
        case B02:       // Beidou B1-2 (1561.098 MHz).
        case B05:       // Beidou B2a (1176.45 MHz).
        case B06:       // Beidou B3 (1268.52 MHz).
        case B07:       // Beidou B2b (1207.14 MHz).
        case B08:       // Beidou B2 (B2a + B2b) (1191.795MHz).
        case J01:       // QZSS L1 (1575.42 MHz).
        case J02:       // QZSS L2 (1227.6 MHz).
        case J05:       // QZSS L5 (1176.45 MHz).
        case J06:       // QZSS L6 / LEX (1278.75 MHz).
        case I05:       // IRNSS L5 (1176.45 MHz).
        case I09:       // IRNSS S (2492.028 MHz).
        case S01:       // SBAS L1 (1575.42 MHz).
        case S05:       // SBAS L5 (1176.45 MHz).
        case Freq_None: // None
            codeCarrierPhaseErrorRatio = 300.0;
        }
    }

    double carrierPhaseErrorA = 0.003; // Carrier-Phase Error Factor a [m] - Measurement error standard deviation
    double carrierPhaseErrorB = 0.003; // Carrier-Phase Error Factor b [m] - Measurement error standard deviation

    return std::pow(satSysErrFactor, 2) * std::pow(codeCarrierPhaseErrorRatio, 2)
           * (std::pow(carrierPhaseErrorA, 2) + std::pow(carrierPhaseErrorB, 2) / std::sin(ele));
}

double psrMeasErrorVar(const SatelliteSystem& satSys, const Frequency& freq, double elevation)
{
    return gnssMeasErrorVar(satSys, freq, elevation, true);
}

double carrierMeasErrorVar(const SatelliteSystem& satSys, const Frequency& freq, double elevation)
{
    return gnssMeasErrorVar(satSys, freq, elevation, false);
}

double codeBiasErrorVar()
{
    constexpr double ERR_CBIAS = 0.3; // Code bias error Std [m]

    return std::pow(ERR_CBIAS, 2);
}

double dopplerErrorVar()
{
    double dopplerFrequency = 1; // Doppler Frequency error factor [Hz] - Measurement error standard deviation

    return std::pow(dopplerFrequency, 2);
}

} // namespace NAV