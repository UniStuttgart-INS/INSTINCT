// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AntexReaderTests.cpp
/// @brief Tests for the Antex reader
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-12-05

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Navigation/GNSS/Positioning/AntexReader.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

namespace NAV::TESTS::AntexReaderTests
{

namespace
{

void testPCO(const std::string& antennaType, Frequency_ freq, const InsTime& insTime, std::optional<Eigen::Vector3d> expected)
{
    CAPTURE(antennaType);
    CAPTURE(std::string(Frequency(freq)));
    CAPTURE(insTime.toYMDHMS(GPST));

    auto pco2arp = NAV::AntexReader::Get().getAntennaPhaseCenterOffsetToARP(antennaType, freq, insTime, "AntexReader");
    if (!expected.has_value())
    {
        CAPTURE(expected.has_value());
        if (NAV::AntexReader::Get()._antennas.contains(antennaType))
        {
            CAPTURE(NAV::AntexReader::Get()._antennas.at(antennaType).serialNumber);

            if (auto antInfo = NAV::AntexReader::Get().getAntennaInfo(antennaType, insTime, "AntexReader"))
            {
                CAPTURE(antInfo->get().from.toYMDHMS(GPST));
                CAPTURE(antInfo->get().until.toYMDHMS(GPST));
                CAPTURE(antInfo->get().date.toYMDHMS(GPST));

                std::string frequencies;
                for (const auto& freqInfo : antInfo->get().freqInformation)
                {
                    frequencies += std::string(Frequency(freqInfo.first)) + ", ";
                }
                frequencies = frequencies.substr(0, frequencies.length() - 2);
                CAPTURE(frequencies);
                if (pco2arp.has_value())
                {
                    CAPTURE(pco2arp.value().transpose());
                    CHECK(!pco2arp.has_value());
                    return;
                }
                CHECK(!pco2arp.has_value());
                return;
            }
            CHECK(!pco2arp.has_value());
            return;
        }
        CHECK(!pco2arp.has_value());
        return;
    }
    expected.value() *= 1e-3;

    if (!pco2arp.has_value())
    {
        REQUIRE(NAV::AntexReader::Get()._antennas.contains(antennaType));
        CAPTURE(NAV::AntexReader::Get()._antennas.at(antennaType).serialNumber);

        auto antInfo = NAV::AntexReader::Get().getAntennaInfo(antennaType, insTime, "AntexReader");
        REQUIRE(antInfo.has_value());

        CAPTURE(antInfo->get().from.toYMDHMS(GPST));
        CAPTURE(antInfo->get().until.toYMDHMS(GPST));
        CAPTURE(antInfo->get().date.toYMDHMS(GPST));

        std::string frequencies;
        for (const auto& freqInfo : antInfo->get().freqInformation)
        {
            frequencies += std::string(Frequency(freqInfo.first)) + ", ";
        }
        frequencies = frequencies.substr(0, frequencies.length() - 2);
        CAPTURE(frequencies);

        REQUIRE(pco2arp.has_value());
    }

    REQUIRE(pco2arp->transpose() == expected->transpose());
}

void testPCV(const std::string& antennaType, Frequency_ freq, const InsTime& insTime, double zenith, std::optional<double> azimuth, std::optional<double> expected)
{
    CAPTURE(antennaType);
    CAPTURE(std::string(Frequency(freq)));
    CAPTURE(insTime.toYMDHMS(GPST));
    CAPTURE(zenith);
    LOG_INFO("\nzenith = {:.1f}°{}", zenith, azimuth ? fmt::format(", azimuth = {:.1f}°", *azimuth) : "");

    if (azimuth) { azimuth = deg2rad(*azimuth); }

    auto antFreqInfo = NAV::AntexReader::Get().getAntennaFrequencyInfo(antennaType, freq, insTime, "AntexReader");
    if (azimuth.has_value())
    {
        Eigen::MatrixXd pattern = antFreqInfo.value().get().pattern;
        std::transform(pattern.row(0).cbegin(), pattern.row(0).cend(), pattern.row(0).begin(), [](double v) { return rad2deg(v); });
        std::transform(pattern.col(0).cbegin(), pattern.col(0).cend(), pattern.col(0).begin(), [](double v) { return rad2deg(v); });
        LOG_INFO("pattern =\n{}", pattern);
    }
    else
    {
        Eigen::MatrixXd pattern = antFreqInfo.value().get().patternAzimuthIndependent;
        std::transform(pattern.row(0).cbegin(), pattern.row(0).cend(), pattern.row(0).begin(), [](double v) { return rad2deg(v); });
        LOG_INFO("patternAzimuthIndependent =\n{}", pattern);
    }

    double elevation = deg2rad(90.0 - zenith);
    auto variation = NAV::AntexReader::Get().getAntennaPhaseCenterVariation(antennaType, freq, insTime, elevation, azimuth, "AntexReader");
    if (!expected.has_value())
    {
        CHECK(!variation.has_value());
        return;
    }
    expected.value() *= 1e-3;
    LOG_INFO("expected = {:.5f}", *expected);

    if (azimuth.has_value())
    {
        CAPTURE(rad2deg(azimuth.value()));
        CHECK_THAT(variation.value(), Catch::Matchers::WithinAbs(expected.value(), 1e-6));
        return;
    }
    CHECK_THAT(variation.value(), Catch::Matchers::WithinAbs(expected.value(), 1e-6));
}

} // namespace

TEST_CASE("[AntexReader] Get antenna phase center offset to ARP", "[AntexReader]")
{
    auto logger = initializeTestLogger();
    NAV::AntexReader::Get().initialize();

    testPCO("CNTT300         NON", G01, {}, std::nullopt); // Antenna not in ANTEX files

    testPCO("TRM59800.00     SCIS", G01, {}, Eigen::Vector3d(0.69, 0.16, 86.63));
    testPCO("CNTT300         NONE", G01, {}, Eigen::Vector3d(2.08, 1.36, 75.26));
    testPCO("CNTT300         NONE", I09, {}, std::nullopt); // Frequency not in ANTEX files

    // has only `VALID FROM` not `VALID UNTIL`
    testPCO("IRNSS-1GEO:I03", I05, InsTime(2014, 10, 14, 23, 59, 59.9, GPST), Eigen::Vector3d(11.4, 1.1, 1280.8)); // before '2014-10-15 0:0:0.0 VALID FROM', we take the first entry
    testPCO("IRNSS-1GEO:I03", I05, InsTime(2014, 10, 14, 0, 0, 0.0, GPST), Eigen::Vector3d(11.4, 1.1, 1280.8));    // exactly '2014-10-15 0:0:0.0 VALID FROM'

    // has `VALID FROM` and `VALID UNTIL`
    // 1992    11    22     0     0    0.0 to 2008    10    16    23    59   59.9999999
    // 2008    10    23     0     0    0.0 to 2009     1     6    23    59   59.9999999
    // 2011     6     2     0     0    0.0 to 2011     7    12    23    59   59.9999999
    testPCO("BLOCK IIA:G01", G01, InsTime(2008, 10, 16, 23, 59, 59.9999999, GPST), Eigen::Vector3d(279.0, 0.0, 2253.47));
    testPCO("BLOCK IIA:G01", G02, InsTime(2008, 10, 16, 23, 59, 59.9999999, GPST), Eigen::Vector3d(279.0, 0.0, 2253.47));
    testPCO("BLOCK IIA:G01", G01, InsTime(2008, 10, 23, 0, 0, 0.0, GPST), Eigen::Vector3d(279.0, 0.0, 2220.62));
    testPCO("BLOCK IIA:G01", G01, InsTime(2022, 10, 23, 0, 0, 0.0, GPST), Eigen::Vector3d(279.0, 0.0, 2512.76)); // after last 'VALID UNTIL', we take last entry
}

TEST_CASE("[AntexReader] Antenna phase center variation pattern (no azimuth)", "[AntexReader]")
{
    auto logger = initializeTestLogger();
    NAV::AntexReader::Get().initialize();

    Eigen::Matrix<double, 2, 19> expected;
    // clang-format off
    expected <<
            0,       5,      10,      15,      20,      25,      30,      35,      40,      45,      50,      55,      60,      65,      70,      75,      80,      85,      90,
         0.00,   -0.31,   -1.18,   -2.52,   -4.15,   -5.87,   -7.46,   -8.74,   -9.57,   -9.86,   -9.58,   -8.74,   -7.35,   -5.39,   -2.77,    0.68,    5.13,   10.59,   16.79;
    // clang-format on
    Eigen::Matrix2Xd pattern = NAV::AntexReader::Get()._antennas.at("TRM59800.00     SCIS").antennaInfo.front().freqInformation.at(G01).patternAzimuthIndependent;
    pattern.row(0) *= 180.0 / std::numbers::pi_v<double>;
    pattern.row(1) *= 1e3;
    LOG_INFO("expected =\n{}", expected);
    LOG_INFO("read =\n{}", pattern);
    REQUIRE_THAT(pattern, Catch::Matchers::WithinAbs(expected, 1e-6));
}

TEST_CASE("[AntexReader] Antenna phase center variation pattern", "[AntexReader]")
{
    auto logger = initializeTestLogger();
    NAV::AntexReader::Get().initialize();

    Eigen::Matrix<double, 74, 20> expected;
    // clang-format off
    expected <<
            0,       0,       5,      10,      15,      20,      25,      30,      35,      40,      45,      50,      55,      60,      65,      70,      75,      80,      85,      90,
          0.0,    0.00,   -0.29,   -1.16,   -2.50,   -4.15,   -5.89,   -7.51,   -8.80,   -9.62,   -9.88,   -9.57,   -8.71,   -7.33,   -5.41,   -2.85,    0.50,    4.75,    9.88,   15.56,
          5.0,    0.00,   -0.29,   -1.16,   -2.51,   -4.16,   -5.91,   -7.53,   -8.83,   -9.65,   -9.91,   -9.61,   -8.75,   -7.36,   -5.43,   -2.86,    0.48,    4.72,    9.83,   15.49,
         10.0,    0.00,   -0.29,   -1.16,   -2.52,   -4.17,   -5.93,   -7.55,   -8.85,   -9.68,   -9.95,   -9.65,   -8.79,   -7.40,   -5.45,   -2.87,    0.48,    4.72,    9.83,   15.49,
         15.0,    0.00,   -0.29,   -1.16,   -2.52,   -4.18,   -5.94,   -7.57,   -8.87,   -9.71,   -9.98,   -9.69,   -8.83,   -7.43,   -5.47,   -2.87,    0.51,    4.76,    9.87,   15.55,
         20.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.19,   -5.95,   -7.58,   -8.89,   -9.73,  -10.02,   -9.73,   -8.87,   -7.46,   -5.48,   -2.85,    0.55,    4.82,    9.96,   15.67,
         25.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.20,   -5.96,   -7.59,   -8.90,   -9.74,  -10.04,   -9.76,   -8.91,   -7.50,   -5.49,   -2.83,    0.61,    4.92,   10.09,   15.85,
         30.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.20,   -5.97,   -7.60,   -8.90,   -9.75,  -10.06,   -9.79,   -8.95,   -7.53,   -5.51,   -2.81,    0.67,    5.04,   10.26,   16.08,
         35.0,    0.00,   -0.29,   -1.17,   -2.54,   -4.21,   -5.97,   -7.59,   -8.90,   -9.75,  -10.07,   -9.81,   -8.98,   -7.56,   -5.52,   -2.78,    0.75,    5.17,   10.46,   16.34,
         40.0,    0.00,   -0.29,   -1.17,   -2.54,   -4.21,   -5.96,   -7.58,   -8.89,   -9.74,  -10.07,   -9.83,   -9.00,   -7.58,   -5.53,   -2.77,    0.81,    5.30,   10.66,   16.62,
         45.0,    0.00,   -0.29,   -1.17,   -2.54,   -4.21,   -5.96,   -7.57,   -8.87,   -9.73,  -10.06,   -9.83,   -9.01,   -7.60,   -5.54,   -2.75,    0.87,    5.42,   10.86,   16.91,
         50.0,    0.00,   -0.29,   -1.17,   -2.54,   -4.20,   -5.95,   -7.56,   -8.85,   -9.70,  -10.03,   -9.81,   -9.02,   -7.62,   -5.56,   -2.75,    0.91,    5.52,   11.05,   17.20,
         55.0,    0.00,   -0.29,   -1.17,   -2.54,   -4.20,   -5.94,   -7.54,   -8.82,   -9.67,  -10.00,   -9.79,   -9.01,   -7.62,   -5.57,   -2.76,    0.92,    5.59,   11.20,   17.46,
         60.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.19,   -5.92,   -7.52,   -8.79,   -9.63,   -9.97,   -9.76,   -8.99,   -7.62,   -5.59,   -2.78,    0.92,    5.63,   11.32,   17.68,
         65.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.19,   -5.91,   -7.50,   -8.76,   -9.60,   -9.92,   -9.72,   -8.96,   -7.61,   -5.59,   -2.81,    0.89,    5.64,   11.40,   17.86,
         70.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.18,   -5.90,   -7.48,   -8.74,   -9.56,   -9.88,   -9.67,   -8.92,   -7.58,   -5.60,   -2.84,    0.86,    5.62,   11.44,   17.99,
         75.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.17,   -5.89,   -7.46,   -8.71,   -9.52,   -9.84,   -9.62,   -8.87,   -7.56,   -5.60,   -2.86,    0.82,    5.59,   11.44,   18.05,
         80.0,    0.00,   -0.29,   -1.17,   -2.53,   -4.17,   -5.88,   -7.45,   -8.69,   -9.49,   -9.80,   -9.58,   -8.83,   -7.52,   -5.58,   -2.87,    0.78,    5.54,   11.40,   18.06,
         85.0,    0.00,   -0.29,   -1.17,   -2.52,   -4.16,   -5.87,   -7.44,   -8.67,   -9.47,   -9.76,   -9.53,   -8.78,   -7.48,   -5.56,   -2.87,    0.76,    5.49,   11.35,   18.01,
         90.0,    0.00,   -0.29,   -1.17,   -2.52,   -4.16,   -5.87,   -7.43,   -8.66,   -9.45,   -9.73,   -9.50,   -8.74,   -7.44,   -5.53,   -2.86,    0.75,    5.45,   11.28,   17.91,
         95.0,    0.00,   -0.29,   -1.17,   -2.52,   -4.16,   -5.86,   -7.42,   -8.65,   -9.43,   -9.71,   -9.47,   -8.70,   -7.40,   -5.48,   -2.82,    0.76,    5.43,   11.20,   17.78,
        100.0,    0.00,   -0.29,   -1.17,   -2.52,   -4.16,   -5.86,   -7.42,   -8.65,   -9.43,   -9.70,   -9.44,   -8.67,   -7.35,   -5.43,   -2.78,    0.79,    5.42,   11.13,   17.63,
        105.0,    0.00,   -0.30,   -1.17,   -2.52,   -4.15,   -5.86,   -7.42,   -8.64,   -9.42,   -9.69,   -9.43,   -8.64,   -7.31,   -5.38,   -2.72,    0.84,    5.42,   11.06,   17.48,
        110.0,    0.00,   -0.30,   -1.17,   -2.52,   -4.15,   -5.86,   -7.42,   -8.65,   -9.43,   -9.69,   -9.42,   -8.62,   -7.28,   -5.32,   -2.65,    0.89,    5.44,   11.00,   17.32,
        115.0,    0.00,   -0.30,   -1.18,   -2.52,   -4.15,   -5.86,   -7.42,   -8.65,   -9.43,   -9.70,   -9.42,   -8.61,   -7.25,   -5.27,   -2.59,    0.95,    5.46,   10.95,   17.19,
        120.0,    0.00,   -0.30,   -1.18,   -2.52,   -4.15,   -5.86,   -7.42,   -8.66,   -9.44,   -9.70,   -9.43,   -8.60,   -7.22,   -5.23,   -2.54,    0.99,    5.47,   10.90,   17.08,
        125.0,    0.00,   -0.30,   -1.18,   -2.52,   -4.16,   -5.86,   -7.43,   -8.67,   -9.45,   -9.71,   -9.43,   -8.60,   -7.20,   -5.20,   -2.50,    1.02,    5.47,   10.85,   16.99,
        130.0,    0.00,   -0.30,   -1.18,   -2.53,   -4.16,   -5.87,   -7.43,   -8.67,   -9.46,   -9.72,   -9.44,   -8.60,   -7.20,   -5.19,   -2.48,    1.03,    5.45,   10.80,   16.93,
        135.0,    0.00,   -0.30,   -1.18,   -2.53,   -4.16,   -5.87,   -7.44,   -8.68,   -9.47,   -9.73,   -9.45,   -8.60,   -7.20,   -5.19,   -2.49,    1.00,    5.41,   10.75,   16.88,
        140.0,    0.00,   -0.31,   -1.19,   -2.53,   -4.16,   -5.87,   -7.44,   -8.69,   -9.48,   -9.74,   -9.46,   -8.61,   -7.21,   -5.21,   -2.52,    0.95,    5.34,   10.68,   16.84,
        145.0,    0.00,   -0.31,   -1.19,   -2.53,   -4.17,   -5.88,   -7.45,   -8.70,   -9.49,   -9.75,   -9.47,   -8.62,   -7.23,   -5.24,   -2.58,    0.88,    5.26,   10.60,   16.80,
        150.0,    0.00,   -0.31,   -1.19,   -2.54,   -4.17,   -5.88,   -7.45,   -8.70,   -9.50,   -9.77,   -9.48,   -8.64,   -7.26,   -5.29,   -2.65,    0.79,    5.16,   10.51,   16.75,
        155.0,    0.00,   -0.31,   -1.20,   -2.54,   -4.17,   -5.88,   -7.46,   -8.71,   -9.51,   -9.78,   -9.49,   -8.66,   -7.29,   -5.34,   -2.73,    0.68,    5.05,   10.42,   16.69,
        160.0,    0.00,   -0.32,   -1.20,   -2.55,   -4.18,   -5.89,   -7.46,   -8.72,   -9.52,   -9.79,   -9.51,   -8.69,   -7.33,   -5.40,   -2.81,    0.58,    4.94,   10.32,   16.62,
        165.0,    0.00,   -0.32,   -1.20,   -2.55,   -4.18,   -5.89,   -7.47,   -8.73,   -9.54,   -9.81,   -9.53,   -8.71,   -7.36,   -5.46,   -2.89,    0.48,    4.83,   10.23,   16.53,
        170.0,    0.00,   -0.32,   -1.21,   -2.55,   -4.18,   -5.90,   -7.48,   -8.74,   -9.55,   -9.83,   -9.55,   -8.74,   -7.40,   -5.51,   -2.96,    0.40,    4.75,   10.15,   16.45,
        175.0,    0.00,   -0.32,   -1.21,   -2.56,   -4.19,   -5.90,   -7.49,   -8.75,   -9.57,   -9.85,   -9.57,   -8.76,   -7.43,   -5.55,   -3.02,    0.33,    4.68,   10.08,   16.36,
        180.0,    0.00,   -0.32,   -1.21,   -2.56,   -4.19,   -5.90,   -7.49,   -8.77,   -9.59,   -9.87,   -9.59,   -8.78,   -7.45,   -5.57,   -3.06,    0.28,    4.63,   10.04,   16.30,
        185.0,    0.00,   -0.32,   -1.22,   -2.56,   -4.19,   -5.91,   -7.50,   -8.78,   -9.60,   -9.89,   -9.61,   -8.79,   -7.46,   -5.59,   -3.08,    0.25,    4.60,   10.02,   16.27,
        190.0,    0.00,   -0.33,   -1.22,   -2.57,   -4.20,   -5.91,   -7.51,   -8.79,   -9.62,   -9.91,   -9.62,   -8.80,   -7.46,   -5.59,   -3.09,    0.24,    4.59,   10.02,   16.28,
        195.0,    0.00,   -0.33,   -1.22,   -2.57,   -4.20,   -5.91,   -7.51,   -8.80,   -9.63,   -9.92,   -9.63,   -8.80,   -7.45,   -5.58,   -3.08,    0.24,    4.59,   10.05,   16.33,
        200.0,    0.00,   -0.33,   -1.22,   -2.57,   -4.20,   -5.91,   -7.51,   -8.81,   -9.64,   -9.93,   -9.63,   -8.79,   -7.43,   -5.56,   -3.07,    0.24,    4.61,   10.10,   16.43,
        205.0,    0.00,   -0.33,   -1.23,   -2.57,   -4.19,   -5.91,   -7.51,   -8.81,   -9.65,   -9.93,   -9.63,   -8.77,   -7.41,   -5.54,   -3.06,    0.26,    4.64,   10.17,   16.56,
        210.0,    0.00,   -0.33,   -1.23,   -2.57,   -4.19,   -5.90,   -7.50,   -8.80,   -9.65,   -9.93,   -9.62,   -8.75,   -7.39,   -5.52,   -3.04,    0.27,    4.67,   10.26,   16.71,
        215.0,    0.00,   -0.33,   -1.23,   -2.57,   -4.19,   -5.89,   -7.49,   -8.80,   -9.64,   -9.92,   -9.61,   -8.73,   -7.36,   -5.49,   -3.02,    0.29,    4.71,   10.35,   16.87,
        220.0,    0.00,   -0.33,   -1.23,   -2.56,   -4.18,   -5.88,   -7.48,   -8.78,   -9.62,   -9.90,   -9.59,   -8.72,   -7.34,   -5.48,   -3.00,    0.31,    4.76,   10.44,   17.02,
        225.0,    0.00,   -0.33,   -1.23,   -2.56,   -4.17,   -5.87,   -7.46,   -8.76,   -9.61,   -9.89,   -9.58,   -8.71,   -7.33,   -5.46,   -2.99,    0.34,    4.80,   10.52,   17.15,
        230.0,    0.00,   -0.33,   -1.22,   -2.56,   -4.16,   -5.86,   -7.44,   -8.74,   -9.59,   -9.87,   -9.57,   -8.70,   -7.33,   -5.45,   -2.97,    0.37,    4.86,   10.60,   17.23,
        235.0,    0.00,   -0.33,   -1.22,   -2.55,   -4.15,   -5.84,   -7.42,   -8.72,   -9.57,   -9.86,   -9.57,   -8.70,   -7.33,   -5.45,   -2.95,    0.41,    4.92,   10.66,   17.27,
        240.0,    0.00,   -0.33,   -1.22,   -2.55,   -4.14,   -5.82,   -7.40,   -8.70,   -9.55,   -9.85,   -9.57,   -8.71,   -7.34,   -5.45,   -2.92,    0.46,    4.98,   10.70,   17.26,
        245.0,    0.00,   -0.33,   -1.22,   -2.54,   -4.13,   -5.81,   -7.38,   -8.68,   -9.53,   -9.85,   -9.57,   -8.73,   -7.35,   -5.44,   -2.89,    0.52,    5.04,   10.73,   17.20,
        250.0,    0.00,   -0.33,   -1.21,   -2.53,   -4.12,   -5.80,   -7.37,   -8.66,   -9.53,   -9.85,   -9.59,   -8.75,   -7.37,   -5.44,   -2.86,    0.58,    5.10,   10.74,   17.10,
        255.0,    0.00,   -0.33,   -1.21,   -2.53,   -4.11,   -5.79,   -7.36,   -8.65,   -9.52,   -9.86,   -9.60,   -8.77,   -7.38,   -5.43,   -2.82,    0.64,    5.15,   10.73,   16.99,
        260.0,    0.00,   -0.32,   -1.21,   -2.52,   -4.11,   -5.78,   -7.35,   -8.65,   -9.53,   -9.87,   -9.62,   -8.78,   -7.38,   -5.41,   -2.77,    0.70,    5.20,   10.72,   16.87,
        265.0,    0.00,   -0.32,   -1.20,   -2.52,   -4.10,   -5.78,   -7.35,   -8.66,   -9.54,   -9.88,   -9.63,   -8.79,   -7.37,   -5.38,   -2.72,    0.76,    5.23,   10.70,   16.77,
        270.0,    0.00,   -0.32,   -1.20,   -2.51,   -4.10,   -5.78,   -7.36,   -8.67,   -9.55,   -9.89,   -9.64,   -8.79,   -7.36,   -5.34,   -2.67,    0.81,    5.26,   10.68,   16.69,
        275.0,    0.00,   -0.32,   -1.20,   -2.51,   -4.10,   -5.78,   -7.37,   -8.68,   -9.57,   -9.91,   -9.64,   -8.78,   -7.33,   -5.30,   -2.63,    0.85,    5.27,   10.65,   16.66,
        280.0,    0.00,   -0.32,   -1.19,   -2.50,   -4.10,   -5.78,   -7.38,   -8.70,   -9.58,   -9.91,   -9.64,   -8.76,   -7.30,   -5.26,   -2.59,    0.87,    5.27,   10.64,   16.66,
        285.0,    0.00,   -0.31,   -1.19,   -2.50,   -4.10,   -5.79,   -7.39,   -8.71,   -9.59,   -9.92,   -9.63,   -8.73,   -7.26,   -5.22,   -2.55,    0.88,    5.26,   10.63,   16.71,
        290.0,    0.00,   -0.31,   -1.18,   -2.50,   -4.10,   -5.80,   -7.40,   -8.73,   -9.60,   -9.91,   -9.61,   -8.69,   -7.21,   -5.18,   -2.53,    0.89,    5.25,   10.63,   16.78,
        295.0,    0.00,   -0.31,   -1.18,   -2.49,   -4.10,   -5.80,   -7.41,   -8.74,   -9.61,   -9.90,   -9.58,   -8.66,   -7.17,   -5.14,   -2.51,    0.88,    5.23,   10.63,   16.86,
        300.0,    0.00,   -0.31,   -1.18,   -2.49,   -4.10,   -5.81,   -7.42,   -8.74,   -9.60,   -9.89,   -9.55,   -8.62,   -7.13,   -5.12,   -2.51,    0.87,    5.22,   10.64,   16.95,
        305.0,    0.00,   -0.31,   -1.17,   -2.49,   -4.10,   -5.81,   -7.42,   -8.74,   -9.60,   -9.87,   -9.52,   -8.58,   -7.10,   -5.10,   -2.51,    0.85,    5.20,   10.65,   17.01,
        310.0,    0.00,   -0.30,   -1.17,   -2.48,   -4.10,   -5.81,   -7.43,   -8.74,   -9.59,   -9.85,   -9.50,   -8.56,   -7.08,   -5.10,   -2.52,    0.84,    5.19,   10.65,   17.04,
        315.0,    0.00,   -0.30,   -1.17,   -2.48,   -4.10,   -5.81,   -7.43,   -8.73,   -9.57,   -9.83,   -9.47,   -8.54,   -7.08,   -5.10,   -2.54,    0.82,    5.17,   10.64,   17.03,
        320.0,    0.00,   -0.30,   -1.16,   -2.48,   -4.10,   -5.81,   -7.42,   -8.73,   -9.56,   -9.81,   -9.45,   -8.53,   -7.08,   -5.12,   -2.56,    0.79,    5.15,   10.61,   16.96,
        325.0,    0.00,   -0.30,   -1.16,   -2.48,   -4.10,   -5.82,   -7.42,   -8.72,   -9.54,   -9.79,   -9.44,   -8.53,   -7.10,   -5.15,   -2.59,    0.77,    5.13,   10.56,   16.84,
        330.0,    0.00,   -0.30,   -1.16,   -2.48,   -4.10,   -5.82,   -7.43,   -8.72,   -9.53,   -9.78,   -9.44,   -8.54,   -7.12,   -5.18,   -2.63,    0.73,    5.09,   10.49,   16.68,
        335.0,    0.00,   -0.29,   -1.16,   -2.48,   -4.10,   -5.82,   -7.43,   -8.72,   -9.53,   -9.78,   -9.44,   -8.55,   -7.15,   -5.22,   -2.67,    0.70,    5.04,   10.40,   16.48,
        340.0,    0.00,   -0.29,   -1.16,   -2.48,   -4.11,   -5.83,   -7.44,   -8.72,   -9.53,   -9.78,   -9.45,   -8.58,   -7.18,   -5.26,   -2.71,    0.66,    4.99,   10.29,   16.27,
        345.0,    0.00,   -0.29,   -1.16,   -2.49,   -4.12,   -5.84,   -7.45,   -8.73,   -9.55,   -9.80,   -9.47,   -8.60,   -7.22,   -5.30,   -2.75,    0.61,    4.92,   10.18,   16.05,
        350.0,    0.00,   -0.29,   -1.16,   -2.49,   -4.13,   -5.86,   -7.47,   -8.75,   -9.56,   -9.82,   -9.50,   -8.64,   -7.26,   -5.34,   -2.79,    0.57,    4.86,   10.06,   15.85,
        355.0,    0.00,   -0.29,   -1.16,   -2.50,   -4.14,   -5.88,   -7.49,   -8.77,   -9.59,   -9.84,   -9.53,   -8.67,   -7.29,   -5.38,   -2.82,    0.53,    4.79,    9.96,   15.68,
        360.0,    0.00,   -0.29,   -1.16,   -2.50,   -4.15,   -5.89,   -7.51,   -8.80,   -9.62,   -9.88,   -9.57,   -8.71,   -7.33,   -5.41,   -2.85,    0.50,    4.75,    9.88,   15.56;
    // clang-format on
    Eigen::MatrixXd pattern = NAV::AntexReader::Get()._antennas.at("TRM59800.00     SCIS").antennaInfo.front().freqInformation.at(G01).pattern;
    pattern.row(0) *= 180.0 / std::numbers::pi_v<double>;
    pattern.col(0) *= 180.0 / std::numbers::pi_v<double>;
    pattern.block(1, 1, pattern.rows() - 1, pattern.cols() - 1) *= 1e3;
    LOG_INFO("expected =\n{}", expected);
    LOG_INFO("read =\n{}", pattern);
    REQUIRE_THAT(pattern, Catch::Matchers::WithinAbs(expected, 1e-6));
}

TEST_CASE("[AntexReader] Get antenna phase center variation (no azimuth)", "[AntexReader]")
{
    auto logger = initializeTestLogger();
    NAV::AntexReader::Get().initialize();

    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 0.0, std::nullopt, 0.0);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 5.0, std::nullopt, -0.31);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 90.0, std::nullopt, 16.79);

    testPCV("TRM59800.00     SCIS", G01, InsTime{}, -1e-7, std::nullopt, std::nullopt);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 90.0 + 1e-7, std::nullopt, std::nullopt);

    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 12.5, std::nullopt, -1.85);
}

TEST_CASE("[AntexReader] Get antenna phase center variation", "[AntexReader]")
{
    auto logger = initializeTestLogger();
    NAV::AntexReader::Get().initialize();

    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 0.0, 0.0, 0.0);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 90.0, 0.0, 15.56);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 0.0, 360.0, 0.0);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 90.0, 360.0, 15.56);

    testPCV("TRM59800.00     SCIS", G01, InsTime{}, -1e-7, 0.0, std::nullopt);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 0.0, -1e-7, std::nullopt);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 90.0 + 1e-7, 0.0, std::nullopt);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 90.0, 360.0 + 1e-7, std::nullopt);

    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 5.0, 0.0, -0.29);
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 85.0, 90.0, 11.28);

    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 80.0 + 2.5, 350.0, (10.06 + 4.86) / 2.0); // same as lerp
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 80.0, 350.0 + 2.5, (4.86 + 4.79) / 2.0);  // same as lerp
    testPCV("TRM59800.00     SCIS", G01, InsTime{}, 80.0 + 2.5, 350.0 + 2.5, 7.4175);
}

TEST_CASE("[AntexReader] List Antennas", "[AntexReader]")
{
    auto logger = initializeTestLogger();
    NAV::AntexReader::Get().initialize();

    std::vector<std::string> mustHaveAntennas = {
        "3S-02-TSADM     NONE",
        "3S-02-TSATE     NONE",
        "AERAT1675_120   SPKE",
        "AERAT1675_542E  NEVE",
        "AERAT2775_43    NONE",
        "AERAT2775_43    SPKE",
        "AOAD/M_B        NONE",
        "AOAD/M_T        DUTD",
        "AOAD/M_T        NONE",
        "AOAD/M_TA_NGS   NONE",
        "AOAD/M_T_RFI_T  NONE",
        "AOAD/M_T_RFI_T  SCIS",
        "APSAPS-3        NONE",
        "ARFAS13DFS      ARFS",
        "ARFAS1FS        ARFC",
        "ASH700228A      NONE",
        "ASH700228B      NONE",
        "ASH700228C      NONE",
        "ASH700228D      NONE",
        "ASH700228E      NONE",
        "ASH700699.L1    NONE",
        "ASH700700.A     NONE",
        "ASH700700.B     NONE",
        "ASH700700.C     NONE",
        "ASH700718A      NONE",
        "ASH700718B      NONE",
        "ASH700829.2     SNOW",
        "ASH700829.3     SNOW",
        "ASH700829.A     SNOW",
        "ASH700829.A1    SNOW",
        "ASH700936A_M    NONE",
        "ASH700936A_M    SNOW",
        "ASH700936B_M    NONE",
        "ASH700936B_M    SNOW",
        "ASH700936C_M    NONE",
        "ASH700936C_M    SNOW",
        "ASH700936D_M    NONE",
        "ASH700936D_M    SCIS",
        "ASH700936D_M    SNOW",
        "ASH700936E      NONE",
        "ASH700936E      SCIS",
        "ASH700936E      SNOW",
        "ASH700936E_C    NONE",
        "ASH700936E_C    SNOW",
        "ASH700936F_C    NONE",
        "ASH700936F_C    SNOW",
        "ASH701008.01B   NONE",
        "ASH701023.A     NONE",
        "ASH701073.1     NONE",
        "ASH701073.1     SCIS",
        "ASH701073.1     SNOW",
        "ASH701073.3     NONE",
        "ASH701933A_M    NONE",
        "ASH701933A_M    SNOW",
        "ASH701933B_M    NONE",
        "ASH701933B_M    SNOW",
        "ASH701933C_M    NONE",
        "ASH701933C_M    SCIS",
        "ASH701933C_M    SCIT",
        "ASH701933C_M    SNOW",
        "ASH701941.1     NONE",
        "ASH701941.2     NONE",
        "ASH701941.A     NONE",
        "ASH701941.B     NONE",
        "ASH701941.B     SCIS",
        "ASH701945B.99   NONE",
        "ASH701945B.99   SCIS",
        "ASH701945B.99   SCIT",
        "ASH701945B_M    NONE",
        "ASH701945B_M    SCIS",
        "ASH701945B_M    SCIT",
        "ASH701945B_M    SNOW",
        "ASH701945C_M    NONE",
        "ASH701945C_M    OLGA",
        "ASH701945C_M    PFAN",
        "ASH701945C_M    SCIS",
        "ASH701945C_M    SCIT",
        "ASH701945C_M    SNOW",
        "ASH701945D_M    NONE",
        "ASH701945D_M    SCIS",
        "ASH701945D_M    SCIT",
        "ASH701945D_M    SNOW",
        "ASH701945E_M    NONE",
        "ASH701945E_M    SCIS",
        "ASH701945E_M    SCIT",
        "ASH701945E_M    SNOW",
        "ASH701945G_M    NONE",
        "ASH701945G_M    SCIS",
        "ASH701945G_M    SCIT",
        "ASH701945G_M    SNOW",
        "ASH701946.2     NONE",
        "ASH701946.2     SNOW",
        "ASH701946.3     NONE",
        "ASH701946.3     SNOW",
        "ASH701975.01A   NONE",
        "ASH701975.01AGP NONE",
        "BEIDOU-2G:C01",
        "BEIDOU-2G:C02",
        "BEIDOU-2G:C03",
        "BEIDOU-2G:C04",
        "BEIDOU-2G:C05",
        "BEIDOU-2G:C17",
        "BEIDOU-2G:C18",
        "BEIDOU-2I:C06",
        "BEIDOU-2I:C07",
        "BEIDOU-2I:C08",
        "BEIDOU-2I:C09",
        "BEIDOU-2I:C10",
        "BEIDOU-2I:C13",
        "BEIDOU-2I:C15",
        "BEIDOU-2I:C16",
        "BEIDOU-2M:C11",
        "BEIDOU-2M:C12",
        "BEIDOU-2M:C13",
        "BEIDOU-2M:C14",
        "BEIDOU-2M:C30",
        "BEIDOU-3G-CAST:C59",
        "BEIDOU-3G-CAST:C60",
        "BEIDOU-3G-CAST:C61",
        "BEIDOU-3I:C38",
        "BEIDOU-3I:C39",
        "BEIDOU-3I:C40",
        "BEIDOU-3M-CAST:C19",
        "BEIDOU-3M-CAST:C20",
        "BEIDOU-3M-CAST:C21",
        "BEIDOU-3M-CAST:C22",
        "BEIDOU-3M-CAST:C23",
        "BEIDOU-3M-CAST:C24",
        "BEIDOU-3M-CAST:C32",
        "BEIDOU-3M-CAST:C33",
        "BEIDOU-3M-CAST:C36",
        "BEIDOU-3M-CAST:C37",
        "BEIDOU-3M-CAST:C41",
        "BEIDOU-3M-CAST:C42",
        "BEIDOU-3M-CAST:C45",
        "BEIDOU-3M-CAST:C46",
        "BEIDOU-3M-CAST:C47",
        "BEIDOU-3M-SECM:C25",
        "BEIDOU-3M-SECM:C26",
        "BEIDOU-3M-SECM:C27",
        "BEIDOU-3M-SECM:C28",
        "BEIDOU-3M-SECM:C29",
        "BEIDOU-3M-SECM:C30",
        "BEIDOU-3M-SECM:C34",
        "BEIDOU-3M-SECM:C35",
        "BEIDOU-3M-SECM:C43",
        "BEIDOU-3M-SECM:C44",
        "BEIDOU-3M-SECM:C48",
        "BEIDOU-3SI-CAST:C18",
        "BEIDOU-3SI-CAST:C32",
        "BEIDOU-3SI-CAST:C56",
        "BEIDOU-3SI-SECM:C16",
        "BEIDOU-3SI-SECM:C31",
        "BEIDOU-3SM-CAST:C19",
        "BEIDOU-3SM-CAST:C28",
        "BEIDOU-3SM-CAST:C33",
        "BEIDOU-3SM-CAST:C34",
        "BEIDOU-3SM-CAST:C57",
        "BEIDOU-3SM-CAST:C58",
        "BLOCK I:G03",
        "BLOCK I:G04",
        "BLOCK I:G05",
        "BLOCK I:G06",
        "BLOCK I:G07",
        "BLOCK I:G08",
        "BLOCK I:G09",
        "BLOCK I:G11",
        "BLOCK I:G12",
        "BLOCK I:G13",
        "BLOCK II:G02",
        "BLOCK II:G14",
        "BLOCK II:G15",
        "BLOCK II:G16",
        "BLOCK II:G17",
        "BLOCK II:G18",
        "BLOCK II:G19",
        "BLOCK II:G20",
        "BLOCK II:G21",
        "BLOCK IIA:G01",
        "BLOCK IIA:G03",
        "BLOCK IIA:G04",
        "BLOCK IIA:G05",
        "BLOCK IIA:G06",
        "BLOCK IIA:G07",
        "BLOCK IIA:G08",
        "BLOCK IIA:G09",
        "BLOCK IIA:G10",
        "BLOCK IIA:G18",
        "BLOCK IIA:G22",
        "BLOCK IIA:G23",
        "BLOCK IIA:G24",
        "BLOCK IIA:G25",
        "BLOCK IIA:G26",
        "BLOCK IIA:G27",
        "BLOCK IIA:G28",
        "BLOCK IIA:G29",
        "BLOCK IIA:G30",
        "BLOCK IIA:G31",
        "BLOCK IIA:G32",
        "BLOCK IIF:G01",
        "BLOCK IIF:G03",
        "BLOCK IIF:G06",
        "BLOCK IIF:G08",
        "BLOCK IIF:G09",
        "BLOCK IIF:G10",
        "BLOCK IIF:G24",
        "BLOCK IIF:G25",
        "BLOCK IIF:G26",
        "BLOCK IIF:G27",
        "BLOCK IIF:G30",
        "BLOCK IIF:G32",
        "BLOCK IIIA:G04",
        "BLOCK IIIA:G11",
        "BLOCK IIIA:G14",
        "BLOCK IIIA:G18",
        "BLOCK IIIA:G23",
        "BLOCK IIIA:G28",
        "BLOCK IIR-A:G11",
        "BLOCK IIR-A:G13",
        "BLOCK IIR-A:G14",
        "BLOCK IIR-A:G16",
        "BLOCK IIR-A:G18",
        "BLOCK IIR-A:G20",
        "BLOCK IIR-A:G21",
        "BLOCK IIR-A:G22",
        "BLOCK IIR-A:G28",
        "BLOCK IIR-B:G02",
        "BLOCK IIR-B:G19",
        "BLOCK IIR-B:G22",
        "BLOCK IIR-B:G23",
        "BLOCK IIR-M:G01",
        "BLOCK IIR-M:G04",
        "BLOCK IIR-M:G05",
        "BLOCK IIR-M:G06",
        "BLOCK IIR-M:G07",
        "BLOCK IIR-M:G08",
        "BLOCK IIR-M:G12",
        "BLOCK IIR-M:G15",
        "BLOCK IIR-M:G17",
        "BLOCK IIR-M:G24",
        "BLOCK IIR-M:G27",
        "BLOCK IIR-M:G28",
        "BLOCK IIR-M:G29",
        "BLOCK IIR-M:G30",
        "BLOCK IIR-M:G31",
        "CHAPS9017       NONE",
        "CHCC220GR       CHCD",
        "CHCC220GR2      CHCD",
        "CHCI80          NONE",
        "CHCX91+S        NONE",
        "CNTAT340        NONE",
        "CNTAT350        CNTS",
        "CNTAT500        CNTS",
        "CNTAT600        CNTS",
        "CNTT30          NONE",
        "CNTT300         NONE",
        "CNTT300PLUS     NONE",
        "EML_REACH_RS2   NONE",
        "EML_REACH_RS2+  NONE",
        "EML_REACH_RX    NONE",
        "ESVUA92         NONE",
        "FOIA90          NONE",
        "GALILEO-0A:E51",
        "GALILEO-0B:E52",
        "GALILEO-1:E11",
        "GALILEO-1:E12",
        "GALILEO-1:E19",
        "GALILEO-1:E20",
        "GALILEO-2:E01",
        "GALILEO-2:E02",
        "GALILEO-2:E03",
        "GALILEO-2:E04",
        "GALILEO-2:E05",
        "GALILEO-2:E07",
        "GALILEO-2:E08",
        "GALILEO-2:E09",
        "GALILEO-2:E10",
        "GALILEO-2:E13",
        "GALILEO-2:E14",
        "GALILEO-2:E15",
        "GALILEO-2:E18",
        "GALILEO-2:E21",
        "GALILEO-2:E22",
        "GALILEO-2:E24",
        "GALILEO-2:E25",
        "GALILEO-2:E26",
        "GALILEO-2:E27",
        "GALILEO-2:E30",
        "GALILEO-2:E31",
        "GALILEO-2:E33",
        "GALILEO-2:E34",
        "GALILEO-2:E36",
        "GINCYF90        NONE",
        "GING20M         NONE",
        "GING30          NONE",
        "GLONASS-K1:R03",
        "GLONASS-K1:R04",
        "GLONASS-K1:R08",
        "GLONASS-K1:R09",
        "GLONASS-K1:R11",
        "GLONASS-K1:R14",
        "GLONASS-K1:R17",
        "GLONASS-K1:R22",
        "GLONASS-K1:R25",
        "GLONASS-K1:R26",
        "GLONASS-K1:R27",
        "GLONASS-M:R01",
        "GLONASS-M:R02",
        "GLONASS-M:R03",
        "GLONASS-M:R04",
        "GLONASS-M:R05",
        "GLONASS-M:R06",
        "GLONASS-M:R07",
        "GLONASS-M:R08",
        "GLONASS-M:R09",
        "GLONASS-M:R10",
        "GLONASS-M:R11",
        "GLONASS-M:R12",
        "GLONASS-M:R13",
        "GLONASS-M:R14",
        "GLONASS-M:R15",
        "GLONASS-M:R16",
        "GLONASS-M:R17",
        "GLONASS-M:R18",
        "GLONASS-M:R19",
        "GLONASS-M:R20",
        "GLONASS-M:R21",
        "GLONASS-M:R22",
        "GLONASS-M:R23",
        "GLONASS-M:R24",
        "GLONASS-M:R27",
        "GLONASS:R01",
        "GLONASS:R02",
        "GLONASS:R03",
        "GLONASS:R04",
        "GLONASS:R05",
        "GLONASS:R06",
        "GLONASS:R07",
        "GLONASS:R08",
        "GLONASS:R09",
        "GLONASS:R10",
        "GLONASS:R11",
        "GLONASS:R12",
        "GLONASS:R13",
        "GLONASS:R14",
        "GLONASS:R15",
        "GLONASS:R16",
        "GLONASS:R17",
        "GLONASS:R18",
        "GLONASS:R19",
        "GLONASS:R20",
        "GLONASS:R21",
        "GLONASS:R22",
        "GLONASS:R23",
        "GLONASS:R24",
        "GMXZENITH06     NONE",
        "GMXZENITH10     NONE",
        "GMXZENITH15     NONE",
        "GMXZENITH16     NONE",
        "GMXZENITH20     NONE",
        "GMXZENITH25     NONE",
        "GMXZENITH25PRO  NONE",
        "GMXZENITH35     NONE",
        "GMXZENITH40     NONE",
        "GMXZENITH60     NONE",
        "HEMS631         NONE",
        "HGGCYH8372      HGGS",
        "HITAT45101CP    HITZ",
        "HXCCGX601A      HXCS",
        "HXCCGX611A      HXCM",
        "IGAIG8          NONE",
        "IRNSS-1GEO:I03",
        "IRNSS-1GEO:I06",
        "IRNSS-1GEO:I07",
        "IRNSS-1IGSO:I01",
        "IRNSS-1IGSO:I02",
        "IRNSS-1IGSO:I04",
        "IRNSS-1IGSO:I05",
        "ITT3750323      SCIS",
        "JAVGRANT_G5T+GP JVSD",
        "JAVRINGANT_DM   JVDM",
        "JAVRINGANT_DM   NONE",
        "JAVRINGANT_DM   SCIS",
        "JAVRINGANT_G5T  JAVC",
        "JAVRINGANT_G5T  JAVD",
        "JAVRINGANT_G5T  NONE",
        "JAVTRIUMPH_1M   NONE",
        "JAVTRIUMPH_1MR  NONE",
        "JAVTRIUMPH_2A   NONE",
        "JAVTRIUMPH_2A+G JVGR",
        "JAVTRIUMPH_2A+P JVGR",
        "JAVTRIUMPH_2A+P JVSD",
        "JAVTRIUMPH_3A   NONE",
        "JAVTRIUMPH_LSA  NONE",
        "JAV_GRANT-G3T   NONE",
        "JAV_GRANT-G3T+G JVSD",
        "JAV_RINGANT_G3T JAVC",
        "JAV_RINGANT_G3T JAVD",
        "JAV_RINGANT_G3T NONE",
        "JNSCR_C146-22-1 NONE",
        "JNSMARANT_GGD   NONE",
        "JPLD/M_R        NONE",
        "JPLD/M_RA_SOP   NONE",
        "JPSLEGANT_E     NONE",
        "JPSODYSSEY_I    NONE",
        "JPSREGANT_DD_E  NONE",
        "JPSREGANT_DD_E1 NONE",
        "JPSREGANT_DD_E2 NONE",
        "JPSREGANT_SD_E  NONE",
        "JPSREGANT_SD_E1 NONE",
        "JPSREGANT_SD_E2 NONE",
        "LEIAR10         NONE",
        "LEIAR20         LEIM",
        "LEIAR20         NONE",
        "LEIAR25         LEIT",
        "LEIAR25         NONE",
        "LEIAR25.R3      LEIT",
        "LEIAR25.R3      NONE",
        "LEIAR25.R4      LEIT",
        "LEIAR25.R4      NONE",
        "LEIAR25.R4      SCIT",
        "LEIAS05         NONE",
        "LEIAS10         NONE",
        "LEIAS11         NONE",
        "LEIAT202+GP     NONE",
        "LEIAT202-GP     NONE",
        "LEIAT302+GP     NONE",
        "LEIAT302-GP     NONE",
        "LEIAT303        LEIC",
        "LEIAT303        NONE",
        "LEIAT502        NONE",
        "LEIAT503        LEIC",
        "LEIAT503        NONE",
        "LEIAT504        LEIS",
        "LEIAT504        NONE",
        "LEIAT504        OLGA",
        "LEIAT504        SCIS",
        "LEIAT504GG      LEIS",
        "LEIAT504GG      NONE",
        "LEIAT504GG      SCIS",
        "LEIAT504GG      SCIT",
        "LEIATX1230      NONE",
        "LEIATX1230+GNSS NONE",
        "LEIATX1230GG    NONE",
        "LEIAX1202       NONE",
        "LEIAX1202GG     NONE",
        "LEIAX1203+GNSS  NONE",
        "LEICGA100       NONE",
        "LEICGA60        NONE",
        "LEIFLX100       NONE",
        "LEIGG02PLUS     NONE",
        "LEIGG03         NONE",
        "LEIGG04         NONE",
        "LEIGG04PLUS     NONE",
        "LEIGS08         NONE",
        "LEIGS08PLUS     NONE",
        "LEIGS09         NONE",
        "LEIGS12         NONE",
        "LEIGS14         NONE",
        "LEIGS15         NONE",
        "LEIGS15.R2      NONE",
        "LEIGS16         NONE",
        "LEIGS18         NONE",
        "LEIICG60        NONE",
        "LEIICG70        NONE",
        "LEIMNA950GG     NONE",
        "LEISR299_INT    NONE",
        "LEISR399_INT    NONE",
        "LEISR399_INTA   NONE",
        "MAC4647942      MMAC",
        "MAC4647942      NONE",
        "MPLL1/L2_SURV   NONE",
        "MPL_WAAS_2224NW NONE",
        "MPL_WAAS_2225NW NONE",
        "MVECR152GNSSA   NONE",
        "MVEGA152GNSSA   NONE",
        "NAVAN2004T      NONE",
        "NAVAN2008T      NONE",
        "NAX3G+C         NONE",
        "NOV501          NONE",
        "NOV501+CR       NONE",
        "NOV502          NONE",
        "NOV502+CR       NONE",
        "NOV503+CR       NONE",
        "NOV503+CR       SPKE",
        "NOV531          NONE",
        "NOV531+CR       NONE",
        "NOV533+CR       NOVC",
        "NOV600          NONE",
        "NOV702          NONE",
        "NOV702GG        NONE",
        "NOV703GGG.R2    NONE",
        "NOV750.R4       NONE",
        "NOV750.R4       NOVS",
        "NOV750.R5       NOVS",
        "NOV850          NONE",
        "NOV_WAAS_600    NONE",
        "QZSS-2A:J04",
        "QZSS-2G:J07",
        "QZSS-2I:J02",
        "QZSS-2I:J03",
        "QZSS:J01",
        "RNG80971.00     NONE",
        "SEN67157596+CR  NONE",
        "SEPALTUS_NR3    NONE",
        "SEPCHOKE_B3E6   NONE",
        "SEPCHOKE_B3E6   SPKE",
        "SEPCHOKE_MC     NONE",
        "SEPCHOKE_MC     SPKE",
        "SEPPOLANT_X_MF  NONE",
        "SEPVC6150L      NONE",
        "SEPVC6150L      SCIS",
        "SJTTL111        NONE",
        "SLGAT45101CP    SLGZ",
        "SOK502          NONE",
        "SOK600          NONE",
        "SOK702          NONE",
        "SOKGCX3         NONE",
        "SOKGRX3         NONE",
        "SOKSA500        NONE",
        "SOK_RADIAN_IS   NONE",
        "SPP135000.00    NONE",
        "SPP571212238+GP NONE",
        "SPPSP85         NONE",
        "SPPSP85UHF      NONE",
        "STHCR3-G3       STHC",
        "STXS10SX017A    NONE",
        "STXS700A        NONE",
        "STXS800         NONE",
        "STXS800A        NONE",
        "STXS8PX003A     NONE",
        "STXS900         NONE",
        "STXS900A        NONE",
        "STXS990A        NONE",
        "STXS9I          NONE",
        "STXS9PX001A     NONE",
        "STXS9SA7224V3.0 NONE",
        "STXSA1000       NONE",
        "STXSA1200       STXR",
        "STXSA1500       STXG",
        "STXSA1800       STXS",
        "TIAPENG2100B    NONE",
        "TIAPENG2100R    NONE",
        "TIAPENG3100R1   NONE",
        "TIAPENG3100R2   NONE",
        "TIAPENG6J2      NONE",
        "TIAPENG7N       NONE",
        "TOP700779A      NONE",
        "TOP72110        NONE",
        "TPSCR.G3        NONE",
        "TPSCR.G3        SCIS",
        "TPSCR.G3        TPSH",
        "TPSCR.G5        NONE",
        "TPSCR.G5        TPSH",
        "TPSCR.G5C       NONE",
        "TPSCR.G5C       TPSH",
        "TPSCR3_GGD      CONE",
        "TPSCR3_GGD      NONE",
        "TPSCR3_GGD      OLGA",
        "TPSCR3_GGD      PFAN",
        "TPSCR4          CONE",
        "TPSCR4          NONE",
        "TPSG3_A1        NONE",
        "TPSG3_A1        TPSD",
        "TPSG5_A1        NONE",
        "TPSHIPER_GD     NONE",
        "TPSHIPER_GGD    NONE",
        "TPSHIPER_HR     NONE",
        "TPSHIPER_HR+PS  NONE",
        "TPSHIPER_LITE   NONE",
        "TPSHIPER_PLUS   NONE",
        "TPSHIPER_VR     NONE",
        "TPSLEGANT2      NONE",
        "TPSLEGANT3_UHF  NONE",
        "TPSLEGANT_G     NONE",
        "TPSODYSSEY_I    NONE",
        "TPSPG_A1        NONE",
        "TPSPG_A1+GP     NONE",
        "TPSPN.A5        NONE",
        "TRM105000.10    NONE",
        "TRM115000.00    NONE",
        "TRM115000.00    TZGD",
        "TRM115000.00+S  SCIT",
        "TRM115000.10    NONE",
        "TRM14177.00     NONE",
        "TRM14532.00     NONE",
        "TRM14532.10     NONE",
        "TRM159800.00    NONE",
        "TRM159800.00    SCIS",
        "TRM159800.00    SCIT",
        "TRM159900.00    NONE",
        "TRM159900.00    SCIS",
        "TRM22020.00+GP  NONE",
        "TRM22020.00-GP  NONE",
        "TRM23903.00     NONE",
        "TRM27947.00+GP  NONE",
        "TRM27947.00-GP  NONE",
        "TRM29659.00     NONE",
        "TRM29659.00     OLGA",
        "TRM29659.00     SCIS",
        "TRM29659.00     SCIT",
        "TRM29659.00     SNOW",
        "TRM29659.00     TCWD",
        "TRM29659.00     UNAV",
        "TRM33429.00+GP  NONE",
        "TRM33429.00-GP  NONE",
        "TRM33429.20+GP  NONE",
        "TRM33429.20+GP  TCWD",
        "TRM33429.20+GP  UNAV",
        "TRM39105.00     NONE",
        "TRM41249.00     NONE",
        "TRM41249.00     SCIT",
        "TRM41249.00     TZGD",
        "TRM41249USCG    SCIT",
        "TRM4800         NONE",
        "TRM55970.00     NONE",
        "TRM55971.00     NONE",
        "TRM55971.00     SCIT",
        "TRM55971.00     TZGD",
        "TRM57970.00     NONE",
        "TRM57971.00     NONE",
        "TRM57971.00     SCIT",
        "TRM57971.00     TZGD",
        "TRM5800         NONE",
        "TRM59800.00     NONE",
        "TRM59800.00     SCIS",
        "TRM59800.00     SCIT",
        "TRM59800.00C    NONE",
        "TRM59800.80     NONE",
        "TRM59800.80     SCIS",
        "TRM59800.80     SCIT",
        "TRM59800.99     NONE",
        "TRM59800.99     SCIS",
        "TRM59800.99     SCIT",
        "TRM59900.00     NONE",
        "TRM59900.00     SCIS",
        "TRMR10          NONE",
        "TRMR10-2        NONE",
        "TRMR12          NONE",
        "TRMR12I         NONE",
        "TRMR2           NONE",
        "TRMR4-3         NONE",
        "TRMR6-4         NONE",
        "TRMR780         NONE",
        "TRMR8-4         NONE",
        "TRMR8S          NONE",
        "TRMR8_GNSS      NONE",
        "TRMR8_GNSS3     NONE",
        "TRMSPS985       NONE",
        "TRMSPS986       NONE",
        "TRSAX4E02       NONE",
        "TWIVC6050       NONE",
        "TWIVC6050       SCIS",
        "TWIVC6050       SCIT",
        "TWIVC6150       NONE",
        "TWIVC6150       SCIS",
        "TWIVP6000       NONE",
        "TWIVP6050_CONE  NONE",
        "TWIVSP6037L     NONE",
    };

    const auto& availableAntennas = NAV::AntexReader::Get().antennas();
    for (const auto& antenna : mustHaveAntennas)
    {
        LOG_INFO("Checking if '{}' is available", antenna);
        CAPTURE(antenna);
        CHECK(availableAntennas.contains(antenna));
    }

    LOG_INFO("All available antennas:\n  {}", fmt::join(availableAntennas, "\n  "));

    std::vector<std::string> diff;
    std::ranges::set_difference(availableAntennas, mustHaveAntennas, std::back_inserter(diff));
    if (!diff.empty())
    {
        LOG_WARN("New available antennas:\n  {}", fmt::join(diff, "\n  "));
    }
}

} // namespace NAV::TESTS::AntexReaderTests