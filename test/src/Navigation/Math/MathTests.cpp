// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MathTests.cpp
/// @brief Math related tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-15

#include "util/Eigen.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Geometry/Quaternion.h>
#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "Logger.hpp"
#include "Navigation/Math/Math.hpp"

#include "Navigation/Transformations/Units.hpp"

namespace NAV::TESTS
{

TEST_CASE("[Math] Catch Matcher significant digits", "[Math]")
{
    auto logger = initializeTestLogger();
    {
        constexpr double val1 = 0.4657e-08;
        constexpr double val2 = 0.46574932423e-08;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = 0.4657e-08;
        constexpr double val2 = 0.46574932423e-08;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = 0.1490e-07;
        constexpr double val2 = 0.149000000002e-07;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = 0.2892e+00;
        constexpr double val2 = 0.28915;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = 0.2892e+00;
        constexpr double val2 = 0.28914;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) != math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, !Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = -0.2892e+00;
        constexpr double val2 = -0.28915;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = 0.2891232e+10;
        constexpr double val2 = 0.28912321334e10;
        LOG_DEBUG("{:.7f} == {:.7f}", val1, math::roundSignificantDigits(val2, 7));
        REQUIRE(math::roundSignificantDigits(val1, 7) == math::roundSignificantDigits(val2, 7));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 7));
    }
    {
        constexpr double val1 = 10.23;
        constexpr double val2 = 10.2345;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = 0.123;
        constexpr double val2 = 0.12345;
        LOG_DEBUG("{:.3f} == {:.3f}", val1, math::roundSignificantDigits(val2, 3));
        REQUIRE(math::roundSignificantDigits(val1, 3) == math::roundSignificantDigits(val2, 3));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 3));
    }
    {
        constexpr double val1 = 0.0123;
        constexpr double val2 = 0.012345;
        LOG_DEBUG("{:.3f} == {:.3f}", val1, math::roundSignificantDigits(val2, 3));
        REQUIRE(math::roundSignificantDigits(val1, 3) == math::roundSignificantDigits(val2, 3));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 3));
    }
    {
        constexpr double val1 = -0.1490e-07;
        constexpr double val2 = -0.149000000002e-07;
        LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
        REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));
    }
    {
        constexpr double val1 = 0.0;
        REQUIRE(val1 == math::roundSignificantDigits(val1, 2));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val1, 2));
    }
    {
        constexpr double val1 = 1e-19;
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val1, 19));
    }
    {
        constexpr double val1 = 1.85041689389;
        constexpr double val2 = 1.850416893885315;
        LOG_DEBUG("{:.12f} == {:.12f}", val1, math::roundSignificantDigits(val2, 12));
        REQUIRE(math::roundSignificantDigits(val1, 12) == math::roundSignificantDigits(val2, 12));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 12));
    }
    {
        constexpr double val1 = -1.85041689389;
        constexpr double val2 = -1.850416893885315;
        LOG_DEBUG("{:.12f} == {:.12f}", val1, math::roundSignificantDigits(val2, 12));
        REQUIRE(math::roundSignificantDigits(val1, 12) == math::roundSignificantDigits(val2, 12));
        REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 12));
    }
}

TEST_CASE("[Math] Catch Matcher significant digits container", "[Math]")
{
    auto logger = initializeTestLogger();

    std::array<double, 6> arr1 = { 0.4657e-08, 0.1490e-07, 0.2892e+00, -0.2892e+00, 10.23, -0.1490e-07 };
    std::array<double, 6> arr2 = { 0.46574932423e-08, 0.149000000002e-07, 0.28915, -0.28915, 10.2345, -0.149000000002e-07 };
    LOG_DEBUG("Array:\n{}\n    ==\n{}", joinToString(arr1, ", ", ":.5e"), joinToString(arr2, ", ", ":.5e"));
    REQUIRE_THAT(arr1, Catch::Matchers::EqualsSigDigitsContainer(arr2, 4));

    std::vector<double> vec1;
    std::vector<double> vec2;
    std::ranges::copy(arr1, std::back_inserter(vec1));
    std::ranges::copy(arr2, std::back_inserter(vec2));
    LOG_DEBUG("Vector:\n{}\n    ==\n{}", joinToString(vec1, ", ", ":.5e"), joinToString(vec2, ", ", ":.5e"));
    REQUIRE_THAT(vec1, Catch::Matchers::EqualsSigDigitsContainer(vec2, 4));
}

TEST_CASE("[Math] Rotation angular rate DCM/Quat Derivative", "[Math]")
{
    auto logger = initializeTestLogger();

    auto A_Titterton = [](const Eigen::Vector3d& T_omega_ST) -> Eigen::Matrix4d {
        // clang-format off
        Eigen::Matrix4d A;
        A <<       0      , -T_omega_ST.x(), -T_omega_ST.y(), -T_omega_ST.z(),
            T_omega_ST.x(),        0       ,  T_omega_ST.z(), -T_omega_ST.y(),
            T_omega_ST.y(), -T_omega_ST.z(),        0       ,  T_omega_ST.x(),
            T_omega_ST.z(),  T_omega_ST.y(), -T_omega_ST.x(),        0       ;
        // clang-format on
        return A;
    };
    auto A_Groves = [](const Eigen::Vector3d& T_omega_TS) -> Eigen::Matrix4d {
        // clang-format off
        Eigen::Matrix4d A;
        A <<       0      , -T_omega_TS.x(), -T_omega_TS.y(), -T_omega_TS.z(),
            T_omega_TS.x(),        0       , -T_omega_TS.z(),  T_omega_TS.y(),
            T_omega_TS.y(),  T_omega_TS.z(),        0       , -T_omega_TS.x(),
            T_omega_TS.z(), -T_omega_TS.y(),  T_omega_TS.x(),        0       ;
        // clang-format on
        return A;
    };

    auto quatFromCoeffsWXYZ = [](const Eigen::Vector4d& coeffsWXYZ) -> Eigen::Quaterniond {
        return { coeffsWXYZ(0), coeffsWXYZ(1), coeffsWXYZ(2), coeffsWXYZ(3) };
    };
    auto quatCoeffsWXYZ = [](const Eigen::Quaterniond& quat) -> Eigen::Vector4d {
        return { quat.w(), quat.x(), quat.y(), quat.z() };
    };

    auto transform = [&]([[maybe_unused]] const char* idx, const Eigen::Vector3d& T_p, const Eigen::Vector3d& S_ref1, const Eigen::Vector3d& S_ref2) {
        Eigen::Matrix3d S_DCM_T = Eigen::Matrix3d::Identity();
        Eigen::Quaterniond S_quat_T = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond S_quatTitterton_T = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond S_quatGroves_T = Eigen::Quaterniond::Identity();

        LOG_INFO("T_{}: {}", idx, T_p.transpose());
        LOG_INFO("S_{}: {}\n", idx, (S_DCM_T * T_p).array().round().transpose());

        Eigen::Vector3d T_omega_TS(0, 0, deg2rad(90));
        Eigen::Vector3d T_omega_ST = -T_omega_TS;
        S_DCM_T = S_DCM_T * math::expMapMatrix(T_omega_ST);
        S_quat_T = S_quat_T * math::expMapQuat(T_omega_ST);
        S_quatTitterton_T = quatFromCoeffsWXYZ(A_Titterton(0.5 * T_omega_ST).exp() * quatCoeffsWXYZ(S_quatTitterton_T));
        S_quatGroves_T = quatFromCoeffsWXYZ(A_Groves(0.5 * T_omega_TS).exp() * quatCoeffsWXYZ(S_quatGroves_T));
        LOG_INFO("S_{} (DCM):       {}", idx, (S_DCM_T * T_p).array().round().transpose());
        LOG_INFO("S_{} (quat):      {}", idx, (S_quat_T * T_p).array().round().transpose());
        LOG_INFO("S_{} (Titterton): {}", idx, (S_quatTitterton_T * T_p).array().round().transpose());
        LOG_INFO("S_{} (Groves):    {}\n", idx, (S_quatGroves_T * T_p).array().round().transpose());
        REQUIRE((S_DCM_T * T_p).array().round().matrix() == S_ref1);
        REQUIRE((S_quat_T * T_p).array().round().matrix() == S_ref1);
        REQUIRE((S_quatTitterton_T * T_p).array().round().matrix() == S_ref1);
        // REQUIRE((S_quatGroves_T * T_p).array().round().matrix() == S_ref1); // NOTE: Groves implementation is different and does not work

        T_omega_TS = Eigen::Vector3d(deg2rad(90), 0, 0);
        T_omega_ST = -T_omega_TS;
        S_DCM_T = S_DCM_T * math::expMapMatrix(T_omega_ST);
        S_quat_T = S_quat_T * math::expMapQuat(T_omega_ST);
        S_quatTitterton_T = quatFromCoeffsWXYZ(A_Titterton(0.5 * T_omega_ST).exp() * quatCoeffsWXYZ(S_quatTitterton_T));
        S_quatGroves_T = quatFromCoeffsWXYZ(A_Groves(0.5 * T_omega_TS).exp() * quatCoeffsWXYZ(S_quatGroves_T));
        LOG_INFO("S_{} (DCM):       {}", idx, (S_DCM_T * T_p).array().round().transpose());
        LOG_INFO("S_{} (quat):      {}", idx, (S_quat_T * T_p).array().round().transpose());
        LOG_INFO("S_{} (Titterton): {}", idx, (S_quatTitterton_T * T_p).array().round().transpose());
        LOG_INFO("S_{} (Groves):    {}\n", idx, (S_quatGroves_T * T_p).array().round().transpose());
        REQUIRE((S_DCM_T * T_p).array().round().matrix() == S_ref2);
        REQUIRE((S_quat_T * T_p).array().round().matrix() == S_ref2);
        REQUIRE((S_quatTitterton_T * T_p).array().round().matrix() == S_ref2);
        // REQUIRE((S_quatGroves_T * T_p).array().round().matrix() == S_ref2); // NOTE: Groves implementation is different and does not work
    };

    transform("a", /* T_a */ { 1, 0, 0 }, /* S_ref1 */ { 0, -1, 0 }, /* S_ref2 */ { 0, -1, 0 });
    transform("b", /* T_b */ { 0, 1, 0 }, /* S_ref1 */ { 1, 0, 0 }, /* S_ref2 */ { 0, 0, -1 });
}

} // namespace NAV::TESTS