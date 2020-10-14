#include <catch2/catch.hpp>

#include "NodeData/State/StateData.hpp"

namespace NAV
{
TEST_CASE("[StateData] Reference Functions", "[StateData]")
{
    StateData state;

    auto quat_nb_coeff = Eigen::Vector4d(1, 2, 3, 4);

    state.quaternion_nb().coeffs() = quat_nb_coeff;

    CHECK(state.quaternion_nb().coeffs() == quat_nb_coeff);

    quat_nb_coeff = Eigen::Vector4d(5, 6, 7, 8);
    auto quat_nb = Quaterniond<Navigation, Body>(quat_nb_coeff);

    state.quaternion_nb() = quat_nb;

    CHECK(state.quaternion_nb().coeffs() == quat_nb.coeffs());

    /* -------------------------------------------------------------------------------------------------------- */

    auto latLonAlt = Vector3d<LLA>(5, 6, 7);

    state.latLonAlt() = latLonAlt;

    CHECK(state.latLonAlt() == latLonAlt);
    CHECK(state.latitude() == latLonAlt(0));
    CHECK(state.longitude() == latLonAlt(1));
    CHECK(state.altitude() == latLonAlt(2));

    /* -------------------------------------------------------------------------------------------------------- */

    double lat = 8;
    double lon = 9;
    double alt = 10;

    state.latitude() = lat;
    state.longitude() = lon;
    state.altitude() = alt;
    CHECK(state.latLonAlt() == Eigen::Vector3d(lat, lon, alt));
    CHECK(state.latitude() == lat);
    CHECK(state.longitude() == lon);
    CHECK(state.altitude() == alt);

    /* -------------------------------------------------------------------------------------------------------- */

    auto vel_n = Vector3d<Navigation>(5, 6, 7);

    state.velocity_n() = vel_n;

    CHECK(state.velocity_n() == vel_n);
}

} // namespace NAV
