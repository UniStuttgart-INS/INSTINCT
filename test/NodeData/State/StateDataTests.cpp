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
    auto quat_nb = Eigen::Quaterniond(quat_nb_coeff);

    state.quaternion_nb() = quat_nb;

    CHECK(state.quaternion_nb().coeffs() == quat_nb.coeffs());

    /* -------------------------------------------------------------------------------------------------------- */

    Eigen::Vector3d latLonAlt = Eigen::Vector3d(5, 6, 7);
    auto pos_e = trafo::lla2ecef_WGS84(latLonAlt);
    state.position_ecef() = pos_e;

    CHECK(state.position_ecef() == pos_e);
    CHECK(state.latLonAlt() == trafo::ecef2lla_WGS84(pos_e));
    CHECK(state.latitude() == trafo::ecef2lla_WGS84(pos_e)(0));
    CHECK(state.longitude() == trafo::ecef2lla_WGS84(pos_e)(1));
    CHECK(state.altitude() == trafo::ecef2lla_WGS84(pos_e)(2));

    // Supposed to do nothing, as we overwrite temporary variable
    state.latLonAlt() = latLonAlt;

    CHECK(state.latLonAlt() == trafo::ecef2lla_WGS84(pos_e));
    CHECK(state.latitude() == trafo::ecef2lla_WGS84(pos_e)(0));
    CHECK(state.longitude() == trafo::ecef2lla_WGS84(pos_e)(1));
    CHECK(state.altitude() == trafo::ecef2lla_WGS84(pos_e)(2));

    /* -------------------------------------------------------------------------------------------------------- */

    auto vel_n = Eigen::Vector3d(5, 6, 7);

    state.velocity_n() = vel_n;

    CHECK(state.velocity_n() == vel_n);
}

} // namespace NAV
