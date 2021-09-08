/// @file NumericalIntegration.hpp
/// @brief
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-09-08

#pragma once

#include <type_traits>

namespace NAV::Integration
{
/// @brief Runge-Kutta Third Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s] (Time difference Δtₖ = (tₖ - tₖ₋₂))
/// @param[in] y__t2 State vector at time tₖ₋₂
/// @param[in] x__t2 Measurement at time tₖ₋₂
/// @param[in] x__t1 Measurement at time tₖ₋₁
/// @param[in] x__t0 Measurement at time tₖ
/// @return State vector at time tₖ
/// @note See Munz & Westermann (2012) - Numerische Behandlung gewöhnlicher und partieller Differenzialgleichungen (ch. 2.5, p. 97)
template<typename Y, typename Scalar, typename X,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y rungeKutta3(Y (*f)(const X&, const Y&), const Scalar& h, const Y& y__t2, const X& x__t2, const X& x__t1, const X& x__t0)
{
    // Runge-Kutta coefficient k₁
    const auto k1 = f(x__t2, y__t2);
    // Runge-Kutta coefficient k₂
    const auto k2 = f(x__t1, y__t2
                                 + h / 2.0 * k1);
    // Runge-Kutta coefficient k₃
    const auto k3 = f(x__t0, y__t2
                                 - h * k1
                                 + 2.0 * h * k2);

    return y__t2 + h / 6.0 * (k1 + 4.0 * k2 + k3);
};

} // namespace NAV::Integration
