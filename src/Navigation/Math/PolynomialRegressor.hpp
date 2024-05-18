// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PolynomialRegressor.hpp
/// @brief Polynomial curve fitting
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-23

#pragma once

#include <cstddef>
#include <utility>

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include "util/Assert.h"
#include "util/Container/ScrollingBuffer.hpp"

#include "Polynomial.hpp"
#include "internal/PolynomialRegressor/IncrementalLeastSquares.hpp"
#include "internal/PolynomialRegressor/LeastSquares.hpp"
#include "internal/PolynomialRegressor/HouseholderQr.hpp"
#include "internal/PolynomialRegressor/BDCSVD.hpp"
#include "internal/PolynomialRegressor/COD.hpp"

namespace NAV
{

/// @brief Polynomial Curve Fitting
/// @tparam Scalar Data type to store
template<typename Scalar = double>
class PolynomialRegressor
{
  public:
    /// Possible Fit strategies
    enum class Strategy
    {
        IncrementalLeastSquares, ///< Incremental Least Squares (only polynomials of order <= 2)
        LeastSquares,            ///< Least Squares (bas if even mildly ill-conditioned)
        HouseholderQR,           ///< Householder QR decomposition
        BDCSVD,                  ///< Bidiagonal Divide and Conquer SVD
        COD,                     ///< Complete Orthogonal Decomposition
        COUNT,                   ///< Amount of items in the enum
    };

    /// @brief Constructor
    /// @param[in] polynomialDegree Degree of the polynomial to fit
    /// @param[in] windowSize Amount of points to use for the fit (sliding window)
    /// @param[in] strategy Strategy to use
    PolynomialRegressor(size_t polynomialDegree, size_t windowSize, Strategy strategy = Strategy::HouseholderQR)
        : _strategy(strategy), _polyDegree(polynomialDegree), _windowSize(windowSize), _incrementalLSQ(polynomialDegree)
    {
        setWindowSize(windowSize);
        setPolynomialDegree(polynomialDegree);
    }

    /// @brief Sets the amount of points used for the fit (sliding window)
    /// @param[in] windowSize Amount of points to use for the fit
    void setWindowSize(size_t windowSize)
    {
        INS_ASSERT_USER_ERROR(windowSize > _polyDegree, "The window size needs to be greater than the polynomial degree.");

        while (windowSize < _windowSize)
        {
            pop_front();
            _windowSize--;
        }

        _windowSize = windowSize;
        _data.resize(windowSize);
    }

    /// @brief Set the Polynomial Degree and resets the data
    /// @param[in] polynomialDegree Degree of the polynomial to fit
    void setPolynomialDegree(size_t polynomialDegree)
    {
        INS_ASSERT_USER_ERROR(polynomialDegree < _windowSize, "The polynomial degree needs to be smaller than the window size.");
        _polyDegree = polynomialDegree;

        _incrementalLSQ.setPolynomialDegree(polynomialDegree);

        reset();
    }

    /// @brief Set the strategy for the fit and resets the data
    /// @param strategy Strategy to use for fitting data
    void setStrategy(Strategy strategy)
    {
        INS_ASSERT_USER_ERROR(strategy != Strategy::COUNT, "You cannot call this function with COUNT strategy");

        _strategy = strategy;
        reset();
    }

    /// @brief Add a data point to the polynomial
    /// @param[in] dataPoint Data point
    void push_back(const std::pair<Scalar, Scalar>& dataPoint)
    {
        push_back(dataPoint.first, dataPoint.second);
    }

    /// @brief Add a data point to the polynomial
    /// @param[in] x X Value
    /// @param[in] y Y Value
    void push_back(const Scalar& x, const Scalar& y)
    {
        if (!_data.empty() && _data.back().first == x)
        {
            if (_strategy == Strategy::IncrementalLeastSquares)
            {
                _incrementalLSQ.removeDataPoint(_data.back().first, _data.back().second);
                _incrementalLSQ.addDataPoint(x, y);
            }
            _data.back().second = y;

            return;
        }

        if (_data.full()) { pop_front(); }

        if (_strategy == Strategy::IncrementalLeastSquares)
        {
            _incrementalLSQ.addDataPoint(x, y);
        }

        _data.push_back(std::make_pair(x, y));
    }

    /// @brief Reset the polynomial coefficients and saved data
    void reset()
    {
        _incrementalLSQ.reset();
        _data.clear();
    }

    /// @brief Calculates the polynomial
    [[nodiscard]] Polynomial<Scalar> calcPolynomial() const
    {
        auto prepareDataVectors = [&]() {
            auto n = static_cast<int>(_data.size());
            Eigen::VectorX<Scalar> x = Eigen::VectorX<Scalar>(n);
            Eigen::VectorX<Scalar> y = Eigen::VectorX<Scalar>(n);

            for (size_t i = 0; i < _data.size(); i++)
            {
                x(static_cast<int>(i)) = _data.at(i).first;
                y(static_cast<int>(i)) = _data.at(i).second;
            }

            return std::make_pair(x, y);
        };

        switch (_strategy)
        {
        case Strategy::IncrementalLeastSquares:
            return { _incrementalLSQ.calcCoefficients() };
        case Strategy::LeastSquares:
        {
            auto [x, y] = prepareDataVectors();
            return { LeastSquares<Scalar>::calcCoefficients(x, y, _polyDegree) };
        }
        case Strategy::HouseholderQR:
        {
            auto [x, y] = prepareDataVectors();
            return { HouseholderQr<Scalar>::calcCoefficients(x, y, _polyDegree) };
        }
        case Strategy::BDCSVD:
        {
            auto [x, y] = prepareDataVectors();
            return { BDCSVD<Scalar>::calcCoefficients(x, y, _polyDegree) };
        }
        case Strategy::COD:
        {
            auto [x, y] = prepareDataVectors();
            return { COD<Scalar>::calcCoefficients(x, y, _polyDegree) };
        }
        case Strategy::COUNT:
            break;
        }
        return { Eigen::VectorX<Scalar>() };
    }

    /// @brief Checks if the amount of data points equals the window size
    [[nodiscard]] bool windowSizeReached() const
    {
        return _data.size() == _windowSize;
    }

    /// @brief Checks if the container has no elements
    [[nodiscard]] bool empty() const { return _data.empty(); }

    /// @brief Gets the underlying buffer
    [[nodiscard]] const ScrollingBuffer<std::pair<Scalar, Scalar>>& data() const { return _data; }

  private:
    /// Strategy to use to fit the polynomial
    Strategy _strategy = Strategy::IncrementalLeastSquares;
    /// Polynomial degree to fit
    size_t _polyDegree = 2;
    /// Amount of points to store
    size_t _windowSize = 10;
    /// Values added to the fit
    ScrollingBuffer<std::pair<Scalar, Scalar>> _data;
    /// Incremental LSQ Regressor
    IncrementalLeastSquares<Scalar> _incrementalLSQ;

    /// @brief Removes the first data point from the polynomial fit (sliding window)
    void pop_front()
    {
        if (_data.empty()) { return; }

        auto [x, y] = _data.front();
        _data.pop_front();

        if (_strategy == Strategy::IncrementalLeastSquares)
        {
            _incrementalLSQ.removeDataPoint(x, y);
        }
    }
};

/// @brief Converts the enum to a string
/// @param[in] strategy Enum value to convert into text
/// @return String representation of the enum
const char* to_string(PolynomialRegressor<>::Strategy strategy);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
template<typename Scalar = double>
void to_json(json& j, const PolynomialRegressor<Scalar>& obj)
{
    j = json{
        { "strategy", obj._strategy },
        { "polyDegree", obj._polyDegree },
        { "windowSize", obj._windowSize },
    };
}

/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
template<typename Scalar = double>
void from_json(const json& j, PolynomialRegressor<Scalar>& obj)
{
    if (j.contains("strategy"))
    {
        j.at("strategy").get_to(obj._strategy);
    }
    if (j.contains("polyDegree"))
    {
        j.at("polyDegree").get_to(obj._polyDegree);
        obj._incrementalLSQ.setPolynomialDegree(obj._polyDegree);
    }
    if (j.contains("windowSize"))
    {
        j.at("windowSize").get_to(obj._windowSize);
    }
}

} // namespace NAV
