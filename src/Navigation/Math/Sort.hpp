// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Sort.hpp
/// @brief Sorting algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-15

#pragma once

#include <vector>
#include <algorithm>
#include <numeric>
#include "util/Eigen.hpp"

namespace NAV
{

/// @brief Gives the permutation to sort the std::vector
/// @param vec Vector to get the sorting permutation for
/// @param compare Comparison operator
/// @return Permutation vector
/// @note Taken from https://stackoverflow.com/a/17074810
template<typename T, typename Compare>
std::vector<size_t> sort_permutation(const std::vector<T>& vec, Compare compare)
{
    std::vector<size_t> p(vec.size());
    std::iota(p.begin(), p.end(), 0); // NOLINT(boost-use-ranges,modernize-use-ranges) // ranges::iota is C++23 and not supported yet
    std::ranges::sort(p, [&](size_t i, size_t j) { return compare(vec[i], vec[j]); });
    return p;
}

/// @brief Gives the permutation to sort the vector
/// @param vec Vector to get the sorting permutation for
/// @param compare Comparison operator
/// @return Permutation vector
/// @note Taken from https://stackoverflow.com/a/17074810, adapted for Eigen
template<typename Derived, typename Compare>
std::vector<size_t> sort_permutation(const Eigen::MatrixBase<Derived>& vec, Compare compare)
{
    std::vector<size_t> p(static_cast<size_t>(vec.rows()));
    std::iota(p.begin(), p.end(), 0); // NOLINT(boost-use-ranges,modernize-use-ranges) // ranges::iota is C++23 and not supported yet
    std::ranges::sort(p, [&](size_t i, size_t j) { return compare(vec(static_cast<Eigen::Index>(i)), vec(static_cast<Eigen::Index>(j))); });
    return p;
}

/// @brief Sort a std::vector with a permutation
/// @param vec Vector to sort
/// @param p Permutation to sort by
/// @return Sorted vector
/// @note Taken from https://stackoverflow.com/a/17074810
template<typename T>
std::vector<T> apply_permutation(const std::vector<T>& vec, const std::vector<size_t>& p)
{
    std::vector<T> sorted_vec(vec.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(),
                   [&](size_t i) { return vec[i]; });
    return sorted_vec;
}

/// @brief Sort a vector with a permutation
/// @param vec Vector to sort
/// @param p Permutation to sort by
/// @return Sorted vector
/// @note Taken from https://stackoverflow.com/a/17074810, adapted for Eigen
template<typename Scalar, int Size>
Eigen::Vector<Scalar, Size> apply_permutation(const Eigen::Vector<Scalar, Size>& vec, const std::vector<size_t>& p)
{
    Eigen::Vector<Scalar, Size> sorted = vec;
    std::transform(p.begin(), p.end(), sorted.begin(),
                   [&](size_t i) { return vec(static_cast<Eigen::Index>(i)); });
    return sorted;
}

/// @brief Sort a matrix row-wise with a permutation
/// @param m Matrix to sort
/// @param p Permutation to sort by
/// @return Sorted matrix
/// @note Taken from https://stackoverflow.com/a/17074810, adapted for Eigen
template<typename Derived>
typename Derived::PlainObject apply_permutation_rowwise(const Eigen::MatrixBase<Derived>& m, const std::vector<size_t>& p)
{
    typename Derived::PlainObject sorted = m;
    std::transform(p.begin(), p.end(), sorted.rowwise().begin(),
                   [&](size_t i) { return m.row(static_cast<Eigen::Index>(i)); });
    return sorted;
}

/// @brief Sort a matrix col-wise with a permutation
/// @param m Matrix to sort
/// @param p Permutation to sort by
/// @return Sorted matrix
/// @note Taken from https://stackoverflow.com/a/17074810, adapted for Eigen
template<typename Derived>
typename Derived::PlainObject apply_permutation_colwise(const Eigen::MatrixBase<Derived>& m, const std::vector<size_t>& p)
{
    typename Derived::PlainObject sorted = m;
    std::transform(p.begin(), p.end(), sorted.colwise().begin(),
                   [&](size_t i) { return m.col(static_cast<Eigen::Index>(i)); });
    return sorted;
}

/// @brief Sort a std::vector with a permutation
/// @param vec Vector to sort
/// @param p Permutation to sort by
template<typename T>
void apply_permutation_in_place(std::vector<T>& vec, const std::vector<size_t>& p)
{
    std::vector<bool> done(p.size());
    for (size_t i = 0; i < p.size(); ++i)
    {
        if (done[i])
        {
            continue;
        }
        done[i] = true;
        size_t prev_j = i;
        size_t j = p[i];
        while (i != j)
        {
            std::swap(vec[prev_j], vec[j]);
            done[j] = true;
            prev_j = j;
            j = p[j];
        }
    }
}

/// @brief Sort a vector with a permutation
/// @param vec Vector to sort
/// @param p Permutation to sort by
/// @note Taken from https://stackoverflow.com/a/17074810
template<typename Scalar, int Size>
void apply_permutation_in_place(Eigen::Vector<Scalar, Size>& vec, const std::vector<size_t>& p)
{
    std::vector<bool> done(p.size());
    for (size_t i = 0; i < p.size(); ++i)
    {
        if (done[i])
        {
            continue;
        }
        done[i] = true;
        size_t prev_j = i;
        size_t j = p[i];
        while (i != j)
        {
            std::swap(vec(static_cast<Eigen::Index>(prev_j)), vec(static_cast<Eigen::Index>(j)));
            done[j] = true;
            prev_j = j;
            j = p[j];
        }
    }
}

/// @brief Sort a matrix row-wise with a permutation
/// @param m Matrix to sort
/// @param p Permutation to sort by
/// @note Taken from https://stackoverflow.com/a/17074810
template<typename Scalar, int Rows, int Cols>
void apply_permutation_rowwise_in_place(Eigen::Matrix<Scalar, Rows, Cols>& m, const std::vector<size_t>& p)
{
    std::vector<bool> done(p.size());
    for (size_t i = 0; i < p.size(); ++i)
    {
        if (done[i])
        {
            continue;
        }
        done[i] = true;
        size_t prev_j = i;
        size_t j = p[i];
        while (i != j)
        {
            m.row(static_cast<Eigen::Index>(prev_j)).swap(m.row(static_cast<Eigen::Index>(j)));
            done[j] = true;
            prev_j = j;
            j = p[j];
        }
    }
}

/// @brief Sort a matrix col-wise with a permutation
/// @param m Matrix to sort
/// @param p Permutation to sort by
/// @note Taken from https://stackoverflow.com/a/17074810
template<typename Scalar, int Rows, int Cols>
void apply_permutation_colwise_in_place(Eigen::Matrix<Scalar, Rows, Cols>& m, const std::vector<size_t>& p)
{
    std::vector<bool> done(p.size());
    for (size_t i = 0; i < p.size(); ++i)
    {
        if (done[i])
        {
            continue;
        }
        done[i] = true;
        size_t prev_j = i;
        size_t j = p[i];
        while (i != j)
        {
            m.col(static_cast<Eigen::Index>(prev_j)).swap(m.col(static_cast<Eigen::Index>(j)));
            done[j] = true;
            prev_j = j;
            j = p[j];
        }
    }
}

} // namespace NAV