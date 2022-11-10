// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CartesianProduct.hpp
/// @brief Algorithm iterating all combinations of a list
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-11-04
///
/// @note Original code by Jonathan Boccara from https://www.fluentcpp.com/2022/03/18/how-to-generate-all-the-combinations-from-several-collections/

#pragma once

#include <algorithm>
#include <iostream>
#include <iterator>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace NAV
{
namespace CartesianProduct
{

namespace tuple_algos
{

/// @brief For-each implementation for tuples
/// @param[in] t The tuple
/// @param[in] f Function to call
template<class Tuple, class F, std::size_t... I>
F for_each_impl(Tuple&& t, F&& f, std::index_sequence<I...> /* seq */)
{
    return (void)std::initializer_list<int>{ (std::forward<F>(f)(std::get<I>(std::forward<Tuple>(t))), 0)... }, f;
}

/// @brief For-each algorithm for tuples
/// @param[in] t The tuple
/// @param[in] f Function to call
/// @return Function return value
template<class F, class Tuple>
constexpr decltype(auto) for_each(Tuple&& t, F&& f)
{
    return for_each_impl(std::forward<Tuple>(t), std::forward<F>(f),
                         std::make_index_sequence<std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
}

/// @brief Transform implementation for tuples
/// @param[in] t Tuple
/// @param[in] f Function to call
/// @return The transformed tuple
template<class Tuple, class F, std::size_t... Is>
auto transform_impl(Tuple&& t, F&& f, std::index_sequence<Is...> /* seq */)
    -> decltype(std::make_tuple(std::forward<F>(f)(std::get<Is>(std::forward<Tuple>(t)))...))
{
    return std::make_tuple(std::forward<F>(f)(std::get<Is>(std::forward<Tuple>(t)))...);
}

// Alternative implementation, which however does not compile with msvc
//
// @brief Transform implementation for tuples
// @param[in] inputs Input tuple
// @param[in] function Function to call
// @return The transformed tuple
// template<typename... Ts, typename Function, size_t... Is>
// auto transform_impl(std::tuple<Ts...> const& inputs, Function function, std::index_sequence<Is...> /* seq */)
// {
//     return std::tuple<std::result_of_t<Function(Ts)>...>{ function(std::get<Is>(inputs))... };
// }

/// @brief Transform algorithm for tuples
/// @param[in] inputs Input tuple
/// @param[in] function Function to call
/// @return The transformed tuple
template<typename... Ts, typename Function>
auto transform(std::tuple<Ts...> const& inputs, Function function)
{
    return transform_impl(inputs, function, std::make_index_sequence<sizeof...(Ts)>{});
}

/// @brief Find_if for tuples
/// @param[in] tuple The tuple to search
/// @param[in] pred Predicate to use for finding
/// @return Index of the found entry
template<typename Tuple, typename Predicate>
size_t find_if(Tuple&& tuple, Predicate pred)
{
    size_t index = std::tuple_size<std::decay_t<Tuple>>::value;
    size_t currentIndex = 0;
    bool found = false;
    for_each(tuple, [&](auto&& value) {
        if (!found && pred(value))
        {
            ++index = currentIndex;
            found = true;
        }
        ++currentIndex;
    });
    return index;
}

/// @brief Any_of algorithm for tuples
/// @param[in] tuple The tuple to search
/// @param[in] pred Predicate to check for
/// @return True if any of the tuple entries fulfills the predicate
template<typename Tuple, typename Predicate>
bool any_of(Tuple&& tuple, Predicate pred)
{
    return find_if(tuple, pred) != std::tuple_size<std::decay_t<Tuple>>::value;
}
} // namespace tuple_algos

/// @brief Dereference a tuple
/// @param[in] tuple Tuple to dereference
/// @return Dereferenced tuple values
template<typename... Ts>
auto dereference(std::tuple<Ts...> const& tuple)
{
    return tuple_algos::transform(tuple, [](auto&& element) -> decltype(auto) { return *element; });
}

/// @brief Helper struct to increment an iterator
template<size_t I>
struct increment_iterator
{
    /// @brief Increments an iterator of tuples
    /// @param[in] iterators The iterator to increment
    /// @param[in] beginIterators Beginning of the Iterators
    /// @param[in] endIterators End of the Iterators
    template<typename... Iterators>
    static void _(std::tuple<Iterators...>& iterators, std::tuple<Iterators...> const& beginIterators, std::tuple<Iterators...> const& endIterators)
    {
        auto& it = std::get<I>(iterators);
        auto const begin = std::get<I>(beginIterators);
        auto const end = std::get<I>(endIterators);

        ++it;

        if (it == end)
        {
            it = begin;
            increment_iterator<I - 1>::_(iterators, beginIterators, endIterators);
        }
    }
};

/// @brief Helper struct to increment an iterator
template<>
struct increment_iterator<0>
{
    /// @brief Increments an iterator of tuples
    /// @param[in] iterators The iterator to increment
    template<typename... Iterators>
    static void _(std::tuple<Iterators...>& iterators, std::tuple<Iterators...> const& /* beginIterators */, std::tuple<Iterators...> const& /* endIterators */)
    {
        auto& it = std::get<0>(iterators);

        ++it;
    }
};

/// @brief Gets the next combination of the tuples of iterators
/// @param[in] iterators The iterator to get the next combination for
/// @param[in] beginIterators Beginning of the Iterators
/// @param[in] endIterators End of the Iterators
template<typename... Iterators>
void next_combination(std::tuple<Iterators...>& iterators, std::tuple<Iterators...> const& beginIterators, std::tuple<Iterators...> const& endIterators)
{
    constexpr auto N = sizeof...(Iterators);
    increment_iterator<N - 1>::_(iterators, beginIterators, endIterators);
}

/// @brief Calls the function on the tuple
/// @param[in] function Function to call
/// @param[in] tuple Tuple to pass as argument
template<typename Function, typename... Ts, size_t... Is>
void callFunction(Function function, std::tuple<Ts...> const& tuple, std::index_sequence<Is...> /* seq */)
{
    function(std::get<Is>(tuple)...);
}

} // namespace CartesianProduct

/// @brief Calls a function on all combinations over ranges
/// @param[in] function Function to call. Needs one parameter for each range given. Type of the parameter must be equal to the range type.
/// @param[in] ranges Ranges to call the function on
template<typename Function, typename... Ranges>
void cartesian_product(Function function, Ranges const&... ranges)
{
    static_assert(sizeof...(Ranges) > 0, "There should be at least one range in cartesian_product.");
    auto const hasEmptyRange = CartesianProduct::tuple_algos::any_of(std::forward_as_tuple(ranges...), [](auto&& range) { return range.size() == 0; });

    if (!hasEmptyRange)
    {
        auto const beginIterators = std::make_tuple(begin(ranges)...);
        auto const endIterators = std::make_tuple(end(ranges)...);

        for (auto iterators = beginIterators; std::get<0>(iterators) != std::get<0>(endIterators); CartesianProduct::next_combination(iterators, beginIterators, endIterators))
        {
            std::apply(function, CartesianProduct::dereference(iterators));
        }
    }
}

/// @brief Calls a function on all combinations over ranges defined in an array, e.g. std::array<std::vector>
/// @param[in] function Function to call. Needs one parameter for each element in the array. Type of the parameter must be equal to the range type.
/// @param[in] list List of ranges to call the function on
template<typename SubList, size_t N, typename Function>
void cartesian_product(Function function, std::array<SubList, N> list)
{
    std::apply([function](auto... xs) { cartesian_product(function, std::forward<decltype(xs)>(xs)...); }, list);
}

/// @brief Calls a function on all combinations over ranges defined in an array, e.g. std::array<std::vector>.
///        Instead of providing the list entry to the function, provides the index of it in the sub list.
/// @param[in] function Function to call. Needs one 'size_t' parameter for each element in the array
/// @param[in] list List of ranges to call the function on
template<typename SubList, size_t N, typename Function>
void cartesian_product_idx(Function function, std::array<SubList, N> list)
{
    std::array<std::vector<size_t>, N> indices;
    for (size_t i = 0; i < list.size(); i++)
    {
        indices.at(i) = std::vector<size_t>(list.at(i).size());
        std::iota(indices.at(i).begin(), indices.at(i).end(), 0);
    }

    std::apply([function](auto... xs) { cartesian_product(function, std::forward<decltype(xs)>(xs)...); }, indices);
}

} // namespace NAV