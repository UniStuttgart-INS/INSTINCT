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

template<class Tuple, class F, std::size_t... I>
F for_each_impl(Tuple&& t, F&& f, std::index_sequence<I...>)
{
    return (void)std::initializer_list<int>{ (std::forward<F>(f)(std::get<I>(std::forward<Tuple>(t))), 0)... }, f;
}

template<class F, class Tuple>
constexpr decltype(auto) for_each(Tuple&& t, F&& f)
{
    return for_each_impl(std::forward<Tuple>(t), std::forward<F>(f),
                         std::make_index_sequence<std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
}

template<typename... Ts, typename Function, size_t... Is>
auto transform_impl(std::tuple<Ts...> const& inputs, Function function, std::index_sequence<Is...>)
{
    return std::tuple<std::result_of_t<Function(Ts)>...>{ function(std::get<Is>(inputs))... };
}

template<typename... Ts, typename Function>
auto transform(std::tuple<Ts...> const& inputs, Function function)
{
    return transform_impl(inputs, function, std::make_index_sequence<sizeof...(Ts)>{});
}

template<typename Tuple, typename Predicate>
size_t find_if(Tuple&& tuple, Predicate pred)
{
    size_t index = std::tuple_size<std::decay_t<Tuple>>::value;
    size_t currentIndex = 0;
    bool found = false;
    // std::ranges::for_each(tuple, [&](auto& value)
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

template<typename Tuple, typename Predicate>
bool any_of(Tuple&& tuple, Predicate pred)
{
    return find_if(tuple, pred) != std::tuple_size<std::decay_t<Tuple>>::value;
}
} // namespace tuple_algos

template<typename... Ts>
auto dereference(std::tuple<Ts...> const& tuple)
{
    return tuple_algos::transform(tuple, [](auto&& element) -> decltype(auto) { return *element; });
}

template<size_t I>
struct increment_iterator
{
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

template<>
struct increment_iterator<0>
{
    template<typename... Iterators>
    static void _(std::tuple<Iterators...>& iterators, std::tuple<Iterators...> const&, std::tuple<Iterators...> const&)
    {
        auto& it = std::get<0>(iterators);

        ++it;
    }
};

template<typename... Iterators>
void next_combination(std::tuple<Iterators...>& iterators, std::tuple<Iterators...> const& beginIterators, std::tuple<Iterators...> const& endIterators)
{
    constexpr auto N = sizeof...(Iterators);
    increment_iterator<N - 1>::_(iterators, beginIterators, endIterators);
}

template<typename Function, typename... Ts, size_t... Is>
void callFunction(Function function, std::tuple<Ts...> const& tuple, std::index_sequence<Is...>)
{
    function(std::get<Is>(tuple)...);
}

} // namespace CartesianProduct

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

template<typename SubList, size_t N, typename Function>
void cartesian_product(Function function, std::array<SubList, N> list)
{
    std::apply([function](auto... xs) { cartesian_product(function, std::forward<decltype(xs)>(xs)...); }, list);
}

} // namespace NAV