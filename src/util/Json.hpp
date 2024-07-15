// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Json.hpp
/// @brief Defines how to save certain datatypes to json
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-10

#pragma once

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include <imgui.h>
#include <implot.h>
#include <stdexcept>
#include <variant>

/// @brief Converts the provided color into a json object
/// @param[out] j Return Json object
/// @param[in] color Color to convert
void to_json(json& j, const ImColor& color);
/// @brief Converts the provided json object into a color
/// @param[in] j Json object with the color values
/// @param[out] color Color to return
void from_json(const json& j, ImColor& color);

/// @brief Converts the provided vector into a json object
/// @param[out] j Return Json object
/// @param[in] vec2 Vector to convert
void to_json(json& j, const ImVec2& vec2);
/// @brief Converts the provided json object into a vector
/// @param[in] j Json object with the vector values
/// @param[out] vec2 Vector to return
void from_json(const json& j, ImVec2& vec2);

/// @brief Converts the provided vector into a json object
/// @param[out] j Return Json object
/// @param[in] vec4 Vector to convert
void to_json(json& j, const ImVec4& vec4);
/// @brief Converts the provided json object into a vector
/// @param[in] j Json object with the vector values
/// @param[out] vec4 Vector to return
void from_json(const json& j, ImVec4& vec4);

/// @brief Converts the provided vector into a json object
/// @param[out] j Return Json object
/// @param[in] style Style to convert
void to_json(json& j, const ImPlotStyle& style);
/// @brief Converts the provided json object into a vector
/// @param[in] j Json object with the vector values
/// @param[out] style Style to return
void from_json(const json& j, ImPlotStyle& style);

namespace detail
{

/// @brief Variant serialize implementation
template<std::size_t N>
struct variant_switch
{
    /// @brief Access operator into the variant
    /// @param index Index of the variant
    /// @param value Json value of the variant
    /// @param v Variant to return
    template<typename Variant>
    void operator()(int index, const json& value, Variant& v) const
    {
        if (index == N)
        {
            v = value.get<std::variant_alternative_t<N, Variant>>();
        }
        else
        {
            variant_switch<N - 1>{}(index, value, v);
        }
    }
};

/// @brief Variant serialize implementation specialization for a single variant
template<>
struct variant_switch<0>
{
    /// @brief Access operator into the variant
    /// @param index Index of the variant
    /// @param value Json value of the variant
    /// @param v Variant to return
    template<typename Variant>
    void operator()(int index, const json& value, Variant& v) const
    {
        if (index == 0)
        {
            v = value.get<std::variant_alternative_t<0, Variant>>();
        }
        else
        {
            throw std::runtime_error(
                "while converting json to variant: invalid index");
        }
    }
};

} // namespace detail

namespace nlohmann
{

/// @brief ADL serializer for JSON
template<typename... Args>
struct adl_serializer<std::variant<Args...>>
{
    /// @brief Write info to a json object
    /// @param[out] j Json output
    /// @param[in] v Object to read info from
    static void to_json(json& j, const std::variant<Args...>& v)
    {
        std::visit([&](auto&& value) {
            j["index"] = v.index();
            j["value"] = std::forward<decltype(value)>(value);
        },
                   v);
    }
    /// @brief Read info from a json object
    /// @param[in] j Json variable to read info from
    /// @param[out] v Output object
    static void from_json(const json& j, std::variant<Args...>& v)
    {
        auto const index = j.at("index").get<int>();
        ::detail::variant_switch<sizeof...(Args) - 1>{}(index, j.at("value"), v);
    }
};

} // namespace nlohmann