/// @file Json.hpp
/// @brief Defines how to save certain datatypes to json
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-10

#pragma once

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include <imgui.h>

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