/// @file Json.hpp
/// @brief Defines how to save certain datatypes to json
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-10

#pragma once

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <imgui.h>

void to_json(json& j, const ImColor& color);
void from_json(const json& j, ImColor& color);

void to_json(json& j, const ImVec2& vec2);
void from_json(const json& j, ImVec2& vec2);