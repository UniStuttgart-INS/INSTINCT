#include "Json.hpp"

void to_json(json& j, const ImColor& color)
{
    j = json{
        { "r", static_cast<int>(color.Value.x * 255.0F) },
        { "g", static_cast<int>(color.Value.y * 255.0F) },
        { "b", static_cast<int>(color.Value.z * 255.0F) },
        { "a", static_cast<int>(color.Value.w * 255.0F) },
    };
}
void from_json(const json& j, ImColor& color)
{
    int r = 0;
    int g = 0;
    int b = 0;
    int a = 255;
    if (j.contains("r"))
    {
        j.at("r").get_to(r);
    }
    if (j.contains("g"))
    {
        j.at("g").get_to(g);
    }
    if (j.contains("b"))
    {
        j.at("b").get_to(b);
    }
    if (j.contains("a"))
    {
        j.at("a").get_to(a);
    }

    color = ImColor(r, g, b, a);
}

void to_json(json& j, const ImVec2& vec2)
{
    j = json{
        { "x", vec2.x },
        { "y", vec2.y },
    };
}
void from_json(const json& j, ImVec2& vec2)
{
    if (j.contains("x"))
    {
        j.at("x").get_to(vec2.x);
    }
    if (j.contains("y"))
    {
        j.at("y").get_to(vec2.y);
    }
}

void to_json(json& j, const ImVec4& vec4)
{
    j = json{
        { "x", vec4.x },
        { "y", vec4.y },
        { "z", vec4.z },
        { "w", vec4.w },
    };
}
void from_json(const json& j, ImVec4& vec4)
{
    if (j.contains("x"))
    {
        j.at("x").get_to(vec4.x);
    }
    if (j.contains("y"))
    {
        j.at("y").get_to(vec4.y);
    }
    if (j.contains("z"))
    {
        j.at("z").get_to(vec4.z);
    }
    if (j.contains("w"))
    {
        j.at("w").get_to(vec4.w);
    }
}