#include "imgui_ex.hpp"

#include <algorithm>

namespace ImGui
{

// ###########################################################################################################

bool DragDouble(const char* label, double* v, float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags)
{
    return DragScalar(label, ImGuiDataType_Double, v, v_speed, &v_min, &v_max, format, flags);
}

bool DragDouble2(const char* label, double v[2], float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_Double, v, 2, v_speed, &v_min, &v_max, format, flags);
}

bool DragDouble3(const char* label, double v[3], float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_Double, v, 3, v_speed, &v_min, &v_max, format, flags);
}

bool DragDouble4(const char* label, double v[4], float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_Double, v, 4, v_speed, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool DragLong(const char* label, int64_t* v, float v_speed, int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags)
{
    return DragScalar(label, ImGuiDataType_S64, v, v_speed, &v_min, &v_max, format, flags);
}

bool DragLong2(const char* label, int64_t v[2], float v_speed, int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_S64, v, 2, v_speed, &v_min, &v_max, format, flags);
}

bool DragLong3(const char* label, int64_t v[3], float v_speed, int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_S64, v, 3, v_speed, &v_min, &v_max, format, flags);
}

bool DragLong4(const char* label, int64_t v[4], float v_speed, int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_S64, v, 4, v_speed, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool DragULong(const char* label, uint64_t* v, float v_speed, uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags)
{
    return DragScalar(label, ImGuiDataType_U64, v, v_speed, &v_min, &v_max, format, flags);
}

bool DragULong2(const char* label, uint64_t v[2], float v_speed, uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_U64, v, 2, v_speed, &v_min, &v_max, format, flags);
}

bool DragULong3(const char* label, uint64_t v[3], float v_speed, uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_U64, v, 3, v_speed, &v_min, &v_max, format, flags);
}

bool DragULong4(const char* label, uint64_t v[4], float v_speed, uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_U64, v, 4, v_speed, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool SliderDouble(const char* label, double* v, double v_min, double v_max, const char* format, ImGuiSliderFlags flags)
{
    return SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max, format, flags);
}

bool SliderDouble2(const char* label, double v[2], double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_Double, v, 2, &v_min, &v_max, format, flags);
}

bool SliderDouble3(const char* label, double v[3], double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_Double, v, 3, &v_min, &v_max, format, flags);
}

bool SliderDouble4(const char* label, double v[4], double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_Double, v, 4, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool SliderLong(const char* label, int64_t* v, int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags)
{
    return SliderScalar(label, ImGuiDataType_S64, v, &v_min, &v_max, format, flags);
}

bool SliderLong2(const char* label, int64_t v[2], int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_S64, v, 2, &v_min, &v_max, format, flags);
}

bool SliderLong3(const char* label, int64_t v[3], int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_S64, v, 3, &v_min, &v_max, format, flags);
}

bool SliderLong4(const char* label, int64_t v[4], int64_t v_min, int64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_S64, v, 4, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool SliderULong(const char* label, uint64_t* v, uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags)
{
    return SliderScalar(label, ImGuiDataType_U64, v, &v_min, &v_max, format, flags);
}

bool SliderULong2(const char* label, uint64_t v[2], uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_U64, v, 2, &v_min, &v_max, format, flags);
}

bool SliderULong3(const char* label, uint64_t v[3], uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_U64, v, 3, &v_min, &v_max, format, flags);
}

bool SliderULong4(const char* label, uint64_t v[4], uint64_t v_min, uint64_t v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_U64, v, 4, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool InputDouble2(const char* label, double v[2], const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return InputScalarN(label, ImGuiDataType_Double, v, 2, nullptr, nullptr, format, flags);
}

bool InputDouble3(const char* label, double v[3], const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return InputScalarN(label, ImGuiDataType_Double, v, 3, nullptr, nullptr, format, flags);
}

bool InputDouble4(const char* label, double v[4], const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return InputScalarN(label, ImGuiDataType_Double, v, 4, nullptr, nullptr, format, flags);
}

// ###########################################################################################################

bool InputFloatL(const char* label, float* v, float v_min, float v_max, float step, float step_fast, const char* format, ImGuiInputTextFlags flags)
{
    if (InputFloat(label, v, step, step_fast, format, flags))
    {
        *v = std::clamp(*v, v_min, v_max);
        return true;
    }
    return false;
}

bool InputFloat2L(const char* label, float v[2], float v_min, float v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputFloat2(label, v, format, flags))
    {
        for (size_t i = 0; i < 2; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputFloat3L(const char* label, float v[3], float v_min, float v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputFloat3(label, v, format, flags))
    {
        for (size_t i = 0; i < 3; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputFloat4L(const char* label, float v[4], float v_min, float v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputFloat4(label, v, format, flags))
    {
        for (size_t i = 0; i < 4; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputIntL(const char* label, int* v, int v_min, int v_max, int step, int step_fast, ImGuiInputTextFlags flags)
{
    if (InputInt(label, v, step, step_fast, flags))
    {
        *v = std::clamp(*v, v_min, v_max);
        return true;
    }
    return false;
}

bool InputInt2L(const char* label, int v[2], int v_min, int v_max, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputInt2(label, v, flags))
    {
        for (size_t i = 0; i < 2; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputInt3L(const char* label, int v[3], int v_min, int v_max, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputInt2(label, v, flags))
    {
        for (size_t i = 0; i < 3; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputInt4L(const char* label, int v[4], int v_min, int v_max, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputInt2(label, v, flags))
    {
        for (size_t i = 0; i < 4; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputDoubleL(const char* label, double* v, double v_min, double v_max, double step, double step_fast, const char* format, ImGuiInputTextFlags flags)
{
    if (InputDouble(label, v, step, step_fast, format, flags))
    {
        *v = std::clamp(*v, v_min, v_max);
        return true;
    }
    return false;
}

bool InputDouble2L(const char* label, double v[2], double v_min, double v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputDouble2(label, v, format, flags))
    {
        for (size_t i = 0; i < 2; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputDouble3L(const char* label, double v[3], double v_min, double v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputDouble3(label, v, format, flags))
    {
        for (size_t i = 0; i < 3; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

bool InputDouble4L(const char* label, double v[4], double v_min, double v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    if (InputDouble4(label, v, format, flags))
    {
        for (size_t i = 0; i < 4; i++)
        {
            v[i] = std::clamp(v[i], v_min, v_max);
        }

        return true;
    }
    return false;
}

} // namespace ImGui

bool operator==(const ImVec2& lhs, const ImVec2& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const ImVec2& lhs, const ImVec2& rhs)
{
    return lhs.x != rhs.x || lhs.y != rhs.y;
}

bool operator==(const ImVec4& lhs, const ImVec4& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w;
}

bool operator!=(const ImVec4& lhs, const ImVec4& rhs)
{
    return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z || lhs.w != rhs.w;
}