#include "imgui_ex.hpp"

#include <algorithm>

// ###########################################################################################################

bool ImGui::DragDouble(const char* label, double* v, float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags)
{
    return DragScalar(label, ImGuiDataType_Double, v, v_speed, &v_min, &v_max, format, flags);
}

bool ImGui::DragDouble2(const char* label, double v[2], float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_Double, v, 2, v_speed, &v_min, &v_max, format, flags);
}

bool ImGui::DragDouble3(const char* label, double v[3], float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_Double, v, 3, v_speed, &v_min, &v_max, format, flags);
}

bool ImGui::DragDouble4(const char* label, double v[4], float v_speed, double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return DragScalarN(label, ImGuiDataType_Double, v, 4, v_speed, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool ImGui::SliderDouble(const char* label, double* v, double v_min, double v_max, const char* format, ImGuiSliderFlags flags)
{
    return SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max, format, flags);
}

bool ImGui::SliderDouble2(const char* label, double v[2], double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_Double, v, 2, &v_min, &v_max, format, flags);
}

bool ImGui::SliderDouble3(const char* label, double v[3], double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_Double, v, 3, &v_min, &v_max, format, flags);
}

bool ImGui::SliderDouble4(const char* label, double v[4], double v_min, double v_max, const char* format, ImGuiSliderFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return SliderScalarN(label, ImGuiDataType_Double, v, 4, &v_min, &v_max, format, flags);
}

// ###########################################################################################################

bool ImGui::InputDouble2(const char* label, double v[2], const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return InputScalarN(label, ImGuiDataType_Double, v, 2, nullptr, nullptr, format, flags);
}

bool ImGui::InputDouble3(const char* label, double v[3], const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return InputScalarN(label, ImGuiDataType_Double, v, 3, nullptr, nullptr, format, flags);
}

bool ImGui::InputDouble4(const char* label, double v[4], const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
{
    return InputScalarN(label, ImGuiDataType_Double, v, 4, nullptr, nullptr, format, flags);
}

// ###########################################################################################################

bool ImGui::InputFloatL(const char* label, float* v, float v_min, float v_max, float step, float step_fast, const char* format, ImGuiInputTextFlags flags)
{
    if (InputFloat(label, v, step, step_fast, format, flags))
    {
        *v = std::clamp(*v, v_min, v_max);
        return true;
    }
    return false;
}

bool ImGui::InputFloat2L(const char* label, float v[2], float v_min, float v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputFloat3L(const char* label, float v[3], float v_min, float v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputFloat4L(const char* label, float v[4], float v_min, float v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputIntL(const char* label, int* v, int v_min, int v_max, int step, int step_fast, ImGuiInputTextFlags flags)
{
    if (InputInt(label, v, step, step_fast, flags))
    {
        *v = std::clamp(*v, v_min, v_max);
        return true;
    }
    return false;
}

bool ImGui::InputInt2L(const char* label, int v[2], int v_min, int v_max, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputInt3L(const char* label, int v[3], int v_min, int v_max, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputInt4L(const char* label, int v[4], int v_min, int v_max, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputDoubleL(const char* label, double* v, double v_min, double v_max, double step, double step_fast, const char* format, ImGuiInputTextFlags flags)
{
    if (InputDouble(label, v, step, step_fast, format, flags))
    {
        *v = std::clamp(*v, v_min, v_max);
        return true;
    }
    return false;
}

bool ImGui::InputDouble2L(const char* label, double v[2], double v_min, double v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputDouble3L(const char* label, double v[3], double v_min, double v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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

bool ImGui::InputDouble4L(const char* label, double v[4], double v_min, double v_max, const char* format, ImGuiInputTextFlags flags) // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
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