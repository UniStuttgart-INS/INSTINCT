/// @file imgui_ex.hpp
/// @brief ImGui extensions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-09-14

#pragma once

#include <imgui.h>

namespace ImGui
{
// Widgets: Drag Sliders
// - CTRL+Click on any drag box to turn them into an input box. Manually input values aren't clamped and can go off-bounds.
// - For all the Float2/Float3/Float4/Int2/Int3/Int4 versions of every functions, note that a 'float v[X]' function
//   argument is the same as 'float* v', the array syntax is just a way to document the number of elements that are
//   expected to be accessible. You can pass address of your first element out of a contiguous set, e.g. &myvector.x
// - Adjust format string to decorate the value with a prefix, a suffix, or adapt the editing and display precision
//   e.g. "%.3f" -> 1.234; "%5.2f secs" -> 01.23 secs; "Biscuit: %.0f" -> Biscuit: 1; etc.
// - Format string may also be set to NULL or use the default format ("%f" or "%d").
// - Speed are per-pixel of mouse movement (v_speed=0.2f: mouse needs to move by 5 pixels to increase value by 1).
//   For gamepad/keyboard navigation, minimum speed is Max(v_speed, minimum_step_at_given_precision).
// - Use v_min < v_max to clamp edits to given limits. Note that CTRL+Click manual input can override those limits.
// - Use v_max = FLT_MAX / INT_MAX etc to avoid clamping to a maximum, same with v_min = -FLT_MAX / INT_MIN to avoid clamping to a minimum.
// - We use the same sets of flags for DragXXX() and SliderXXX() functions as the features are the same and it makes it easier to swap them.
// - Legacy: Pre-1.78 there are DragXXX() function signatures that takes a final `float power=1.0f' argument instead of the `ImGuiSliderFlags flags=0' argument.
//   If you get a warning converting a float to ImGuiSliderFlags, read https://github.com/ocornut/imgui/issues/3361
// - If v_min >= v_max we have no bound

/// @brief Shows a Drag GUI element for 'double'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragDouble(const char* label, double* v, float v_speed = 1.0F, double v_min = 0.0, double v_max = 0.0, const char* format = "%.6f", ImGuiSliderFlags flags = 0);

/// @brief Shows a Drag GUI element for an array of 'double[2]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragDouble2(const char* label, double v[2], float v_speed = 1.0F, double v_min = 0.0, double v_max = 0.0, const char* format = "%.6f", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Drag GUI element for an array of 'double[3]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragDouble3(const char* label, double v[3], float v_speed = 1.0F, double v_min = 0.0, double v_max = 0.0, const char* format = "%.6f", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Drag GUI element for an array of 'double[4]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragDouble4(const char* label, double v[4], float v_speed = 1.0F, double v_min = 0.0, double v_max = 0.0, const char* format = "%.6f", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

// #####################################################################################################################

// Widgets: Regular Sliders
// - CTRL+Click on any slider to turn them into an input box. Manually input values aren't clamped and can go off-bounds.
// - Adjust format string to decorate the value with a prefix, a suffix, or adapt the editing and display precision e.g. "%.3f" -> 1.234; "%5.2f secs" -> 01.23 secs; "Biscuit: %.0f" -> Biscuit: 1; etc.
// - Format string may also be set to NULL or use the default format ("%f" or "%d").
// - Legacy: Pre-1.78 there are SliderXXX() function signatures that takes a final `float power=1.0f' argument instead of the `ImGuiSliderFlags flags=0' argument.
//   If you get a warning converting a float to ImGuiSliderFlags, read https://github.com/ocornut/imgui/issues/3361

/// @brief Shows a Slider GUI element for 'double'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderDouble(const char* label, double* v, double v_min, double v_max, const char* format = "%.6f", ImGuiSliderFlags flags = 0);

/// @brief Shows a Slider GUI element for an array of 'double[2]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderDouble2(const char* label, double v[2], double v_min, double v_max, const char* format = "%.6f", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Slider GUI element for an array of 'double[3]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderDouble3(const char* label, double v[3], double v_min, double v_max, const char* format = "%.6f", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Slider GUI element for an array of 'double[4]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderDouble4(const char* label, double v[4], double v_min, double v_max, const char* format = "%.6f", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

// #####################################################################################################################

// Widgets: Input with Keyboard
// - If you want to use InputText() with std::string or any custom dynamic string type, see misc/cpp/imgui_stdlib.h and comments in imgui_demo.cpp.
// - Most of the ImGuiInputTextFlags flags are only useful for InputText() and not for InputFloatX, InputIntX, InputDouble etc.

/// @brief Shows an InputText GUI element for an array of 'double[2]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputDouble2(const char* label, double v[2], const char* format = "%.6f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows an InputText GUI element for an array of 'double[3]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputDouble3(const char* label, double v[3], const char* format = "%.6f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows an InputText GUI element for an array of 'double[4]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputDouble4(const char* label, double v[4], const char* format = "%.6f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

} // namespace ImGui