/// @file imgui_ex.hpp
/// @brief ImGui extensions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-09-14

#pragma once

#include <imgui.h>

#include <limits>
#include <cstdint>

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

/// @brief Shows a Drag GUI element for 'int64'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragLong(const char* label, int64_t* v, float v_speed = 1.0F, int64_t v_min = 0.0, int64_t v_max = 0.0, const char* format = "%ld", ImGuiSliderFlags flags = 0);

/// @brief Shows a Drag GUI element for an array of 'int64[2]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragLong2(const char* label, int64_t v[2], float v_speed = 1.0F, int64_t v_min = 0.0, int64_t v_max = 0.0, const char* format = "%ld", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Drag GUI element for an array of 'int64[3]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragLong3(const char* label, int64_t v[3], float v_speed = 1.0F, int64_t v_min = 0.0, int64_t v_max = 0.0, const char* format = "%ld", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Drag GUI element for an array of 'int64[4]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragLong4(const char* label, int64_t v[4], float v_speed = 1.0F, int64_t v_min = 0.0, int64_t v_max = 0.0, const char* format = "%ld", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

// #####################################################################################################################

/// @brief Shows a Drag GUI element for 'uint64'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragULong(const char* label, uint64_t* v, float v_speed = 1.0F, uint64_t v_min = 0.0, uint64_t v_max = 0.0, const char* format = "%lu", ImGuiSliderFlags flags = 0);

/// @brief Shows a Drag GUI element for an array of 'uint64[2]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragULong2(const char* label, uint64_t v[2], float v_speed = 1.0F, uint64_t v_min = 0.0, uint64_t v_max = 0.0, const char* format = "%lu", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Drag GUI element for an array of 'uint64[3]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragULong3(const char* label, uint64_t v[3], float v_speed = 1.0F, uint64_t v_min = 0.0, uint64_t v_max = 0.0, const char* format = "%lu", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Drag GUI element for an array of 'uint64[4]'
/// @param[in] label Label to display beside the drag. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_speed Speed to drag the value
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool DragULong4(const char* label, uint64_t v[4], float v_speed = 1.0F, uint64_t v_min = 0.0, uint64_t v_max = 0.0, const char* format = "%lu", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

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

/// @brief Shows a Slider GUI element for 'int64'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderLong(const char* label, int64_t* v, int64_t v_min, int64_t v_max, const char* format = "%ld", ImGuiSliderFlags flags = 0);

/// @brief Shows a Slider GUI element for an array of 'int64[2]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderLong2(const char* label, int64_t v[2], int64_t v_min, int64_t v_max, const char* format = "%ld", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Slider GUI element for an array of 'int64[3]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderLong3(const char* label, int64_t v[3], int64_t v_min, int64_t v_max, const char* format = "%ld", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Slider GUI element for an array of 'int64[4]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderLong4(const char* label, int64_t v[4], int64_t v_min, int64_t v_max, const char* format = "%ld", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

// #####################################################################################################################

/// @brief Shows a Slider GUI element for 'uint64'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderULong(const char* label, uint64_t* v, uint64_t v_min, uint64_t v_max, const char* format = "%lu", ImGuiSliderFlags flags = 0);

/// @brief Shows a Slider GUI element for an array of 'uint64[2]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderULong2(const char* label, uint64_t v[2], uint64_t v_min, uint64_t v_max, const char* format = "%lu", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Slider GUI element for an array of 'uint64[3]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderULong3(const char* label, uint64_t v[3], uint64_t v_min, uint64_t v_max, const char* format = "%lu", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a Slider GUI element for an array of 'uint64[4]'
/// @param[in] label Label to display beside the slider. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum value allowed
/// @param[in] v_max Maximum value allowed
/// @param[in] format Printf format to display the value with
/// @param[in] flags Slider flags to modify the behavior
/// @return True if the value was changed
bool SliderULong4(const char* label, uint64_t v[4], uint64_t v_min, uint64_t v_max, const char* format = "%lu", ImGuiSliderFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

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

// ###########################################################################################################

/// @brief Shows a value limited InputText GUI element for 'float'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] step If greater than 0, this will show buttons to increase/decrease the value
/// @param[in] step_fast If greater than 0, this will show buttons to increase/decrease the value
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputFloatL(const char* label, float* v, float v_min = std::numeric_limits<float>::lowest(), float v_max = std::numeric_limits<float>::max(), float step = 0.0F, float step_fast = 0.0F, const char* format = "%.3f", ImGuiInputTextFlags flags = 0);

/// @brief Shows a value limited InputText GUI element for an array of 'float[2]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputFloat2L(const char* label, float v[2], float v_min = std::numeric_limits<float>::lowest(), float v_max = std::numeric_limits<float>::max(), const char* format = "%.3f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for an array of 'float[3]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputFloat3L(const char* label, float v[3], float v_min = std::numeric_limits<float>::lowest(), float v_max = std::numeric_limits<float>::max(), const char* format = "%.3f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for an array of 'float[4]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputFloat4L(const char* label, float v[4], float v_min = std::numeric_limits<float>::lowest(), float v_max = std::numeric_limits<float>::max(), const char* format = "%.3f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for 'int'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] step If greater than 0, this will show buttons to increase/decrease the value
/// @param[in] step_fast If greater than 0, this will show buttons to increase/decrease the value
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputIntL(const char* label, int* v, int v_min = std::numeric_limits<int>::lowest(), int v_max = std::numeric_limits<int>::max(), int step = 1, int step_fast = 100, ImGuiInputTextFlags flags = 0);

/// @brief Shows a value limited InputText GUI element for an array of 'int[2]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputInt2L(const char* label, int v[2], int v_min = std::numeric_limits<int>::lowest(), int v_max = std::numeric_limits<int>::max(), ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for an array of 'int[3]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputInt3L(const char* label, int v[3], int v_min = std::numeric_limits<int>::lowest(), int v_max = std::numeric_limits<int>::max(), ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for an array of 'int[4]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputInt4L(const char* label, int v[4], int v_min = std::numeric_limits<int>::lowest(), int v_max = std::numeric_limits<int>::max(), ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for 'double'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] step If greater than 0, this will show buttons to increase/decrease the value
/// @param[in] step_fast If greater than 0, this will show buttons to increase/decrease the value
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputDoubleL(const char* label, double* v, double v_min = std::numeric_limits<double>::lowest(), double v_max = std::numeric_limits<double>::max(), double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", ImGuiInputTextFlags flags = 0);

/// @brief Shows a value limited InputText GUI element for an array of 'double[2]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputDouble2L(const char* label, double v[2], double v_min = std::numeric_limits<double>::lowest(), double v_max = std::numeric_limits<double>::max(), const char* format = "%.3f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for an array of 'double[3]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputDouble3L(const char* label, double v[3], double v_min = std::numeric_limits<double>::lowest(), double v_max = std::numeric_limits<double>::max(), const char* format = "%.3f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

/// @brief Shows a value limited InputText GUI element for an array of 'double[4]'
/// @param[in] label Label to display beside the input. Has to be unique (use # to hide text afterwards to append an uid)
/// @param[in, out] v Pointer to the value to modify
/// @param[in] v_min Minimum to clamp the value to
/// @param[in] v_max Maximum to clamp the value to
/// @param[in] format Printf format to display the value with
/// @param[in] flags InputText flags to modify the behavior
/// @return True if the value was changed
bool InputDouble4L(const char* label, double v[4], double v_min = std::numeric_limits<double>::lowest(), double v_max = std::numeric_limits<double>::max(), const char* format = "%.3f", ImGuiInputTextFlags flags = 0); // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)

} // namespace ImGui

/// @brief Equal comparison operator
/// @param[in] lhs Left-hand side
/// @param[in] rhs Right-hand-side
/// @return True if the elements are equal
bool operator==(const ImVec4& lhs, const ImVec4& rhs);

/// @brief Unequal comparison operator
/// @param[in] lhs Left-hand side
/// @param[in] rhs Right-hand-side
/// @return True if the elements are unequal
bool operator!=(const ImVec4& lhs, const ImVec4& rhs);