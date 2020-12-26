/// @file Splitter.hpp
/// @brief Screen Divider
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

namespace NAV::gui::widgets
{
/// @brief Vertical or horizontal Screen Divider
/// @param[in] split_vertically Vertical or horizontal splitter
/// @param[in] thickness Thickness in Pixels
/// @param[in, out] size1 Size left or above the splitter
/// @param[in, out] size2 Size right or below the splitter
/// @param[in] min_size1 Minimum size left or above the splitter
/// @param[in] min_size2 Minimum size right or below the spliter
/// @param[in] splitter_long_axis_size Length of the splitter
/// @return True when the splitter is moved
bool Splitter(bool split_vertically, float thickness,
              float* size1, float* size2,
              float min_size1, float min_size2,
              float splitter_long_axis_size = -1.0F);

} // namespace NAV::gui::widgets
