// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TextAnsiColored.hpp"

#include <array>

namespace
{

constexpr int kMaxChar = 10000;
std::array<char, kMaxChar> char_buf;
std::array<ImU32, kMaxChar> col_buf;
std::array<bool, kMaxChar> char_skip;

} // namespace

namespace jet
{

std::vector<std::string> split(const std::string& str, const std::string& delim = " ")
{
    std::vector<std::string> res;
    std::size_t previous = 0;
    std::size_t current = str.find(delim);
    while (current != std::string::npos)
    {
        res.push_back(str.substr(previous, current - previous));
        previous = current + delim.length();
        current = str.find(delim, previous);
    }
    res.push_back(str.substr(previous, current - previous));
    return res;
}

} // namespace jet

namespace ImGui
{

bool ParseColor(const char* s, ImU32* col, int* skipChars)
{
    if (s[0] != '\033' || s[1] != '[')
    {
        return false;
    }

    if (s[2] == 'm')
    {
        *col = ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_Text));
        *skipChars = 3;
        return true;
    }

    if (s[2] == '0' && s[3] == 'm')
    {
        *col = ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(ImGuiCol_Text));
        *skipChars = 4;
        return true;
    }

    const char* seqEnd = &s[2];
    while (*seqEnd != 'm')
    {
        seqEnd++;
    }

    std::string seq{ &s[2], seqEnd };
    std::string colorStr;
    for (const auto& el : jet::split(seq, ";"))
    {
        if (el[0] == '3' && el.size() == 2)
        {
            colorStr = el;
            break;
        }
    }

    if (!colorStr.empty())
    {
        switch (colorStr[1])
        {
        case '0':
            *col = 0xffcccccc;
            break;
        case '1':
            *col = 0xff7a77f2;
            break;
        case '2':
            *col = 0xff99cc99;
            break;
        case '3':
            *col = 0xff66ccff;
            break;
        case '4':
            *col = 0xffcc9966;
            break;
        case '5':
            *col = 0xffcc99cc;
            break;
        case '6':
            *col = 0xffcccc66;
            break;
        case '7':
            *col = 0xff2d2d2d;
            break;
        default:
            return false;
        }
    }

    *skipChars = static_cast<int>(seqEnd - s + 1);
    return true;
}

void ImFont_RenderAnsiText(const ImFont* font,
                           ImDrawList* draw_list,
                           float size,
                           ImVec2 pos,
                           ImU32 col,
                           const ImVec4& clip_rect,
                           const char* text_begin,
                           const char* text_end,
                           float wrap_width = 0.0F,
                           bool cpu_fine_clip = false)
{
    if (!text_end)
    {
        // ImGui functions generally already provides a valid text_end,
        // so this is merely to handle direct calls.
        text_end = text_begin + strlen(text_begin);
    }

    // Align to be pixel perfect
    // Align to be pixel perfect
    pos.x = IM_FLOOR(pos.x);
    pos.y = IM_FLOOR(pos.y);
    float x = pos.x;
    float y = pos.y;
    if (y > clip_rect.w)
    {
        return;
    }

    const float scale = size / font->FontSize;
    const float line_height = font->FontSize * scale;
    const bool word_wrap_enabled = (wrap_width > 0.0F);
    const char* word_wrap_eol = nullptr;

    // Fast-forward to first visible line
    const char* s = text_begin;
    if (y + line_height < clip_rect.y && !word_wrap_enabled)
    {
        while (y + line_height < clip_rect.y && s < text_end)
        {
            s = static_cast<const char*>(memchr(s, '\n', static_cast<size_t>(text_end - s)));
            s = s ? s + 1 : text_end;
            y += line_height;
        }
    }

    // For large text, scan for the last visible line in order to avoid over-reserving in the call to PrimReserve()
    // Note that very large horizontal line will still be affected by the issue (e.g. a one megabyte string buffer
    // without a newline will likely crash atm)
    if (text_end - s > 10000 && !word_wrap_enabled)
    {
        const char* s_end = s;
        float y_end = y;
        while (y_end < clip_rect.w && s_end < text_end)
        {
            s_end = static_cast<const char*>(memchr(s_end, '\n', static_cast<size_t>(text_end - s_end)));
            s = s ? s + 1 : text_end;
            y_end += line_height;
        }
        text_end = s_end;
    }
    if (s == text_end)
    {
        return;
    }

    // Reserve vertices for remaining worse case (over-reserving is useful and easily amortized)
    const int vtx_count_max = static_cast<int>(text_end - s) * 4;
    const int idx_count_max = static_cast<int>(text_end - s) * 6;
    const int idx_expected_size = draw_list->IdxBuffer.Size + idx_count_max;
    draw_list->PrimReserve(idx_count_max, vtx_count_max);

    ImDrawVert* vtx_write = draw_list->_VtxWritePtr;
    ImDrawIdx* idx_write = draw_list->_IdxWritePtr;
    unsigned int vtx_current_idx = draw_list->_VtxCurrentIdx;

    {
        for (size_t i = 0; i < static_cast<size_t>(text_end - text_begin); i++)
        {
            char_skip.at(i) = false;
        }
        size_t index = 0;
        int skipChars = 0;
        const char* sLocal = s;
        ImU32 temp_col = col;
        while (sLocal < text_end)
        {
            if (sLocal < text_end - 4 && ParseColor(sLocal, &temp_col, &skipChars))
            {
                sLocal += skipChars;
                for (size_t i = 0; i < static_cast<size_t>(skipChars); i++)
                {
                    char_skip.at(index + i) = true;
                }
                index += static_cast<size_t>(skipChars);
            }
            else
            {
                char_buf.at(index) = *sLocal;
                col_buf.at(index) = temp_col;
                char_skip.at(index) = false;
                ++index;
                ++sLocal;
            }
        }
    }

    const char* s1 = s;
    while (s < text_end)
    {
        if (char_skip.at(static_cast<size_t>(s - s1)))
        {
            s++;
            continue;
        }
        if (word_wrap_enabled)
        {
            // Calculate how far we can render. Requires two passes on the string data but keeps the code simple and
            // not intrusive for what's essentially an uncommon feature.
            if (!word_wrap_eol)
            {
                word_wrap_eol = font->CalcWordWrapPositionA(scale, s, text_end, wrap_width - (x - pos.x));
                if (word_wrap_eol == s) // Wrap_width is too small to fit anything. Force displaying 1 character to
                {                       // minimize the height discontinuity.
                    word_wrap_eol++;    // +1 may not be a character start point in UTF-8 but it's ok because we use s
                }                       // >= word_wrap_eol below
            }

            if (s >= word_wrap_eol)
            {
                x = pos.x;
                y += line_height;
                word_wrap_eol = nullptr;

                // Wrapping skips upcoming blanks
                while (s < text_end)
                {
                    const char c = *s;
                    if (ImCharIsBlankA(c))
                    {
                        s++;
                    }
                    else if (c == '\n')
                    {
                        s++;
                        break;
                    }
                    else
                    {
                        break;
                    }
                }
                continue;
            }
        }

        // Decode and advance source
        auto c = static_cast<unsigned int>(static_cast<unsigned char>(*s));
        if (c < 0x80)
        {
            s += 1;
        }
        else
        {
            s += ImTextCharFromUtf8(&c, s, text_end);
            if (c == 0) // Malformed UTF-8?
            {
                break;
            }
        }

        if (c < 32)
        {
            if (c == '\n')
            {
                x = pos.x;
                y += line_height;
                if (y > clip_rect.w)
                {
                    break; // break out of main loop
                }
                continue;
            }
            if (c == '\r')
            {
                continue;
            }
        }

        float char_width = 0.0F;
        if (const ImFontGlyph* glyph = font->FindGlyph(static_cast<ImWchar>(c)))
        {
            char_width = glyph->AdvanceX * scale;

            // Arbitrarily assume that both space and tabs are empty glyphs as an optimization
            if (c != ' ' && c != '\t')
            {
                // We don't do a second finer clipping test on the Y axis as we've already skipped anything before
                // clip_rect.y and exit once we pass clip_rect.w
                float x1 = x + glyph->X0 * scale;
                float x2 = x + glyph->X1 * scale;
                float y1 = y + glyph->Y0 * scale;
                float y2 = y + glyph->Y1 * scale;
                if (x1 <= clip_rect.z && x2 >= clip_rect.x)
                {
                    // Render a character
                    float u1 = glyph->U0;
                    float v1 = glyph->V0;
                    float u2 = glyph->U1;
                    float v2 = glyph->V1;

                    // CPU side clipping used to fit text in their frame when the frame is too small. Only does
                    // clipping for axis aligned quads.
                    if (cpu_fine_clip)
                    {
                        if (x1 < clip_rect.x)
                        {
                            u1 = u1 + (1.0F - (x2 - clip_rect.x) / (x2 - x1)) * (u2 - u1);
                            x1 = clip_rect.x;
                        }
                        if (y1 < clip_rect.y)
                        {
                            v1 = v1 + (1.0F - (y2 - clip_rect.y) / (y2 - y1)) * (v2 - v1);
                            y1 = clip_rect.y;
                        }
                        if (x2 > clip_rect.z)
                        {
                            u2 = u1 + ((clip_rect.z - x1) / (x2 - x1)) * (u2 - u1);
                            x2 = clip_rect.z;
                        }
                        if (y2 > clip_rect.w)
                        {
                            v2 = v1 + ((clip_rect.w - y1) / (y2 - y1)) * (v2 - v1);
                            y2 = clip_rect.w;
                        }
                        if (y1 >= y2)
                        {
                            x += char_width;
                            continue;
                        }
                    }

                    // We are NOT calling PrimRectUV() here because non-inlined causes too much overhead in a debug
                    // builds. Inlined here:
                    ImU32 temp_col = col_buf.at(static_cast<size_t>(s - text_begin - 1));
                    {
                        idx_write[0] = vtx_current_idx;
                        idx_write[1] = vtx_current_idx + 1;
                        idx_write[2] = vtx_current_idx + 2;
                        idx_write[3] = vtx_current_idx;
                        idx_write[4] = vtx_current_idx + 2;
                        idx_write[5] = vtx_current_idx + 3;
                        vtx_write[0].pos.x = x1;
                        vtx_write[0].pos.y = y1;
                        vtx_write[0].col = temp_col;
                        vtx_write[0].uv.x = u1;
                        vtx_write[0].uv.y = v1;
                        vtx_write[1].pos.x = x2;
                        vtx_write[1].pos.y = y1;
                        vtx_write[1].col = temp_col;
                        vtx_write[1].uv.x = u2;
                        vtx_write[1].uv.y = v1;
                        vtx_write[2].pos.x = x2;
                        vtx_write[2].pos.y = y2;
                        vtx_write[2].col = temp_col;
                        vtx_write[2].uv.x = u2;
                        vtx_write[2].uv.y = v2;
                        vtx_write[3].pos.x = x1;
                        vtx_write[3].pos.y = y2;
                        vtx_write[3].col = temp_col;
                        vtx_write[3].uv.x = u1;
                        vtx_write[3].uv.y = v2;
                        vtx_write += 4;
                        vtx_current_idx += 4;
                        idx_write += 6;
                    }
                }
            }
        }

        x += char_width;
    }

    // Give back unused vertices
    draw_list->VtxBuffer.resize(static_cast<int>(vtx_write - draw_list->VtxBuffer.Data));
    draw_list->IdxBuffer.resize(static_cast<int>(idx_write - draw_list->IdxBuffer.Data));
    draw_list->CmdBuffer[draw_list->CmdBuffer.Size - 1].ElemCount -= static_cast<unsigned int>(idx_expected_size - draw_list->IdxBuffer.Size);
    draw_list->_VtxWritePtr = vtx_write;
    draw_list->_IdxWritePtr = idx_write;
    draw_list->_VtxCurrentIdx = static_cast<unsigned int>(draw_list->VtxBuffer.Size);
}

void ImDrawList_AddAnsiText(ImDrawList* drawList,
                            const ImFont* font,
                            float font_size,
                            const ImVec2& pos,
                            ImU32 col,
                            const char* text_begin,
                            const char* text_end = nullptr,
                            float wrap_width = 0.0F,
                            const ImVec4* cpu_fine_clip_rect = nullptr)
{
    if ((col & IM_COL32_A_MASK) == 0)
    {
        return;
    }

    if (text_end == nullptr)
    {
        text_end = text_begin + strlen(text_begin);
    }
    if (text_begin == text_end)
    {
        return;
    }

    // Pull default font/size from the shared ImDrawListSharedData instance
    if (font == nullptr)
    {
        font = drawList->_Data->Font;
    }
    if (font_size == 0.0F)
    {
        font_size = drawList->_Data->FontSize;
    }

    IM_ASSERT(font->ContainerAtlas->TexID == drawList->_TextureIdStack.back()); // Use high-level ImGui::PushFont()
                                                                                // or low-level
                                                                                // ImDrawList::PushTextureId() to
                                                                                // change font.

    ImVec4 clip_rect = drawList->_ClipRectStack.back();
    if (cpu_fine_clip_rect)
    {
        clip_rect.x = ImMax(clip_rect.x, cpu_fine_clip_rect->x);
        clip_rect.y = ImMax(clip_rect.y, cpu_fine_clip_rect->y);
        clip_rect.z = ImMin(clip_rect.z, cpu_fine_clip_rect->z);
        clip_rect.w = ImMin(clip_rect.w, cpu_fine_clip_rect->w);
    }
    ImFont_RenderAnsiText(font,
                          drawList,
                          font_size,
                          pos,
                          col,
                          clip_rect,
                          text_begin,
                          text_end,
                          wrap_width,
                          cpu_fine_clip_rect != nullptr);
}

void RenderAnsiText(ImVec2 pos, const char* text, const char* text_end, bool hide_text_after_hash)
{
    ImGuiContext& g = *GImGui;
    ImGuiWindow* window = g.CurrentWindow;

    // Hide anything after a '##' string
    const char* text_display_end = nullptr;
    if (hide_text_after_hash)
    {
        text_display_end = FindRenderedTextEnd(text, text_end);
    }
    else
    {
        if (!text_end)
        {
            text_end = text + strlen(text);
        }
        text_display_end = text_end;
    }

    if (text != text_display_end)
    {
        ImDrawList_AddAnsiText(window->DrawList, g.Font, g.FontSize, pos, GetColorU32(ImGuiCol_Text), text, text_display_end);
        if (g.LogEnabled)
        {
            LogRenderedText(&pos, text, text_display_end);
        }
    }
}

void RenderAnsiTextWrapped(ImVec2 pos, const char* text, const char* text_end, float wrap_width)
{
    ImGuiContext& g = *GImGui;
    ImGuiWindow* window = g.CurrentWindow;

    if (!text_end)
    {
        text_end = text + strlen(text);
    }

    if (text != text_end)
    {
        ImDrawList_AddAnsiText(window->DrawList, g.Font, g.FontSize, pos, GetColorU32(ImGuiCol_Text), text, text_end, wrap_width);
        if (g.LogEnabled)
        {
            LogRenderedText(&pos, text, text_end);
        }
    }
}

} // namespace ImGui

void ImGui::TextAnsiUnformatted(const char* text, const char* text_end)
{
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
    {
        return;
    }

    ImGuiContext& g = *GImGui;
    IM_ASSERT(text != nullptr);
    const char* text_begin = text;
    if (text_end == nullptr)
    {
        text_end = text + strlen(text); // NOLINT(clang-analyzer-core.NonNullParamChecker) - false positive, as checked by IM_ASSERT
    }

    const ImVec2 text_pos(window->DC.CursorPos.x, window->DC.CursorPos.y + window->DC.CurrLineTextBaseOffset);
    const float wrap_pos_x = window->DC.TextWrapPos;
    const bool wrap_enabled = wrap_pos_x >= 0.0F;
    if (text_end - text > 2000 && !wrap_enabled)
    {
        // Long text!
        // Perform manual coarse clipping to optimize for long multi-line text
        // - From this point we will only compute the width of lines that are visible. Optimization only available
        // when word-wrapping is disabled.
        // - We also don't vertically center the text within the line full height, which is unlikely to matter
        // because we are likely the biggest and only item on the line.
        // - We use memchr(), pay attention that well optimized versions of those str/mem functions are much faster
        // than a casually written loop.
        const char* line = text;
        const float line_height = GetTextLineHeight();
        const ImRect clip_rect = window->ClipRect;
        ImVec2 text_size(0, 0);

        if (text_pos.y <= clip_rect.Max.y)
        {
            ImVec2 pos = text_pos;

            // Lines to skip (can't skip when logging text)
            if (!g.LogEnabled)
            {
                int lines_skippable = static_cast<int>((clip_rect.Min.y - text_pos.y) / line_height);
                if (lines_skippable > 0)
                {
                    int lines_skipped = 0;
                    while (line < text_end && lines_skipped < lines_skippable)
                    {
                        const char* line_end = static_cast<const char*>(memchr(line, '\n', static_cast<size_t>(text_end - line)));
                        if (!line_end)
                        {
                            line_end = text_end;
                        }
                        line = line_end + 1;
                        lines_skipped++;
                    }
                    pos.y += static_cast<float>(lines_skipped) * line_height;
                }
            }

            // Lines to render
            if (line < text_end)
            {
                ImRect line_rect(pos, pos + ImVec2(FLT_MAX, line_height));
                while (line < text_end)
                {
                    if (IsClippedEx(line_rect, 0))
                    {
                        break;
                    }

                    const char* line_end = static_cast<const char*>(memchr(line, '\n', static_cast<size_t>(text_end - line)));
                    if (!line_end)
                    {
                        line_end = text_end;
                    }
                    const ImVec2 line_size = CalcTextSize(line, line_end, false);
                    text_size.x = ImMax(text_size.x, line_size.x);
                    RenderAnsiText(pos, line, line_end, false);
                    line = line_end + 1;
                    line_rect.Min.y += line_height;
                    line_rect.Max.y += line_height;
                    pos.y += line_height;
                }

                // Count remaining lines
                int lines_skipped = 0;
                while (line < text_end)
                {
                    const char* line_end = static_cast<const char*>(memchr(line, '\n', static_cast<size_t>(text_end - line)));
                    if (!line_end)
                    {
                        line_end = text_end;
                    }
                    line = line_end + 1;
                    lines_skipped++;
                }
                pos.y += static_cast<float>(lines_skipped) * line_height;
            }

            text_size.y += (pos - text_pos).y;
        }

        ImRect bb(text_pos, text_pos + text_size);
        ItemSize(bb);
        ItemAdd(bb, 0);
    }
    else
    {
        const float wrap_width = wrap_enabled ? CalcWrapWidthForPos(window->DC.CursorPos, wrap_pos_x) : 0.0F;
        const ImVec2 text_size = CalcTextSize(text_begin, text_end, false, wrap_width);

        // Account of baseline offset
        ImRect bb(text_pos, text_pos + text_size);
        ItemSize(text_size);
        if (!ItemAdd(bb, 0))
        {
            return;
        }

        // Render (we don't hide text after ## in this end-user function)
        RenderAnsiTextWrapped(bb.Min, text_begin, text_end, wrap_width);
    }
}

void ImGui::TextAnsiV(const char* fmt, va_list args)
{
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
    {
        return;
    }

    ImGuiContext& g = *GImGui;
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat-nonliteral"
#endif
    const char* text_end = g.TempBuffer.Data + ImFormatStringV(g.TempBuffer.Data, strlen(g.TempBuffer.Data), fmt, args); // NOLINT(clang-diagnostic-format-nonliteral)
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic pop
#endif
    TextAnsiUnformatted(g.TempBuffer.Data, text_end);
}

void ImGui::TextAnsiColoredV(const ImVec4& col, const char* fmt, va_list args)
{
    PushStyleColor(ImGuiCol_Text, col);
    TextAnsiV(fmt, args);
    PopStyleColor();
}

void ImGui::TextAnsiColored(const ImVec4& col, const char* fmt, ...) // NOLINT(cert-dcl50-cpp)
{
    va_list args;
    va_start(args, fmt);
    TextAnsiColoredV(col, fmt, args);
    va_end(args);
}
