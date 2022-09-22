//------------------------------------------------------------------------------
// LICENSE
//   This software is dual-licensed to the public domain and under the following
//   license: you are granted a perpetual, irrevocable license to copy, modify,
//   publish, and distribute this file as you see fit.
//
// CREDITS
//   Written by Michal Cichon
//------------------------------------------------------------------------------
#include "BlueprintNodeBuilder.hpp"
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>

//------------------------------------------------------------------------------

ax::NodeEditor::Utilities::BlueprintNodeBuilder::BlueprintNodeBuilder(ImTextureID texture, int textureWidth, int textureHeight)
    : HeaderTextureId(texture), HeaderTextureWidth(textureWidth), HeaderTextureHeight(textureHeight) {}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::Begin(ax::NodeEditor::NodeId id)
{
    HasHeader = false;
    HeaderMin = HeaderMax = ImVec2();

    ax::NodeEditor::PushStyleVar(StyleVar_NodePadding, ImVec4(8, 4, 8, 8));

    ax::NodeEditor::BeginNode(id);

    ImGui::PushID(id.AsPointer());
    CurrentNodeId = id;

    SetStage(Stage::Begin);
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::End()
{
    SetStage(Stage::End);

    ax::NodeEditor::EndNode();

    if (ImGui::IsItemVisible())
    {
        auto alpha = static_cast<int>(255 * ImGui::GetStyle().Alpha);

        auto* drawList = ax::NodeEditor::GetNodeBackgroundDrawList(CurrentNodeId);

        const auto halfBorderWidth = ax::NodeEditor::GetStyle().NodeBorderWidth * 0.5F;

        auto headerColor = IM_COL32(0, 0, 0, alpha) | (HeaderColor & IM_COL32(255, 255, 255, 0));
        if ((HeaderMax.x > HeaderMin.x) && (HeaderMax.y > HeaderMin.y) && HeaderTextureId)
        {
            const auto uv = ImVec2(
                (HeaderMax.x - HeaderMin.x) / (4.0F * static_cast<float>(HeaderTextureWidth)),
                (HeaderMax.y - HeaderMin.y) / (4.0F * static_cast<float>(HeaderTextureHeight)));

            drawList->AddImageRounded(HeaderTextureId,
                                      HeaderMin - ImVec2(8 - halfBorderWidth, 4 - halfBorderWidth),
                                      HeaderMax + ImVec2(8 - halfBorderWidth, 0),
                                      ImVec2(0.0F, 0.0F), uv,
                                      headerColor, GetStyle().NodeRounding, 1 | 2);

            auto headerSeparatorMin = ImVec2(HeaderMin.x, HeaderMax.y);
            auto headerSeparatorMax = ImVec2(HeaderMax.x, HeaderMin.y);

            if ((headerSeparatorMax.x > headerSeparatorMin.x) && (headerSeparatorMax.y > headerSeparatorMin.y))
            {
                drawList->AddLine(
                    headerSeparatorMin + ImVec2(-(8 - halfBorderWidth), -0.5F),
                    headerSeparatorMax + ImVec2((8 - halfBorderWidth), -0.5F),
                    ImColor(255, 255, 255, 96 * alpha / (3 * 255)), 1.0F);
            }
        }
    }

    CurrentNodeId = 0;

    ImGui::PopID();

    ax::NodeEditor::PopStyleVar();

    SetStage(Stage::Invalid);
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::Header(const ImVec4& color)
{
    HeaderColor = ImColor(color);
    SetStage(Stage::Header);
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::EndHeader()
{
    SetStage(Stage::Content);
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::Input(ax::NodeEditor::PinId id)
{
    if (CurrentStage == Stage::Begin)
    {
        SetStage(Stage::Content);
    }

    const auto applyPadding = (CurrentStage == Stage::Input);

    SetStage(Stage::Input);

    if (applyPadding)
    {
        ImGui::Spring(0);
    }

    Pin(id, PinKind::Input);

    ImGui::BeginHorizontal(id.AsPointer());
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::EndInput()
{
    ImGui::EndHorizontal();

    EndPin();
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::Middle()
{
    if (CurrentStage == Stage::Begin)
    {
        SetStage(Stage::Content);
    }

    SetStage(Stage::Middle);
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::Output(ax::NodeEditor::PinId id)
{
    if (CurrentStage == Stage::Begin)
    {
        SetStage(Stage::Content);
    }

    const auto applyPadding = (CurrentStage == Stage::Output);

    SetStage(Stage::Output);

    if (applyPadding)
    {
        ImGui::Spring(0);
    }

    Pin(id, PinKind::Output);

    ImGui::BeginHorizontal(id.AsPointer());
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::EndOutput()
{
    ImGui::EndHorizontal();

    EndPin();
}

bool ax::NodeEditor::Utilities::BlueprintNodeBuilder::SetStage(Stage stage)
{
    if (stage == CurrentStage)
    {
        return false;
    }

    auto oldStage = CurrentStage;
    CurrentStage = stage;

    switch (oldStage)
    {
    case Stage::Begin:
        break;

    case Stage::Header:
        ImGui::EndHorizontal();
        HeaderMin = ImGui::GetItemRectMin();
        HeaderMax = ImGui::GetItemRectMax();

        // spacing between header and content
        ImGui::Spring(0, ImGui::GetStyle().ItemSpacing.y * 2.0F);

        break;

    case Stage::Content:
        break;

    case Stage::Input:
        ax::NodeEditor::PopStyleVar(2);

        ImGui::Spring(1, 0);
        ImGui::EndVertical();

        // #debug
        // ImGui::GetWindowDrawList()->AddRect(
        //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 255));

        break;

    case Stage::Middle:
        ImGui::EndVertical();

        // #debug
        // ImGui::GetWindowDrawList()->AddRect(
        //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 255));

        break;

    case Stage::Output:
        ax::NodeEditor::PopStyleVar(2);

        ImGui::Spring(1, 0);
        ImGui::EndVertical();

        // #debug
        // ImGui::GetWindowDrawList()->AddRect(
        //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 255));

        break;

    case Stage::End:
        // break;

    case Stage::Invalid:
        break;
    }

    switch (stage)
    {
    case Stage::Begin:
        ImGui::BeginVertical("node");
        break;

    case Stage::Header:
        HasHeader = true;

        ImGui::BeginHorizontal("header");
        break;

    case Stage::Content:
        if (oldStage == Stage::Begin)
        {
            ImGui::Spring(0);
        }

        ImGui::BeginHorizontal("content");
        ImGui::Spring(0, 0);
        break;

    case Stage::Input:
        ImGui::BeginVertical("inputs", ImVec2(0, 0), 0.0F);

        ax::NodeEditor::PushStyleVar(ax::NodeEditor::StyleVar_PivotAlignment, ImVec2(0, 0.5F));
        ax::NodeEditor::PushStyleVar(ax::NodeEditor::StyleVar_PivotSize, ImVec2(0, 0));

        if (!HasHeader)
        {
            ImGui::Spring(1, 0);
        }
        break;

    case Stage::Middle:
        ImGui::Spring(1);
        ImGui::BeginVertical("middle", ImVec2(0, 0), 1.0F);
        break;

    case Stage::Output:
        if (oldStage == Stage::Middle || oldStage == Stage::Input)
        {
            ImGui::Spring(1);
        }
        else
        {
            ImGui::Spring(1, 0);
        }
        ImGui::BeginVertical("outputs", ImVec2(0, 0), 1.0F);

        ax::NodeEditor::PushStyleVar(ax::NodeEditor::StyleVar_PivotAlignment, ImVec2(1.0F, 0.5F));
        ax::NodeEditor::PushStyleVar(ax::NodeEditor::StyleVar_PivotSize, ImVec2(0, 0));

        if (!HasHeader)
        {
            ImGui::Spring(1, 0);
        }
        break;

    case Stage::End:
        if (oldStage == Stage::Input)
        {
            ImGui::Spring(1, 0);
        }
        if (oldStage != Stage::Begin)
        {
            ImGui::EndHorizontal();
        }
        ContentMin = ImGui::GetItemRectMin();
        ContentMax = ImGui::GetItemRectMax();

        // ImGui::Spring(0);
        ImGui::EndVertical();
        NodeMin = ImGui::GetItemRectMin();
        NodeMax = ImGui::GetItemRectMax();
        break;

    case Stage::Invalid:
        break;
    }

    return true;
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::Pin(ax::NodeEditor::PinId id, ax::NodeEditor::PinKind kind)
{
    ax::NodeEditor::BeginPin(id, kind);
}

void ax::NodeEditor::Utilities::BlueprintNodeBuilder::EndPin()
{
    ax::NodeEditor::EndPin();

    // #debug
    // ImGui::GetWindowDrawList()->AddRectFilled(
    //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 64));
}
