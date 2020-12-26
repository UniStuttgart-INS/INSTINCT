//------------------------------------------------------------------------------
// LICENSE
//   This software is dual-licensed to the public domain and under the following
//   license: you are granted a perpetual, irrevocable license to copy, modify,
//   publish, and distribute this file as you see fit.
//
// CREDITS
//   Written by Michal Cichon
//------------------------------------------------------------------------------
#pragma once

//------------------------------------------------------------------------------
#include <imgui_node_editor.h>

//------------------------------------------------------------------------------
namespace ax::NodeEditor::Utilities
{
//------------------------------------------------------------------------------
struct BlueprintNodeBuilder
{
    explicit BlueprintNodeBuilder(ImTextureID texture = nullptr, int textureWidth = 0, int textureHeight = 0);

    void Begin(NodeId id);
    void End();

    void Header(const ImVec4& color = ImVec4(1, 1, 1, 1));
    void EndHeader();

    void Input(PinId id);
    static void EndInput();

    void Middle();

    void Output(PinId id);
    static void EndOutput();

  private:
    enum class Stage
    {
        Invalid,
        Begin,
        Header,
        Content,
        Input,
        Output,
        Middle,
        End
    };

    bool SetStage(Stage stage);

    static void Pin(PinId id, ax::NodeEditor::PinKind kind);
    static void EndPin();

    ImTextureID HeaderTextureId;
    int HeaderTextureWidth;
    int HeaderTextureHeight;
    NodeId CurrentNodeId = 0;
    Stage CurrentStage = Stage::Invalid;
    ImU32 HeaderColor = IM_COL32(255, 255, 255, 0);
    ImVec2 NodeMin;
    ImVec2 NodeMax;
    ImVec2 HeaderMin;
    ImVec2 HeaderMax;
    ImVec2 ContentMin;
    ImVec2 ContentMax;
    bool HasHeader = false;
};

//------------------------------------------------------------------------------
} // namespace ax::NodeEditor::Utilities