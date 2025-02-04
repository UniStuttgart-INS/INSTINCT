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

/// @brief Node Builder class
class BlueprintNodeBuilder
{
  public:
    /// @brief Constructor
    /// @param[in] texture Pointer to the texture to use for the node
    /// @param[in] textureWidth Width of the provided texture
    /// @param[in] textureHeight Height of the provided texture
    explicit BlueprintNodeBuilder(ImTextureID texture = nullptr, int textureWidth = 0, int textureHeight = 0);

    /// @brief Begins building a node
    /// @param[in] id Id of the node to build
    void Begin(ax::NodeEditor::NodeId id);

    /// @brief Ends building a node
    void End();

    /// @brief Begins building the header
    /// @param[in] color Color of the header
    void Header(const ImVec4& color = ImVec4(1, 1, 1, 1));

    /// @brief Ends building the header
    void EndHeader();

    /// @brief Begins building an input pin
    /// @param[in] id Id of the pin to build
    void Input(ax::NodeEditor::PinId id);
    /// @brief Ends building the input pin
    static void EndInput();

    /// @brief Begins building of the middle of the node
    void Middle();

    /// @brief Begins building an output pin
    /// @param[in] id Id of the pin to build
    void Output(ax::NodeEditor::PinId id);
    /// @brief Ends building the output pin
    static void EndOutput();

  private:
    /// @brief Stages in the build process
    enum class Stage : uint8_t
    {
        Invalid, ///< Invalid stage
        Begin,   ///< Beginning of node construction
        Header,  ///< Currently building the header
        Content, ///< Currently building the content
        Input,   ///< Currently building an input pin
        Output,  ///< Currently building an output pin
        Middle,  ///< Currently building the middle
        End      ///< End of node construction
    };

    /// @brief Set the stage of the node build process. Takes care of all the Layout elements
    /// @param[in] stage Stage to set
    /// @return True if stage was set correctly
    bool SetStage(Stage stage);

    /// @brief Begins building a pin
    /// @param[in] id Id of the pin to build
    /// @param[in] kind Kind of the pin (input/output)
    static void Pin(ax::NodeEditor::PinId id, ax::NodeEditor::PinKind kind);
    /// @brief Ends building the pin
    static void EndPin();

    ImTextureID HeaderTextureId;                    ///< @brief Pointer to the texture to use for the header
    int HeaderTextureWidth;                         ///< Width of the header texture
    int HeaderTextureHeight;                        ///< Height of the header texture
    NodeId CurrentNodeId = 0;                       ///< Id of the node currently built
    Stage CurrentStage = Stage::Invalid;            ///< Current stage of the build process
    ImU32 HeaderColor = IM_COL32(255, 255, 255, 0); ///< Color of the header
    ImVec2 NodeMin;                                 ///< Minimum size of the node
    ImVec2 NodeMax;                                 ///< Maximum size of the node
    ImVec2 HeaderMin;                               ///< Minimum size of the header
    ImVec2 HeaderMax;                               ///< Maximum size of the header
    ImVec2 ContentMin;                              ///< Minimum size of the content
    ImVec2 ContentMax;                              ///< Maximum size of the content
    bool HasHeader = false;                         ///< Flag whether the node has a header
};

//------------------------------------------------------------------------------
} // namespace ax::NodeEditor::Utilities