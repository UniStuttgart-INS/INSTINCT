/// @file Node.hpp
/// @brief Node Class
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui.h>
#include <imgui_node_editor.h>
#include <imgui_stdlib.h>

#include "internal/Pin.hpp"

#include "util/Logger.hpp"

#include <string>
#include <vector>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace NAV
{
class NodeData;

namespace gui
{
class NodeEditorApplication;
} // namespace gui

class Node
{
  public:
    /// Kind information class
    struct Kind
    {
        enum Value : uint8_t
        {
            Blueprint,
            Simple,
            GroupBox,
        };

        Kind() = default;

        //NOLINTNEXTLINE(hicpp-explicit-conversions, google-explicit-constructor)
        constexpr Kind(Value kind)
            : value(kind) {}

        explicit Kind(const std::string& string)
        {
            if (string == "Blueprint")
            {
                value = Kind::Blueprint;
            }
            else if (string == "Simple")
            {
                value = Kind::Simple;
            }
            else if (string == "GroupBox")
            {
                value = Kind::GroupBox;
            }
        }

        explicit operator Value() const { return value; } // Allow switch(Node::Value(kind)) and comparisons.
        explicit operator bool() = delete;                // Prevent usage: if(fruit)
        Kind& operator=(Value v)
        {
            value = v;
            return *this;
        }

        friend constexpr bool operator==(const Node::Kind& lhs, const Node::Kind& rhs);
        friend constexpr bool operator!=(const Node::Kind& lhs, const Node::Kind& rhs);

        friend constexpr bool operator==(const Node::Kind& lhs, const Node::Kind::Value& rhs);
        friend constexpr bool operator==(const Node::Kind::Value& lhs, const Node::Kind& rhs);
        friend constexpr bool operator!=(const Node::Kind& lhs, const Node::Kind::Value& rhs);
        friend constexpr bool operator!=(const Node::Kind::Value& lhs, const Node::Kind& rhs);

        explicit operator std::string() const
        {
            switch (value)
            {
            case Kind::Blueprint:
                return "Blueprint";
            case Kind::Simple:
                return "Simple";
            case Kind::GroupBox:
                return "GroupBox";
            }
            return "";
        }

      private:
        Value value;
    };

    /// @brief Default constructor
    Node() = default;
    /// @brief Destructor
    virtual ~Node() = default;
    /// @brief Copy constructor
    Node(const Node&) = delete;
    /// @brief Move constructor
    Node(Node&&) = delete;
    /// @brief Copy assignment operator
    Node& operator=(const Node&) = delete;
    /// @brief Move assignment operator
    Node& operator=(Node&&) = delete;

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Interface                                                */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief String representation of the Class Type
    [[nodiscard]] virtual std::string type() const = 0;

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true
    virtual void guiConfig();

    /// @brief Saves the node into a json object
    [[nodiscard]] virtual json save() const = 0;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    virtual void restore(const json& j) = 0;

    /// @brief Initialize the Node
    bool initializeNode();

    /// @brief Deinitialize the Node
    void deinitializeNode();

    /// @brief Resets the node. In case of file readers, that moves the read cursor to the start
    virtual void resetNode();

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    virtual bool onCreateLink(Pin* startPin, Pin* endPin);

    /// @brief Called when a link is to be deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    virtual void onDeleteLink(Pin* startPin, Pin* endPin);

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member functions                                             */
    /* -------------------------------------------------------------------------------------------------------- */

    template<typename T>
    [[nodiscard]] T* getInputValue(size_t portIndex)
    {
        // clang-format off
        if constexpr (std::is_same_v<T, bool>
                   || std::is_same_v<T, int>
                   || std::is_same_v<T, float>
                   || std::is_same_v<T, double>
                   || std::is_same_v<T, std::string>)
        { // clang-format on
            if (auto* pval = std::get_if<T*>(&inputPins.at(portIndex).data))
            {
                return *pval;
            }
        }
        else // constexpr
        {
            if (auto* pval = std::get_if<void*>(&inputPins.at(portIndex).data))
            {
                return static_cast<T*>(*pval);
            }
        }

        return nullptr;
    }

    template<typename T>
    [[nodiscard]] T* getInputValue(size_t portIndex) const
    {
        // clang-format off
        if constexpr (std::is_same_v<T, bool>
                   || std::is_same_v<T, int>
                   || std::is_same_v<T, float>
                   || std::is_same_v<T, double>
                   || std::is_same_v<T, std::string>)
        { // clang-format on
            if (const auto* pval = std::get_if<T*>(&inputPins.at(portIndex).data))
            {
                return *pval;
            }
        }
        else // constexpr
        {
            if (const auto* pval = std::get_if<void*>(&inputPins.at(portIndex).data))
            {
                return static_cast<T*>(*pval);
            }
        }

        return nullptr;
    }

    /// @brief Calls all registered callbacks on the specified output port
    /// @param[in] portIndex Output port where to call the callbacks
    /// @param[in] data The data to pass to the callback targets
    void invokeCallbacks(size_t portIndex, const std::shared_ptr<NodeData>& data);

    template<typename T, class... Args>
    T callInputFunction(size_t portIndex, const Args&... args)
    {
        if (auto* function = std::get_if<std::pair<Node*, void (Node::*)()>>(&inputPins.at(portIndex).data))
        {
            Node* node = function->first;
            auto callbackProto = function->second;

#pragma GCC diagnostic push
#if defined(__GNUC__) && !defined(__clang__)
    #pragma GCC diagnostic ignored "-Wcast-function-type" // NOLINT
#endif
            auto callback = reinterpret_cast<T (Node::*)(Args...)>(callbackProto);
#pragma GCC diagnostic pop

            return (node->*callback)(std::forward<const Args>(args)...);
        }

        return T{};
    }

    /// @brief Returns the index of the pin
    /// @param[in] pinId Id of the Pin
    /// @return The index of the pin
    [[nodiscard]] size_t pinIndexFromId(ax::NodeEditor::PinId pinId) const;

    /// @brief Node name and id
    [[nodiscard]] std::string nameId() const
    {
        return fmt::format("{} ({})", name, size_t(id));
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Unique Id of the Node
    ax::NodeEditor::NodeId id = 0;
    /// Kind of the Node
    Kind kind = Kind::Blueprint;
    /// Name of the Node
    std::string name;
    /// List of input pins
    std::vector<Pin> inputPins;
    /// List of output pins
    std::vector<Pin> outputPins;
    /// Color of the node
    ImColor color{ 255, 255, 255 };
    /// Size of the node in pixels
    ImVec2 size{ 0, 0 };
    /// Flag if the config window is shown
    bool showConfig = false;

    /// Flag if the config window should be shown
    bool hasConfig = false;
    /// Node disabled Shortcuts
    bool nodeDisabledShortcuts = false;

    /// Enables the callbacks
    bool callbacksEnabled = false;

    /// @brief Flag, if the node is initialized
    [[nodiscard]] bool isInitialized() const;

    /// @brief Flag, if the node is currently initializing
    [[nodiscard]] bool isInitializing() const;

    /// @brief Flag, if the node is currently deinitializing
    [[nodiscard]] bool isDeinitializing() const;

  private:
    /// @brief Abstract Initialization of the Node
    virtual bool initialize();

    /// @brief Deinitialize the Node
    virtual void deinitialize();

    /// Flag, if the node is initialized
    bool isInitialized_ = false;

    /// Flag, if the node is currently initializing
    bool isInitializing_ = false;
    /// Flag, if the node is currently deinitializing
    bool isDeinitializing_ = false;

    friend class gui::NodeEditorApplication;
};

constexpr bool operator==(const Node::Kind& lhs, const Node::Kind& rhs) { return lhs.value == rhs.value; }
constexpr bool operator!=(const Node::Kind& lhs, const Node::Kind& rhs) { return lhs.value != rhs.value; }

constexpr bool operator==(const Node::Kind& lhs, const Node::Kind::Value& rhs) { return lhs.value == rhs; }
constexpr bool operator==(const Node::Kind::Value& lhs, const Node::Kind& rhs) { return lhs == rhs.value; }
constexpr bool operator!=(const Node::Kind& lhs, const Node::Kind::Value& rhs) { return lhs.value != rhs; }
constexpr bool operator!=(const Node::Kind::Value& lhs, const Node::Kind& rhs) { return lhs != rhs.value; }

} // namespace NAV