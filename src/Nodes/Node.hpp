/// @file Node.hpp
/// @brief Node Class
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include "internal/Pin.hpp"

#include "util/Logger.hpp"

#include <string>
#include <vector>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace NAV
{
class NodeData;

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

        constexpr bool operator==(const Kind& other) const { return value == other.value; }
        constexpr bool operator!=(const Kind& other) const { return value != other.value; }

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
    virtual void config() = 0;

    /// @brief Saves the node into a json object
    [[nodiscard]] virtual json save() const = 0;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    virtual void restore(const json& j) = 0;

    /// @brief Initialize the Node
    virtual void initialize();

    /// @brief Deinitialize the Node
    virtual void deinitialize();

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member functions                                             */
    /* -------------------------------------------------------------------------------------------------------- */

    template<typename T>
    T* getInputValue(size_t portIndex)
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
    T* getInputValue(size_t portIndex) const
    {
        if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, int> || std::is_same_v<T, float> || std::is_same_v<T, double> || std::is_same_v<T, std::string>)
        {
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
};

} // namespace NAV