/// @file Node.hpp
/// @brief Node Class
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include "Pin.hpp"

#include "util/Logger.hpp"

#include <string>
#include <vector>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace NAV
{
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

        //NOLINTNEXTLINE(hicpp-explicit-conversions)
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
    Node()
    {
        name.reserve(30);
    }
    /// @brief Destructor
    virtual ~Node()
    {
        LOG_TRACE("called");
    }
    /// @brief Copy constructor
    Node(const Node&) = delete;
    /// @brief Move constructor
    Node(Node&&) = delete;
    /// @brief Copy assignment operator
    Node& operator=(const Node&) = delete;
    /// @brief Move assignment operator
    Node& operator=(Node&&) = delete;

    [[nodiscard]] virtual std::string type() const = 0;

    [[nodiscard]] virtual json save() const = 0;

    virtual void restore(json const& /*j*/) = 0;

    virtual void config() = 0;

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
            if (auto pval = std::get_if<T*>(&inputPins.at(portIndex).data))
            {
                return *pval;
            }
        }
        else // constexpr
        {
            if (auto pval = std::get_if<void*>(&inputPins.at(portIndex).data))
            {
                return static_cast<T*>(*pval);
            }
        }

        return nullptr;
    }

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
    /// Flag if the config window should be shown
    bool hasConfig = false;
    /// Flag if the config window is shown
    bool showConfig = false;
};

} // namespace NAV