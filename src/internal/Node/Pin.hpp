/// @file Pin.hpp
/// @brief Pin class
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <string>
#include <variant>
#include <vector>
#include <memory>
#include <tuple>

namespace NAV
{
class Node;
class NodeData;

using NodeCallback = std::tuple<Node*, void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId), ax::NodeEditor::LinkId>;
using NotifyFunction = std::tuple<Node*, void (Node::*)(ax::NodeEditor::LinkId), ax::NodeEditor::LinkId>;
using WatcherCallback = std::pair<void (*)(const std::shared_ptr<const NodeData>&), ax::NodeEditor::LinkId>;

class Pin
{
  public:
    /// @brief Type of the data on the Pin
    struct Type
    {
        /// @brief Type of the data on the Pin
        enum Value : uint8_t
        {
            None,     ///< Not initialized
            Flow,     ///< NodeData Trigger
            Bool,     ///< Boolean
            Int,      ///< Integer Number
            Float,    ///< Floating Point Number
            String,   ///< std::string
            Object,   ///< Generic Object
            Matrix,   ///< Matrix Object
            Delegate, ///< Reference to the Node object
        };

        constexpr Type() = default;

        //NOLINTNEXTLINE(hicpp-explicit-conversions, google-explicit-constructor)
        constexpr Type(Value type)
            : value(type) {}

        explicit Type(const std::string& typeString)
        {
            if (typeString == "Flow")
            {
                value = Type::Flow;
            }
            else if (typeString == "Bool")
            {
                value = Type::Bool;
            }
            else if (typeString == "Int")
            {
                value = Type::Int;
            }
            else if (typeString == "Float")
            {
                value = Type::Float;
            }
            else if (typeString == "String")
            {
                value = Type::String;
            }
            else if (typeString == "Object")
            {
                value = Type::Object;
            }
            else if (typeString == "Matrix")
            {
                value = Type::Matrix;
            }
            else if (typeString == "Delegate")
            {
                value = Type::Delegate;
            }
        }

        explicit operator Value() const { return value; } // Allow switch(Pin::Value(type)) and comparisons.
        explicit operator bool() = delete;                // Prevent usage: if(fruit)
        Type& operator=(Value v)
        {
            value = v;
            return *this;
        }

        friend constexpr bool operator==(const Pin::Type& lhs, const Pin::Type& rhs);
        friend constexpr bool operator!=(const Pin::Type& lhs, const Pin::Type& rhs);

        friend constexpr bool operator==(const Pin::Type& lhs, const Pin::Type::Value& rhs);
        friend constexpr bool operator==(const Pin::Type::Value& lhs, const Pin::Type& rhs);
        friend constexpr bool operator!=(const Pin::Type& lhs, const Pin::Type::Value& rhs);
        friend constexpr bool operator!=(const Pin::Type::Value& lhs, const Pin::Type& rhs);

        explicit operator std::string() const
        {
            switch (value)
            {
            case Type::None:
                return "None";
            case Type::Flow:
                return "Flow";
            case Type::Bool:
                return "Bool";
            case Type::Int:
                return "Int";
            case Type::Float:
                return "Float";
            case Type::String:
                return "String";
            case Type::Object:
                return "Object";
            case Type::Matrix:
                return "Matrix";
            case Type::Delegate:
                return "Delegate";
            }
            return "";
        }

      private:
        Value value = Value::None;
    };

    /// Kind of the Pin (Input/Output)
    struct Kind
    {
        /// @brief Kind of the Pin (Input/Output)
        enum Value : uint8_t
        {
            None,
            Output,
            Input,
        };

        Kind() = default;

        //NOLINTNEXTLINE(hicpp-explicit-conversions, google-explicit-constructor)
        constexpr Kind(Value kind)
            : value(kind) {}

        explicit Kind(const std::string& kindString)
        {
            if (kindString == "Input")
            {
                value = Kind::Input;
            }
            else if (kindString == "Output")
            {
                value = Kind::Output;
            }
        }

        explicit operator Value() const { return value; } // Allow switch(Pin::Value(type)) and comparisons.
        explicit operator bool() = delete;                // Prevent usage: if(fruit)
        Kind& operator=(Value v)
        {
            value = v;
            return *this;
        }

        friend constexpr bool operator==(const Pin::Kind& lhs, const Pin::Kind& rhs);
        friend constexpr bool operator!=(const Pin::Kind& lhs, const Pin::Kind& rhs);

        friend constexpr bool operator==(const Pin::Kind& lhs, const Pin::Kind::Value& rhs);
        friend constexpr bool operator==(const Pin::Kind::Value& lhs, const Pin::Kind& rhs);
        friend constexpr bool operator!=(const Pin::Kind& lhs, const Pin::Kind::Value& rhs);
        friend constexpr bool operator!=(const Pin::Kind::Value& lhs, const Pin::Kind& rhs);

        explicit operator std::string() const
        {
            switch (value)
            {
            case Kind::None:
                return "None";
            case Kind::Input:
                return "Input";
            case Kind::Output:
                return "Output";
            }
        }

      private:
        Value value = Value::None;
    };

    using PinData = std::variant<void*, bool*, int*, float*, double*, std::string*,
                                 void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId), // Input Flow, receive data
                                 std::shared_ptr<const NAV::NodeData> (Node::*)(bool)>;                          // Output Flow, read data

    /// @brief Default constructor
    Pin() = default;
    /// @brief Destructor
    ~Pin() = default;
    /// @brief Copy constructor
    Pin(const Pin&) = default;
    /// @brief Move constructor
    Pin(Pin&&) = default;
    /// @brief Copy assignment operator
    Pin& operator=(const Pin&) = default;
    /// @brief Move assignment operator
    Pin& operator=(Pin&&) = default;

    /// @brief Constructor
    /// @param[in] id Unique Id of the Pin
    /// @param[in] name Name of the Pin
    /// @param[in] type Type of the Pin
    /// @param[in] pinKind Kind of the Pin (Input/Output)
    /// @param[in] parentNode Reference to the parent node
    Pin(ax::NodeEditor::PinId id, const char* name, Type type, Kind kind, Node* parentNode)
        : id(id), name(name), type(type), kind(kind), parentNode(parentNode) {}

    /// @brief Checks if this pin can connect to the provided pin
    /// @param[in] b The pin to create a link to
    /// @return True if it can create a link
    [[nodiscard]] bool canCreateLink(const Pin& b) const;

    /// @brief Checks if the first list of data identifiers has a common entry with the second
    /// @param[in] a First list of data identifiers
    /// @param[in] b Second list of data identifiers
    /// @return True if they have a common entry
    [[nodiscard]] static bool dataIdentifierHaveCommon(const std::vector<std::string>& a, const std::vector<std::string>& b);

    /// @brief Get the Icon Color object
    /// @return Color struct
    [[nodiscard]] ImColor getIconColor() const;

    /// @brief Draw the Pin Icon
    /// @param[in] connected Flag if the pin is connected
    /// @param[in] alpha Alpha value of the pin
    void drawPinIcon(bool connected, int alpha) const;

    /// Unique Id of the Pin
    ax::NodeEditor::PinId id;
    /// Name of the Pin
    std::string name;
    /// Type of the Pin
    Type type = Type::None;
    /// Kind of the Pin (Input/Output)
    Kind kind = Kind::None;
    /// Reference to the parent node
    Node* parentNode = nullptr;
    /// Pointer to data which is transferred over this pin
    PinData data = static_cast<void*>(nullptr);
    /// Notify Function to call when the data is updated
    std::vector<NotifyFunction> notifyFunc;
    /// Callback List
    std::vector<NodeCallback> callbacks;
    /// One or multiple Data Identifiers (Unique name which is used for data flows)
    std::vector<std::string> dataIdentifier;

#ifdef TESTING
    /// Watcher Callbacks are used in testing to check the transmitted data
    std::vector<WatcherCallback> watcherCallbacks;
#endif

  private:
    /// Size of the Pin Icons in [px]
    static constexpr int m_PinIconSize = 24;
};

constexpr bool operator==(const Pin::Kind& lhs, const Pin::Kind& rhs) { return lhs.value == rhs.value; }
constexpr bool operator!=(const Pin::Kind& lhs, const Pin::Kind& rhs) { return lhs.value != rhs.value; }

constexpr bool operator==(const Pin::Kind& lhs, const Pin::Kind::Value& rhs) { return lhs.value == rhs; }
constexpr bool operator==(const Pin::Kind::Value& lhs, const Pin::Kind& rhs) { return lhs == rhs.value; }
constexpr bool operator!=(const Pin::Kind& lhs, const Pin::Kind::Value& rhs) { return lhs.value != rhs; }
constexpr bool operator!=(const Pin::Kind::Value& lhs, const Pin::Kind& rhs) { return lhs != rhs.value; }

constexpr bool operator==(const Pin::Type& lhs, const Pin::Type& rhs) { return lhs.value == rhs.value; }
constexpr bool operator!=(const Pin::Type& lhs, const Pin::Type& rhs) { return lhs.value != rhs.value; }

constexpr bool operator==(const Pin::Type& lhs, const Pin::Type::Value& rhs) { return lhs.value == rhs; }
constexpr bool operator==(const Pin::Type::Value& lhs, const Pin::Type& rhs) { return lhs == rhs.value; }
constexpr bool operator!=(const Pin::Type& lhs, const Pin::Type::Value& rhs) { return lhs.value != rhs; }
constexpr bool operator!=(const Pin::Type::Value& lhs, const Pin::Type& rhs) { return lhs != rhs.value; }

void to_json(json& j, const Pin& pin);
void from_json(const json& j, Pin& pin);

} // namespace NAV