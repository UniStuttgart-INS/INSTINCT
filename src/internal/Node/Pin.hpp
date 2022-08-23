/// @file Pin.hpp
/// @brief Pin class
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <imgui_node_editor.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include <string>
#include <variant>
#include <vector>
#include <memory>
#include <tuple>
#include <mutex>

namespace NAV
{
class Node;
class NodeData;
class InputPin;
class OutputPin;

// /// @brief Tuple of types allowed as a node callback
// using NodeCallbackInfo = std::tuple<Node*, void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId), ax::NodeEditor::LinkId>;
// /// @brief Tuple of types allowed as notify functions
// using NotifyFunctionInfo = std::tuple<Node*, void (Node::*)(ax::NodeEditor::LinkId), ax::NodeEditor::LinkId>;
// /// @brief Tuple of types allowed as watcher callbacks
// using WatcherCallbackInfo = std::pair<void (*)(const std::shared_ptr<const NodeData>&), ax::NodeEditor::LinkId>;

/// @brief Pins in the GUI for information exchange
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

        /// @brief Default Constructor
        constexpr Type() = default;

        /// @brief Implicit Constructor from Value type
        /// @param[in] type Value type to construct from
        constexpr Type(Value type) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
            : value(type)
        {}

        /// @brief Constructor from std::string
        /// @param[in] typeString String representation of the type
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

        /// @brief Allow switch(Node::Value(type)) and comparisons
        explicit operator Value() const { return value; }
        /// @brief Prevent usage: if(pin)
        explicit operator bool() = delete;
        /// @brief Assignment operator from Value type
        /// @param[in] v Value type to construct from
        /// @return The Type type from the value type
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

        /// @brief std::string conversion operator
        /// @return A std::string representation of the pin type
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
        /// @brief Value of the pin type
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

        /// @brief Default Constructor
        Kind() = default;

        /// @brief Implicit Constructor from Value type
        /// @param[in] kind Value type to construct from
        constexpr Kind(Value kind) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
            : value(kind)
        {}

        /// @brief Constructor from std::string
        /// @param[in] kindString String representation of the type
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

        /// @brief Allow switch(Node::Value(kind)) and comparisons
        explicit operator Value() const { return value; }
        /// @brief Prevent usage: if(pin)
        explicit operator bool() = delete;
        /// @brief Assignment operator from Value type
        /// @param[in] v Value type to construct from
        /// @return The Kind type from the value type
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

        /// @brief std::string conversion operator
        /// @return A std::string representation of the pin kind
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
        /// @brief Value of the pin kind
        Value value = Value::None;
    };

    /// Link between two pins
    struct Link
    {
        ax::NodeEditor::LinkId linkId = 0;        ///< Unique id of the link
        Node* connectedNode = nullptr;            ///< Pointer to the node, which is connected to this pin
        ax::NodeEditor::PinId connectedPinId = 0; ///< Id of the pin, which is connected to this pin

      protected:
        /// @brief Default Constructor
        Link() = default;

        /// @brief Constructor
        /// @param[in] linkId Unique id of the link
        /// @param[in] connectedNode Node connected on the other end of the link
        /// @param[in] connectedPinId Id of the pin, which is connected on the other end of the link
        Link(ax::NodeEditor::LinkId linkId,
             Node* connectedNode,
             ax::NodeEditor::PinId connectedPinId)
            : linkId(linkId), connectedNode(connectedNode), connectedPinId(connectedPinId) {}
    };

    // /// Callback function type to call when firable
    // using OldFlowCallback = void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId);
    // /// Notify function type to call when the connected value changed
    // using OldNotifyFunc = void (Node::*)(ax::NodeEditor::LinkId);
    // /// FileReader pollData function type
    // using OldPollDataFunc = std::shared_ptr<const NAV::NodeData> (Node::*)(bool);

    /// @brief Constructor
    /// @param[in] id Unique Id of the Pin
    /// @param[in] name Name of the Pin
    /// @param[in] type Type of the Pin
    /// @param[in] kind Kind of the Pin (Input/Output)
    /// @param[in] parentNode Reference to the parent node
    Pin(ax::NodeEditor::PinId id, const char* name, Type type, Kind kind, Node* parentNode)
        : id(id), name(name), type(type), kind(kind), parentNode(parentNode) {}

    /// @brief Checks if pins can connect
    /// @param[in] startPin The start pin to create a link to
    /// @param[in] endPin The end pin to create a link to
    /// @return True if the pins can create a link
    [[nodiscard]] static bool canCreateLink(const OutputPin& startPin, const InputPin& endPin);

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
    /// One or multiple Data Identifiers (Unique name which is used for data flows)
    std::vector<std::string> dataIdentifier;
    /// Reference to the parent node
    Node* parentNode = nullptr;

  protected:
    /// @brief Default constructor
    Pin() = default;

  private:
    /// Size of the Pin Icons in [px]
    static constexpr int m_PinIconSize = 24;
};

/// Output pins of nodes
class OutputPin : public Pin
{
  public:
    /// @brief Constructor
    /// @param[in] id Unique Id of the Pin
    /// @param[in] name Name of the Pin
    /// @param[in] type Type of the Pin
    /// @param[in] parentNode Reference to the parent node
    OutputPin(ax::NodeEditor::PinId id, const char* name, Type type, Node* parentNode)
        : Pin(id, name, type, Pin::Kind::Output, parentNode) {}

    /// @brief Destructor
    ~OutputPin() = default;
    /// @brief Copy constructor
    OutputPin(const OutputPin&) = delete;
    /// @brief Move constructor
    OutputPin(OutputPin&& other) noexcept
        : Pin(other)
    {
        std::scoped_lock<std::mutex> guard(other.dataAccessMutex); // Make sure the mutex of the other element is not used
        data = other.data;
    }
    /// @brief Copy assignment operator
    OutputPin& operator=(const OutputPin&) = delete;
    /// @brief Move assignment operator
    OutputPin& operator=(OutputPin&& other) noexcept
    {
        if (this != &other)
        {
            *static_cast<Pin*>(this) = other;

            std::scoped_lock<std::mutex> guard(other.dataAccessMutex);
            data = other.data;
        }
        return *this;
    }

    /// @brief Checks if this pin can connect to the provided pin
    /// @param[in] other The pin to create a link to
    /// @return True if it can create a link
    [[nodiscard]] bool canCreateLink(const InputPin& other) const;

    /// FileReader/Simulator pollData function type
    using PollDataFunc = std::shared_ptr<const NAV::NodeData> (Node::*)(bool);

    /// @brief Possible Types represented by an output pin
    using PinData = std::variant<const void*,        // Object/Matrix/Delegate
                                 const bool*,        // Bool
                                 const int*,         // Int
                                 const float*,       // Float
                                 const double*,      // Float
                                 const std::string*, // String
                                 PollDataFunc>;      // Flow (FileReader poll data function)

    /// Pointer to data (owned by this node) which is transferred over this pin
    PinData data = static_cast<void*>(nullptr);

    /// Mutex to interact with the data object
    std::mutex dataAccessMutex;

    /// @brief Connects this pin to another
    /// @param[in] endPin Pin which should be linked to this pin
    /// @return True if the link could be created
    bool connect(InputPin& endPin);

    /// @brief Disconnects the link
    /// @param[in] endPin Pin which should be disconnected from this pin
    void disconnect(InputPin& endPin);

    /// Collection of information about the connected node and pin
    struct OutgoingLink : public Link
    {
        /// @brief Default Constructor
        OutgoingLink() = default;

        /// @brief Constructor
        /// @param[in] linkId Unique id of the link
        /// @param[in] connectedNode Node connected on the other end of the link
        /// @param[in] connectedPinId Id of the pin, which is connected on the other end of the link
        OutgoingLink(ax::NodeEditor::LinkId linkId,
                     Node* connectedNode,
                     ax::NodeEditor::PinId connectedPinId)
            : Link(linkId, connectedNode, connectedPinId) {}
    };

    /// Info to identify the linked pins
    std::vector<OutgoingLink> links;
};

/// Input pins of nodes
class InputPin : public Pin
{
  public:
    /// @brief Constructor
    /// @param[in] id Unique Id of the Pin
    /// @param[in] name Name of the Pin
    /// @param[in] type Type of the Pin
    /// @param[in] parentNode Reference to the parent node
    InputPin(ax::NodeEditor::PinId id, const char* name, Type type, Node* parentNode)
        : Pin(id, name, type, Pin::Kind::Input, parentNode) {}

    /// @brief Checks if this pin can connect to the provided pin
    /// @param[in] other The pin to create a link to
    /// @return True if it can create a link
    [[nodiscard]] bool canCreateLink(const OutputPin& other) const;

    /// Flow data callback function type to call when firable
    using FlowFirableCallbackFunc = void (Node::*)(const std::shared_ptr<const NodeData>&, ax::NodeEditor::LinkId);
    /// Notify function type to call when the connected value changed
    using DataChangedNotifyFunc = void (Node::*)(ax::NodeEditor::LinkId);

    /// Callback function types
    using Callback = std::variant<FlowFirableCallbackFunc, // Flow:  Callback function type to call when firable
                                  DataChangedNotifyFunc>;  // Other: Notify function type to call when the connected value changed

    /// Callback to call when the node is firable or when it should be notified of data change
    Callback callback;

    /// @brief Connects this pin to another
    /// @param[in] startPin Pin which should be linked to this pin
    /// @return True if the link could be created
    bool connect(OutputPin& startPin);

    /// @brief Disconnects the link
    /// @param[in] startPin Pin which should be disconnected from this pin
    void disconnect(OutputPin& startPin);

    /// Collection of information about the connected node and pin
    struct IncomingLink : public Link
    {
        /// @brief Default Constructor
        IncomingLink() = default;

        /// @brief Constructor
        /// @param[in] linkId Unique id of the link
        /// @param[in] connectedNode Node connected on the other end of the link
        /// @param[in] connectedPinId Id of the pin, which is connected on the other end of the link
        IncomingLink(ax::NodeEditor::LinkId linkId,
                     Node* connectedNode,
                     ax::NodeEditor::PinId connectedPinId)
            : Link(linkId, connectedNode, connectedPinId) {}

        /// @brief Returns a pointer to the pin which is connected to this one
        [[nodiscard]] const OutputPin* getConnectedPin() const;

        /// @brief Get a pointer to the value connected on the pin
        /// @tparam T Type of the connected object
        /// @return Pointer to the object
        template<typename T>
        [[nodiscard]] const T* getValue() const
        {
            if (const auto* connectedPin = getConnectedPin())
            {
                // clang-format off
                if constexpr (std::is_same_v<T, const bool>
                           || std::is_same_v<T, const int>
                           || std::is_same_v<T, const float>
                           || std::is_same_v<T, const double>
                           || std::is_same_v<T, const std::string>)
                { // clang-format on
                    if (const auto* pVal = std::get_if<const T*>(&(connectedPin->data)))
                    {
                        return *pVal;
                    }
                }
                else // if constexpr (std::is_same_v<T, void>
                     //            || std::is_same_v<T, PollDataFunc>)
                {
                    if (const auto* pVal = std::get_if<const void*>(&(connectedPin->data)))
                    {
                        return static_cast<const T*>(*pVal);
                    }
                }
            }

            return nullptr;
        }
    };

    /// Info to identify the linked pin
    IncomingLink link;
};

/// @brief Equal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Pin::Kind& lhs, const Pin::Kind& rhs) { return lhs.value == rhs.value; }
/// @brief Inequal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Pin::Kind& lhs, const Pin::Kind& rhs) { return !(lhs == rhs); }

/// @brief Equal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Pin::Kind& lhs, const Pin::Kind::Value& rhs) { return lhs.value == rhs; }
/// @brief Equal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Pin::Kind::Value& lhs, const Pin::Kind& rhs) { return lhs == rhs.value; }
/// @brief Inequal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Pin::Kind& lhs, const Pin::Kind::Value& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares Pin::Kind values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Pin::Kind::Value& lhs, const Pin::Kind& rhs) { return !(lhs == rhs); }

/// @brief Equal compares Pin::Type values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Pin::Type& lhs, const Pin::Type& rhs) { return lhs.value == rhs.value; }
/// @brief Inequal compares Pin::Type values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Pin::Type& lhs, const Pin::Type& rhs) { return !(lhs == rhs); }

/// @brief Equal compares Pin::Type values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Pin::Type& lhs, const Pin::Type::Value& rhs) { return lhs.value == rhs; }
/// @brief Equal compares Pin::Type values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Pin::Type::Value& lhs, const Pin::Type& rhs) { return lhs == rhs.value; }
/// @brief Inequal compares Pin::Type values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Pin::Type& lhs, const Pin::Type::Value& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares Pin::Type values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Pin::Type::Value& lhs, const Pin::Type& rhs) { return !(lhs == rhs); }

/// @brief Converts the provided pin into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] pin Node to convert into json
void to_json(json& j, const Pin& pin);
/// @brief Converts the provided json object into a pin object
/// @param[in] j Json object with the needed values
/// @param[out] pin Object to fill from the json
void from_json(const json& j, Pin& pin);

} // namespace NAV