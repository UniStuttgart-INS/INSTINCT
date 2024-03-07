// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Pin.hpp
/// @brief Pin class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
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
#include <atomic>
#include <condition_variable>

#include "util/Logger.hpp"
#include "util/Container/TsDeque.hpp"
#include "Navigation/Time/InsTime.hpp"

namespace NAV
{
class Node;
class NodeData;
class InputPin;
class OutputPin;

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
            None,   ///< None
            Output, ///< Output Pin
            Input,  ///< Input Pin
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

    /// @brief Create a Link between the two given pins
    /// @param[in] startPin Start Pin of the link
    /// @param[in] endPin End Pin of the link
    /// @param[in] linkId Id of the link to create
    /// @return True if the link could be created
    static bool createLink(OutputPin& startPin, InputPin& endPin, ax::NodeEditor::LinkId linkId = 0);

    /// @brief Destroys and recreates a link from this pin to another
    /// @param[in] startPin Start Pin of the link
    /// @param[in] endPin End Pin of the link
    /// @return True if the link could be created
    static bool recreateLink(OutputPin& startPin, InputPin& endPin);

    /// @brief Disconnects the link
    /// @param[in] startPin Start Pin of the link
    /// @param[in] endPin End Pin of the link
    static void deleteLink(OutputPin& startPin, InputPin& endPin);

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

    /// @brief Default constructor (for serialization)
    OutputPin() = default;
    /// @brief Destructor
    ~OutputPin() = default;
    /// @brief Copy constructor
    OutputPin(const OutputPin&) = delete;
    /// @brief Move constructor
    OutputPin(OutputPin&& other) noexcept
        : Pin(std::move(other)),
          links(std::move(other.links)),
          data(other.data),
          noMoreDataAvailable(other.noMoreDataAvailable.load()),
          blocksConnectedNodeFromFinishing(other.blocksConnectedNodeFromFinishing.load()) {}
    /// @brief Copy assignment operator
    OutputPin& operator=(const OutputPin&) = delete;
    /// @brief Move assignment operator
    OutputPin& operator=(OutputPin&& other) noexcept
    {
        if (this != &other)
        {
            links = std::move(other.links);
            data = other.data;
            noMoreDataAvailable = other.noMoreDataAvailable.load();
            blocksConnectedNodeFromFinishing = other.blocksConnectedNodeFromFinishing.load();
            Pin::operator=(std::move(other));
        }
        return *this;
    }

    /// @brief Checks if this pin can connect to the provided pin
    /// @param[in] other The pin to create a link to
    /// @return True if it can create a link
    [[nodiscard]] bool canCreateLink(const InputPin& other) const;

    /// @brief Checks if the pin is linked
    /// @return True if a link exists on this pin
    [[nodiscard]] bool isPinLinked() const;

    /// @brief Checks if the pin is linked to the other pin
    /// @param[in] endPin The pin to check if they are linked
    /// @return True if a link exists on the pins
    [[nodiscard]] bool isPinLinked(const InputPin& endPin) const;

    /// @brief Creates a link from this pin to another, calling all node specific callbacks
    /// @param[in] endPin Pin which should be linked to this pin
    /// @param[in] linkId Id of the link to create
    /// @return True if the link could be created
    bool createLink(InputPin& endPin, ax::NodeEditor::LinkId linkId = 0);

    /// @brief Destroys and recreates a link from this pin to another
    /// @param[in] endPin Pin which should be linked to this pin
    /// @return True if the link could be created
    bool recreateLink(InputPin& endPin);

    /// @brief Disconnects the link
    /// @param[in] endPin Pin which should be linked to this pin
    void deleteLink(InputPin& endPin);

    /// @brief Disconnects all links
    void deleteLinks();

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

        /// @brief Returns a pointer to the pin which is connected to this one
        [[nodiscard]] InputPin* getConnectedPin() const;

        /// @brief Flag to signal the connected node, that the data was changed
        bool dataChangeNotification = false;
    };

    /// Info to identify the linked pins
    std::vector<OutgoingLink> links;

    /// @brief FileReader/Simulator peekPollData function type for nodes with more than one polling pin
    ///
    /// - First parameter is the index of the pin in the outputPins vector
    /// - Second parameter is a boolean 'peek'
    ///   This function gets called twice.
    ///   - First with 'peek = true':   There an observation with a valid InsTime must be provided. `invokeCallbacks(...` should not be called.
    ///   - Second with 'peek = false': Here the message is read again and `invokeCallbacks(...)` should be called
    using PeekPollDataFunc = std::shared_ptr<const NAV::NodeData> (Node::*)(size_t, bool);

    /// FileReader/Simulator pollData function type for nodes with a single poll pin
    using PollDataFunc = std::shared_ptr<const NAV::NodeData> (Node::*)();

    /// @brief Possible Types represented by an output pin
    using PinData = std::variant<const void*,        // Object/Matrix/Delegate
                                 const bool*,        // Bool
                                 const int*,         // Int
                                 const float*,       // Float
                                 const double*,      // Float
                                 const std::string*, // String
                                 PeekPollDataFunc,   // Flow (FileReader poll data function with peeking)
                                 PollDataFunc>;      // Flow (FileReader poll data function)

    /// Pointer to data (owned by this node) which is transferred over this pin
    PinData data = static_cast<void*>(nullptr);

    /// Mutex to interact with the data object and also the dataAccessCounter variable
    std::mutex dataAccessMutex;

    /// @brief Counter for data accessing
    size_t dataAccessCounter = 0;

    /// Condition variable to signal that the data was read by connected nodes (used for non-flow pins)
    std::condition_variable dataAccessConditionVariable;

    /// Flag set, when no more data is available on this pin
    std::atomic<bool> noMoreDataAvailable = true;

    /// Flag, whether connected nodes can finish with this pin not being finished yet (needed to make a loop with nodes)
    std::atomic<bool> blocksConnectedNodeFromFinishing = true;

    friend class Pin;
    friend class InputPin;

  private:
    /// @brief Connects this pin to another
    /// @param[in] endPin Pin which should be linked to this pin
    /// @param[in] linkId Id of the link to create
    void connect(InputPin& endPin, ax::NodeEditor::LinkId linkId = 0);

    /// @brief Disconnects the link
    /// @param[in] endPin Pin which should be disconnected from this pin
    void disconnect(InputPin& endPin);
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

    /// @brief Default constructor (for serialization)
    InputPin() = default;
    /// @brief Destructor
    ~InputPin() = default;
    /// @brief Copy constructor
    InputPin(const InputPin&) = delete;
    /// @brief Move constructor
    InputPin(InputPin&& other) noexcept
        : Pin(std::move(other)),
          link(other.link),
          callback(other.callback),
          firable(other.firable),
          priority(other.priority),
          neededForTemporalQueueCheck(other.neededForTemporalQueueCheck),
          dropQueueIfNotFirable(other.dropQueueIfNotFirable),
          queueBlocked(other.queueBlocked),
          queue(other.queue) {}
    /// @brief Copy assignment operator
    InputPin& operator=(const InputPin&) = delete;
    /// @brief Move assignment operator
    InputPin& operator=(InputPin&& other) noexcept
    {
        if (this != &other)
        {
            // copy if trivially-copyable, otherwise move
            link = other.link;
            callback = other.callback;
            firable = other.firable;
            priority = other.priority;
            neededForTemporalQueueCheck = other.neededForTemporalQueueCheck;
            dropQueueIfNotFirable = other.dropQueueIfNotFirable;
            queueBlocked = other.queueBlocked;
            queue = std::move(other.queue);
            Pin::operator=(std::move(other));
        }
        return *this;
    }

    /// @brief Checks if this pin can connect to the provided pin
    /// @param[in] other The pin to create a link to
    /// @return True if it can create a link
    [[nodiscard]] bool canCreateLink(const OutputPin& other) const;

    /// @brief Checks if the pin is linked
    /// @return True if a link exists on this pin
    [[nodiscard]] bool isPinLinked() const;

    /// @brief Creates a link from this pin to another, calling all node specific callbacks
    /// @param[in] startPin Pin which should be linked to this pin
    /// @param[in] linkId Id of the link to create
    /// @return True if the link could be created
    bool createLink(OutputPin& startPin, ax::NodeEditor::LinkId linkId = 0);

    /// @brief Destroys and recreates a link from this pin to another
    /// @param[in] startPin Pin which should be linked to this pin
    /// @return True if the link could be created
    bool recreateLink(OutputPin& startPin);

    /// @brief Disconnects the link
    void deleteLink();

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
        [[nodiscard]] OutputPin* getConnectedPin() const;

        /// @brief Value wrapper, automatically incrementing and decrementing the data access counter
        template<typename T>
        class ValueWrapper
        {
          public:
            /// @brief Constructor
            /// @param v Reference to the value to wrap
            /// @param outputPin Output pin reference
            /// @param dataChangeNotification Reference to the Data change notification of the link
            ValueWrapper(const T* v, OutputPin* outputPin, bool& dataChangeNotification)
                : v(v), outputPin(outputPin)
            {
                std::scoped_lock guard(outputPin->dataAccessMutex);
                if (!dataChangeNotification)
                {
                    outputPin->dataAccessCounter++;
                }
                dataChangeNotification = false; // We take 'ownership' of the incremented dataAccessCounter
            }
            /// @brief Desctructor
            ~ValueWrapper()
            {
                if (outputPin)
                {
                    std::scoped_lock guard(outputPin->dataAccessMutex);
                    if (outputPin->dataAccessCounter > 0)
                    {
                        outputPin->dataAccessCounter--;
                        if (outputPin->dataAccessCounter == 0)
                        {
                            outputPin->dataAccessConditionVariable.notify_all();
                        }
                    }
                }
            }

            /// @brief Copy constructor
            ValueWrapper(const ValueWrapper& other)
                : v(other.v), outputPin(other.outputPin)
            {
                std::scoped_lock guard(outputPin->dataAccessMutex);
                outputPin->dataAccessCounter++;
            }
            /// @brief Move constructor
            ValueWrapper(ValueWrapper&& other) noexcept
                : v(std::move(other.v)), outputPin(std::move(other.outputPin)) {}
            /// @brief Copy assignment operator
            ValueWrapper& operator=(const ValueWrapper& other)
            {
                if (this != &other)
                {
                    v = other.v;
                    outputPin = other.outputPin;
                    std::scoped_lock guard(outputPin->dataAccessMutex);
                    outputPin->dataAccessCounter++;
                }
                return *this;
            }
            /// @brief Move assignment operator
            ValueWrapper& operator=(ValueWrapper&& other) noexcept
            {
                if (this != &other)
                {
                    v = std::move(other.v);
                    outputPin = std::move(other.outputPin);
                }
                return *this;
            }

            /// Pointer to the value wrapped
            const T* v;

          private:
            /// Pointer to the output pin representing the data
            OutputPin* outputPin;
        };

        /// @brief Get the value connected on the pin if the pin is connected. Automatically increments the data access counter to prevent editing.
        /// @tparam T Type of the connected object
        /// @return ValueWrapper with a const pointer to the connected data
        template<typename T>
        [[nodiscard]] std::optional<ValueWrapper<T>> getValue() const
        {
            if (auto* connectedPin = getConnectedPin())
            {
                // if (!connectedPin->parentNode->isInitialized()) { return nullptr; } // TODO: Check this here (Problem: member access into incomplete type 'NAV::Node')

                auto outgoingLink = std::find_if(connectedPin->links.begin(), connectedPin->links.end(), [&](const OutputPin::OutgoingLink& link) {
                    return link.linkId == linkId;
                });

                // clang-format off
                if constexpr (std::is_same_v<T, bool>
                           || std::is_same_v<T, int>
                           || std::is_same_v<T, float>
                           || std::is_same_v<T, double>
                           || std::is_same_v<T, std::string>) // clang-format on
                {
                    if (const auto* pVal = std::get_if<const T*>(&(connectedPin->data));
                        pVal && *pVal)
                    {
                        return ValueWrapper<T>(*pVal, connectedPin, outgoingLink->dataChangeNotification);
                    }
                }
                else
                {
                    if (const auto* pVal = std::get_if<const void*>(&(connectedPin->data));
                        pVal && *pVal)
                    {
                        return ValueWrapper<T>(static_cast<const T*>(*pVal), connectedPin, outgoingLink->dataChangeNotification);
                    }
                }
            }

            return std::nullopt;
        }
    };

    /// Info to identify the linked pin
    IncomingLink link;

    /// Node data queue type
    using NodeDataQueue = TsDeque<std::shared_ptr<const NAV::NodeData>>;

    /// Flow data callback function type to call when firable.
    /// - 1st Parameter: Queue with the received messages
    /// - 2nd Parameter: Pin index of the pin the data is received on
    using FlowFirableCallbackFunc = void (Node::*)(NodeDataQueue&, size_t);
    /// Notify function type to call when the connected value changed
    /// - 1st Parameter: Time when the message was received
    /// - 2nd Parameter: Pin index of the pin the data is received on
    using DataChangedNotifyFunc = void (Node::*)(const InsTime&, size_t);
    /// Callback function types
    using Callback = std::variant<FlowFirableCallbackFunc, // Flow:  Callback function type to call when firable
                                  DataChangedNotifyFunc>;  // Other: Notify function type to call when the connected value changed

    /// Callback to call when the node is firable or when it should be notified of data change
    Callback callback;

    /// Function type to call when checking if a pin is firable
    using FlowFirableCheckFunc = bool (*)(const Node*, const InputPin&);

    /// @brief Function to check if the callback is firable
    FlowFirableCheckFunc firable = [](const Node*, const InputPin& inputPin) { return !inputPin.queue.empty() && !inputPin.queueBlocked; };

    /// @brief Priority when checking firable condition related to other pins (higher priority gets triggered first)
    int priority = 0;

    /// @brief Whether it should be checked for temporal ordering
    bool neededForTemporalQueueCheck = true;

    /// @brief If true, drops elements from the queue if not firable, otherwise sleeps the worker
    bool dropQueueIfNotFirable = true;

    /// If true no more messages are accepted to the queue
    bool queueBlocked = false;

    /// Queue with received data
    NodeDataQueue queue;

#ifdef TESTING
    /// Flow data watcher callback function type to call when firable.
    /// - 1st Parameter: Queue with the received messages
    /// - 2nd Parameter: Pin index of the pin the data is received on
    using FlowFirableWatcherCallbackFunc = std::function<void(const Node*, const NodeDataQueue&, size_t)>;
    /// Notify watcher function type to call when the connected value changed
    /// - 1st Parameter: Time when the message was received
    /// - 2nd Parameter: Pin index of the pin the data is received on
    using DataChangedWatcherNotifyFunc = std::function<void(const Node*, const InsTime&, size_t)>;

    /// Watcher callback function types
    using WatcherCallback = std::variant<FlowFirableWatcherCallbackFunc, // Flow:  Callback function type to call when firable
                                         DataChangedWatcherNotifyFunc>;  // Other: Notify function type to call when the connected value changed

    /// Watcher Callbacks are used in testing to check the transmitted data
    std::vector<WatcherCallback> watcherCallbacks;
#endif

    friend class Pin;
    friend class OutputPin;

  private:
    /// @brief Connects this pin to another
    /// @param[in] startPin Pin which should be linked to this pin
    /// @param[in] linkId Id of the link to create
    void connect(OutputPin& startPin, ax::NodeEditor::LinkId linkId = 0);

    /// @brief Disconnects the link
    void disconnect();
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
void to_json(json& j, const OutputPin& pin);
/// @brief Converts the provided json object into a pin object
/// @param[in] j Json object with the needed values
/// @param[out] pin Object to fill from the json
void from_json(const json& j, OutputPin& pin);

/// @brief Converts the provided pin into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] pin Node to convert into json
void to_json(json& j, const InputPin& pin);
/// @brief Converts the provided json object into a pin object
/// @param[in] j Json object with the needed values
/// @param[out] pin Object to fill from the json
void from_json(const json& j, InputPin& pin);

} // namespace NAV