#include "TouchTracker.hpp"

#include <map>

namespace ed = ax::NodeEditor;

struct NodeIdLess
{
    bool operator()(const ed::NodeId& lhs, const ed::NodeId& rhs) const
    {
        return lhs.AsPointer() < rhs.AsPointer();
    }
};

const float m_TouchTime = 1.0F;
std::map<ed::NodeId, float, NodeIdLess> m_NodeTouchTime;

/// @brief  Trigger a touch event on the specified node
/// @param[in] id Id of the node to trigger the event on
void NAV::gui::TouchNode(ed::NodeId id)
{
    m_NodeTouchTime[id] = m_TouchTime;
}

/// @brief Get the Touch Progress for the specified node
/// @param[in] id Id of the Node to check
/// @return The Touch progress towards the touch time
float NAV::gui::GetTouchProgress(ed::NodeId id)
{
    auto it = m_NodeTouchTime.find(id);
    if (it != m_NodeTouchTime.end() && it->second > 0.0F)
    {
        return (m_TouchTime - it->second) / m_TouchTime;
    }

    return 0.0F;
}

/// @brief Updates the touch events for all nodes
/// @param[in] deltaTime Time elapsed since last frame, in [seconds]
void NAV::gui::UpdateTouch(float deltaTime)
{
    for (auto& entry : m_NodeTouchTime)
    {
        if (entry.second > 0.0F)
        {
            entry.second -= deltaTime;
        }
    }
}