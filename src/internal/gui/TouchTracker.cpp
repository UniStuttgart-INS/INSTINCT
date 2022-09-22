// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TouchTracker.hpp"

#include <map>

/// @brief Comparison operator for node Ids
struct NodeIdLess
{
    /// @brief Smaller comparison operator for node Ids
    /// @param[in] lhs Left-hand-side of the operator
    /// @param[in] rhs Right-hand-side of the operator
    /// @return Whether lhs < rhs
    bool operator()(const ax::NodeEditor::NodeId& lhs, const ax::NodeEditor::NodeId& rhs) const
    {
        return lhs.AsPointer() < rhs.AsPointer();
    }
};

const float m_TouchTime = 1.0F;
std::map<ax::NodeEditor::NodeId, float, NodeIdLess> m_NodeTouchTime;

/// @brief  Trigger a touch event on the specified node
/// @param[in] id Id of the node to trigger the event on
void NAV::gui::TouchNode(ax::NodeEditor::NodeId id)
{
    m_NodeTouchTime[id] = m_TouchTime;
}

/// @brief Get the Touch Progress for the specified node
/// @param[in] id Id of the Node to check
/// @return The Touch progress towards the touch time
float NAV::gui::GetTouchProgress(ax::NodeEditor::NodeId id)
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