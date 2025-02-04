// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FlowAnimation.hpp"

#include <vector>
#include <mutex>
#include <algorithm>

namespace
{

/// Queue to store animations till the draw threads pushes them onto the library
std::vector<std::pair<ax::NodeEditor::LinkId, ax::NodeEditor::FlowDirection>> flowAnimationQueue;
/// Mutex to lock the animation queue
std::mutex flowAnimationQueueMutex;

} // namespace

void NAV::FlowAnimation::Add(ax::NodeEditor::LinkId id, ax::NodeEditor::FlowDirection direction)
{
    const std::scoped_lock<std::mutex> lock(flowAnimationQueueMutex);
    if (std::ranges::find(flowAnimationQueue, std::make_pair(id, direction)) == flowAnimationQueue.end())
    {
        flowAnimationQueue.emplace_back(id, direction);
    }
}

void NAV::FlowAnimation::ProcessQueue()
{
    const std::scoped_lock<std::mutex> lock(flowAnimationQueueMutex);
    for (const auto& [linkId, direction] : flowAnimationQueue)
    {
        ax::NodeEditor::Flow(linkId, direction);
    }

    flowAnimationQueue.clear();
}
