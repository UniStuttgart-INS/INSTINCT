#include "FlowAnimation.hpp"

#include <vector>
#include <mutex>

/// Queue to store animations till the draw threads pushes them onto the library
static std::vector<std::pair<ax::NodeEditor::LinkId, ax::NodeEditor::FlowDirection>> flowAnimationQueue;
/// Mutex to lock the animation queue
static std::mutex flowAnimationQueueMutex;

void NAV::FlowAnimation::Add(ax::NodeEditor::LinkId id, ax::NodeEditor::FlowDirection direction)
{
    const std::lock_guard<std::mutex> lock(flowAnimationQueueMutex);
    flowAnimationQueue.emplace_back(id, direction);
}

void NAV::FlowAnimation::ProcessQueue()
{
    const std::lock_guard<std::mutex> lock(flowAnimationQueueMutex);
    for (const auto& [linkId, direction] : flowAnimationQueue)
    {
        ax::NodeEditor::Flow(linkId, direction);
    }

    flowAnimationQueue.clear();
}
