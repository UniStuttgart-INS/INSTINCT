/// @file FlowManager.hpp
/// @brief Save/Load the Nodes
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-16

#pragma once

#include <string>
#include "internal/gui/GlobalActions.hpp"

namespace NAV::flow
{
void SaveFlow(GlobalActions& globalAction);

void SaveFlowAs(const std::string& filepath);

bool LoadFlow(const std::string& filepath);

bool HasUnsavedChanges();

void ApplyChanges();

void DiscardChanges();

std::string GetCurrentFilename();

void SetCurrentFilename(const std::string& newFilename);

std::string GetProgramRootPath();

void SetProgramRootPath(const std::string& newRootPath);

} // namespace NAV::flow
