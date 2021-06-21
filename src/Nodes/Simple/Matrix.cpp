#include "Matrix.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::Matrix::Matrix()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 330, 420 };
    kind = Kind::Simple;

    nm::CreateOutputPin(this, "", Pin::Type::Matrix, "Eigen::MatrixXd", &matrix);

    initMatrix = Eigen::MatrixXd::Zero(nRows, nCols);
    matrix = initMatrix;

    updateNumberOfOutputPins();
}

NAV::Matrix::~Matrix()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Matrix::typeStatic()
{
    return "Matrix";
}

std::string NAV::Matrix::type() const
{
    return typeStatic();
}

std::string NAV::Matrix::category()
{
    return "Simple";
}

void NAV::Matrix::guiConfig()
{
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader(("Options##" + std::to_string(size_t(id))).c_str()))
    {
        if (ImGui::InputInt("Rows", &nRows))
        {
            if (nRows < 1)
            {
                nRows = 1;
            }
            LOG_DEBUG("{}: # Rows changed to {}", nameId(), nRows);
            flow::ApplyChanges();
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nRows, nCols);
            for (int64_t row = 0; row < initMatrix.rows() && row < mat.rows(); row++)
            {
                for (int64_t col = 0; col < initMatrix.cols() && col < mat.cols(); col++)
                {
                    mat(row, col) = initMatrix(row, col);
                }
            }
            for (auto& block : blocks)
            {
                if (block.startRow < 0)
                {
                    block.startRow = 0;
                }
                else if (block.startRow >= mat.rows())
                {
                    block.startRow = static_cast<int>(mat.rows() - 1);
                }

                if (block.blockRows < 1)
                {
                    block.blockRows = 1;
                }
                else if (block.blockRows >= mat.rows() - block.startRow)
                {
                    block.blockRows = static_cast<int>(mat.rows() - block.startRow);
                }
            }

            initMatrix = mat;
            matrix = initMatrix;
            for (auto& block : blocks)
            {
                block.matrix = &matrix;
            }
            for (auto& outputPin : outputPins)
            {
                auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPin.id);
                for (auto& connectedLink : connectedLinks)
                {
                    nm::RefreshLink(connectedLink->id);
                }
            }
        }
        if (ImGui::InputInt("Cols", &nCols))
        {
            if (nCols < 1)
            {
                nCols = 1;
            }
            LOG_DEBUG("{}: # Cols changed to {}", nameId(), nCols);
            flow::ApplyChanges();
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(nRows, nCols);
            for (int64_t row = 0; row < initMatrix.rows() && row < mat.rows(); row++)
            {
                for (int64_t col = 0; col < initMatrix.cols() && col < mat.cols(); col++)
                {
                    mat(row, col) = initMatrix(row, col);
                }
            }

            for (auto& block : blocks)
            {
                if (block.startCol < 0)
                {
                    block.startCol = 0;
                }
                else if (block.startCol >= mat.cols())
                {
                    block.startCol = static_cast<int>(mat.cols() - 1);
                }

                if (block.blockCols < 1)
                {
                    block.blockCols = 1;
                }
                else if (block.blockCols >= mat.cols() - block.startCol)
                {
                    block.blockCols = static_cast<int>(mat.cols() - block.startCol);
                }
            }

            initMatrix = mat;
            matrix = initMatrix;
            for (auto& block : blocks)
            {
                block.matrix = &matrix;
            }
            for (auto& outputPin : outputPins)
            {
                auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPin.id);
                for (auto& connectedLink : connectedLinks)
                {
                    nm::RefreshLink(connectedLink->id);
                }
            }
        }
        if (ImGui::InputInt("# Subblocks", &nBlocks))
        {
            if (nBlocks < 0)
            {
                nBlocks = 0;
            }
            LOG_DEBUG("{}: nBlocks changed to {}", nameId(), nBlocks);
            flow::ApplyChanges();
            updateNumberOfOutputPins();
        }

        if (ImGui::Button("Update Matrix with"))
        {
            matrix = initMatrix;
            for (auto& block : blocks)
            {
                block.matrix = &matrix;
            }
        }
        ImGui::SameLine();
        ImGui::TextUnformatted("Init Matrix:");
        if (ImGui::BeginTable("Init Matrix", static_cast<int>(initMatrix.cols() + 1),
                              ImGuiTableFlags_Borders | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0F, 0.0F)))
        {
            ImGui::TableSetupColumn("");
            for (int64_t col = 0; col < initMatrix.cols(); col++)
            {
                ImGui::TableSetupColumn(std::to_string(col).c_str());
            }
            ImGui::TableHeadersRow();
            for (int64_t row = 0; row < initMatrix.rows(); row++)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(std::to_string(row).c_str());
                ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                for (int64_t col = 0; col < initMatrix.cols(); col++)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(50);
                    if (ImGui::InputDouble(("##initMatrix(" + std::to_string(row) + ", " + std::to_string(col) + ")").c_str(),
                                           &initMatrix(row, col), 0.0, 0.0, "%.1f"))
                    {
                        flow::ApplyChanges();
                    }
                }
            }
            ImGui::EndTable();
        }
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader(("Current Matrix##" + std::to_string(size_t(id))).c_str()))
    {
        if (ImGui::BeginTable("Current Matrix", static_cast<int>(matrix.cols() + 1),
                              ImGuiTableFlags_Borders | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0F, 0.0F)))
        {
            ImGui::TableSetupColumn("");
            for (int64_t col = 0; col < matrix.cols(); col++)
            {
                ImGui::TableSetupColumn(std::to_string(col).c_str());
            }
            ImGui::TableHeadersRow();
            for (int64_t row = 0; row < matrix.rows(); row++)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(std::to_string(row).c_str());
                ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                for (int64_t col = 0; col < matrix.cols(); col++)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.1f", matrix(row, col));
                }
            }

            ImGui::EndTable();
        }
    }

    for (size_t blockIndex = 0; blockIndex < blocks.size(); blockIndex++)
    {
        Pin& outputPin = outputPins.at(blockIndex + 1);

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader((outputPin.name + "## " + std::to_string(size_t(id))).c_str()))
        {
            auto& block = blocks.at(blockIndex);

            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::TreeNode(("Options##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str()))
            {
                ImGui::InputText(("Pin name##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str(), &block.pinName);
                if (outputPin.name != block.pinName && !ImGui::IsItemActive())
                {
                    outputPin.name = block.pinName;
                    flow::ApplyChanges();
                    LOG_DEBUG("{}: # Pin name changed to {}", nameId(), outputPin.name);
                }
                if (ImGui::InputInt(("Start Row##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str(), &block.startRow))
                {
                    if (block.startRow < 0)
                    {
                        block.startRow = 0;
                    }
                    else if (block.startRow >= matrix.rows())
                    {
                        block.startRow = static_cast<int>(matrix.rows() - 1);
                    }

                    if (block.blockRows < 1)
                    {
                        block.blockRows = 1;
                    }
                    else if (block.blockRows >= matrix.rows() - block.startRow)
                    {
                        block.blockRows = static_cast<int>(matrix.rows() - block.startRow);
                    }

                    auto& outputPin = outputPins.at(blockIndex + 1);
                    auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPin.id);
                    for (auto& connectedLink : connectedLinks)
                    {
                        nm::RefreshLink(connectedLink->id);
                    }

                    flow::ApplyChanges();
                    LOG_DEBUG("{}: # Start Row of pin {} changed to {}", nameId(), outputPin.name, block.startRow);
                }
                if (ImGui::InputInt(("Start Col##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str(), &block.startCol))
                {
                    if (block.startCol < 0)
                    {
                        block.startCol = 0;
                    }
                    else if (block.startCol >= matrix.cols())
                    {
                        block.startCol = static_cast<int>(matrix.cols() - 1);
                    }

                    if (block.blockCols < 1)
                    {
                        block.blockCols = 1;
                    }
                    else if (block.blockCols >= matrix.cols() - block.startCol)
                    {
                        block.blockCols = static_cast<int>(matrix.cols() - block.startCol);
                    }

                    auto& outputPin = outputPins.at(blockIndex + 1);
                    auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPin.id);
                    for (auto& connectedLink : connectedLinks)
                    {
                        nm::RefreshLink(connectedLink->id);
                    }

                    flow::ApplyChanges();
                    LOG_DEBUG("{}: # Start Col of pin {} changed to {}", nameId(), outputPin.name, block.startCol);
                }
                if (ImGui::InputInt(("Block Rows##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str(), &block.blockRows))
                {
                    if (block.blockRows < 1)
                    {
                        block.blockRows = 1;
                    }
                    else if (block.blockRows >= matrix.rows() - block.startRow)
                    {
                        block.blockRows = static_cast<int>(matrix.rows() - block.startRow);
                    }

                    auto& outputPin = outputPins.at(blockIndex + 1);
                    auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPin.id);
                    for (auto& connectedLink : connectedLinks)
                    {
                        nm::RefreshLink(connectedLink->id);
                    }

                    flow::ApplyChanges();
                    LOG_DEBUG("{}: # Block rows of pin {} changed to {}", nameId(), outputPin.name, block.blockRows);
                }
                if (ImGui::InputInt(("Block Cols##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str(), &block.blockCols))
                {
                    if (block.blockCols < 1)
                    {
                        block.blockCols = 1;
                    }
                    else if (block.blockCols >= matrix.cols() - block.startCol)
                    {
                        block.blockCols = static_cast<int>(matrix.cols() - block.startCol);
                    }

                    auto& outputPin = outputPins.at(blockIndex + 1);
                    auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPin.id);
                    for (auto& connectedLink : connectedLinks)
                    {
                        nm::RefreshLink(connectedLink->id);
                    }

                    flow::ApplyChanges();
                    LOG_DEBUG("{}: # Block cols of pin {} changed to {}", nameId(), outputPin.name, block.blockCols);
                }

                ImGui::TreePop();
            }

            if (ImGui::BeginTable(("Current Block Matrix##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str(),
                                  static_cast<int>(matrix.cols() + 1), ImGuiTableFlags_Borders | ImGuiTableFlags_ColumnsWidthFixed, ImVec2(0.0F, 0.0F)))
            {
                ImGui::TableSetupColumn("");
                for (int64_t col = 0; col < matrix.cols(); col++)
                {
                    ImGui::TableSetupColumn(std::to_string(col).c_str());
                }
                ImGui::TableHeadersRow();
                for (int64_t row = 0; row < matrix.rows(); row++)
                {
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(std::to_string(row).c_str());
                    ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                    for (int64_t col = 0; col < matrix.cols(); col++)
                    {
                        ImGui::TableNextColumn();
                        if (row < block.startRow || row >= block.startRow + block.blockRows
                            || col < block.startCol || col >= block.startCol + block.blockCols)
                        {
                            ImGui::TextDisabled("%.1f", matrix(row, col));
                        }
                        else
                        {
                            auto blockMatrix = block();
                            ImGui::Text("%.1f", blockMatrix(row - block.startRow, col - block.startCol));
                        }
                    }
                }

                ImGui::EndTable();
            }
        }
    }
}

[[nodiscard]] json NAV::Matrix::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nRows"] = nRows;
    j["nCols"] = nCols;
    j["nBlocks"] = nBlocks;
    j["blocks"] = blocks;
    j["matrix"] = initMatrix;

    return j;
}

void NAV::Matrix::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nRows"))
    {
        j.at("nRows").get_to(nRows);
    }
    if (j.contains("nCols"))
    {
        j.at("nCols").get_to(nCols);
    }
    if (j.contains("nBlocks"))
    {
        j.at("nBlocks").get_to(nBlocks);
        updateNumberOfOutputPins();
    }
    if (j.contains("matrix"))
    {
        j.at("matrix").get_to(initMatrix);
        matrix = initMatrix;
        outputPins.at(OutputPortIndex_FullMatrix).data = Pin::PinData(&matrix);
    }
    if (j.contains("blocks"))
    {
        j.at("blocks").get_to(blocks);
        for (size_t blockIndex = 0; blockIndex < blocks.size(); blockIndex++)
        {
            blocks.at(blockIndex).matrix = &matrix;
            outputPins.at(blockIndex + 1).data = Pin::PinData(&blocks.at(blockIndex));
        }
    }
}

bool NAV::Matrix::initialize()
{
    LOG_TRACE("{}: called", nameId());

    matrix = initMatrix;
    for (auto& block : blocks)
    {
        block.matrix = &matrix;
    }

    return true;
}

void NAV::Matrix::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::Matrix::updateNumberOfOutputPins()
{
    while (outputPins.size() - 1 < static_cast<size_t>(nBlocks))
    {
        blocks.emplace_back(matrix, std::to_string(blocks.size() + 1), 0, 0, initMatrix.rows(), initMatrix.cols());
        nm::CreateOutputPin(this, std::to_string(blocks.size()).c_str(), Pin::Type::Matrix, "BlockMatrix", &blocks.back());
    }
    while (outputPins.size() - 1 > static_cast<size_t>(nBlocks))
    {
        auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.back().id);
        for (Link* link : connectedLinks)
        {
            nm::DeleteLink(link->id);
        }
        outputPins.pop_back();
        blocks.pop_back();
    }

    for (size_t blockIndex = 0; blockIndex < blocks.size(); blockIndex++)
    {
        blocks.at(blockIndex).matrix = &matrix;
        outputPins.at(blockIndex + 1).data = Pin::PinData(&blocks.at(blockIndex));
    }
    for (auto& outputPin : outputPins)
    {
        auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPin.id);
        for (auto& connectedLink : connectedLinks)
        {
            nm::RefreshLink(connectedLink->id);
        }
    }
}

bool NAV::Matrix::onCreateLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    return true;
}

void NAV::Matrix::onDeleteLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));
}

void NAV::Matrix::notifyOnOutputValueChanged(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(link->startPinId), size_t(link->endPinId));

        if (link->startPinId == outputPins.front().id) // Change on the Eigen::MatrixXd Pin notified
        {
            for (size_t i = 1; i < outputPins.size(); i++) // Loop through all MatrixBlock pins and trigger a notify
            {
                for (auto& [node, callback, linkId] : outputPins.at(i).notifyFunc)
                {
                    if (nm::showFlowWhenNotifyingValueChange)
                    {
                        ax::NodeEditor::Flow(linkId);
                    }

                    std::invoke(callback, node, linkId);
                }
            }
        }
        else // Change on any MatrixBlock Pin notified
        {
            size_t pinIndex = pinIndexFromId(link->startPinId);

            auto& block = blocks.at(pinIndex - 1);

            for (size_t i = 0; i < outputPins.size(); i++) // Loop through all pins and trigger a notify
            {
                if (i == pinIndex) // Don't trigger notify on self, as this is done already by notifyInputValueChanged
                {
                    continue;
                }

                if (i == 0 // Trigger notify on Eigen::MatrixXd pin
                    ||     // Check if modified block is part of any other subblock
                    (((blocks.at(i - 1).startRow <= block.startRow && blocks.at(i - 1).startRow + blocks.at(i - 1).blockRows > block.startRow)
                      || (blocks.at(i - 1).startRow >= block.startRow && blocks.at(i - 1).startRow < block.startRow + block.blockRows))
                     && ((blocks.at(i - 1).startCol <= block.startCol && blocks.at(i - 1).startCol + blocks.at(i - 1).blockCols > block.startCol)
                         || (blocks.at(i - 1).startCol >= block.startCol && blocks.at(i - 1).startCol < block.startCol + block.blockCols))))
                {
                    for (auto& [node, callback, linkId] : outputPins.at(i).notifyFunc)
                    {
                        if (nm::showFlowWhenNotifyingValueChange)
                        {
                            ax::NodeEditor::Flow(linkId);
                        }

                        std::invoke(callback, node, linkId);
                    }
                }
            }
        }
    }
}