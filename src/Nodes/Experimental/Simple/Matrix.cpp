#include "Matrix.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/NodeEditorApplication.hpp"

NAV::experimental::Matrix::Matrix()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 330, 420 };
    kind = Kind::Simple;

    nm::CreateOutputPin(this, "", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_matrix);

    _initMatrix = Eigen::MatrixXd::Zero(_nRows, _nCols);
    _matrix = _initMatrix;

    updateNumberOfOutputPins();
}

NAV::experimental::Matrix::~Matrix()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::experimental::Matrix::typeStatic()
{
    return "Matrix";
}

std::string NAV::experimental::Matrix::type() const
{
    return typeStatic();
}

std::string NAV::experimental::Matrix::category()
{
    return "Experimental/Simple";
}

void NAV::experimental::Matrix::guiConfig()
{
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader(("Options##" + std::to_string(size_t(id))).c_str()))
    {
        if (ImGui::InputInt("Rows", &_nRows))
        {
            if (_nRows < 1)
            {
                _nRows = 1;
            }
            LOG_DEBUG("{}: # Rows changed to {}", nameId(), _nRows);
            flow::ApplyChanges();
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(_nRows, _nCols);
            for (int64_t row = 0; row < _initMatrix.rows() && row < mat.rows(); row++)
            {
                for (int64_t col = 0; col < _initMatrix.cols() && col < mat.cols(); col++)
                {
                    mat(row, col) = _initMatrix(row, col);
                }
            }
            for (auto& block : _blocks)
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

            _initMatrix = mat;
            _matrix = _initMatrix;
            for (auto& block : _blocks)
            {
                block.matrix = &_matrix;
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
        if (ImGui::InputInt("Cols", &_nCols))
        {
            if (_nCols < 1)
            {
                _nCols = 1;
            }
            LOG_DEBUG("{}: # Cols changed to {}", nameId(), _nCols);
            flow::ApplyChanges();
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(_nRows, _nCols);
            for (int64_t row = 0; row < _initMatrix.rows() && row < mat.rows(); row++)
            {
                for (int64_t col = 0; col < _initMatrix.cols() && col < mat.cols(); col++)
                {
                    mat(row, col) = _initMatrix(row, col);
                }
            }

            for (auto& block : _blocks)
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

            _initMatrix = mat;
            _matrix = _initMatrix;
            for (auto& block : _blocks)
            {
                block.matrix = &_matrix;
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
        if (ImGui::InputInt("# Subblocks", &_nBlocks))
        {
            if (_nBlocks < 0)
            {
                _nBlocks = 0;
            }
            LOG_DEBUG("{}: nBlocks changed to {}", nameId(), _nBlocks);
            flow::ApplyChanges();
            updateNumberOfOutputPins();
        }

        if (ImGui::Button("Update Matrix with"))
        {
            _matrix = _initMatrix;
            for (auto& block : _blocks)
            {
                block.matrix = &_matrix;
            }
        }
        ImGui::SameLine();
        ImGui::TextUnformatted("Init Matrix:");
        if (ImGui::BeginTable("Init Matrix", static_cast<int>(_initMatrix.cols() + 1),
                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
        {
            ImGui::TableSetupColumn("");
            for (int64_t col = 0; col < _initMatrix.cols(); col++)
            {
                ImGui::TableSetupColumn(std::to_string(col).c_str());
            }
            ImGui::TableHeadersRow();
            for (int64_t row = 0; row < _initMatrix.rows(); row++)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(std::to_string(row).c_str());
                ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                for (int64_t col = 0; col < _initMatrix.cols(); col++)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(50 * gui::NodeEditorApplication::windowFontRatio());
                    if (ImGui::InputDouble(("##initMatrix(" + std::to_string(row) + ", " + std::to_string(col) + ")").c_str(),
                                           &_initMatrix(row, col), 0.0, 0.0, "%.1f"))
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
        if (ImGui::BeginTable("Current Matrix", static_cast<int>(_matrix.cols() + 1),
                              ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
        {
            ImGui::TableSetupColumn("");
            for (int64_t col = 0; col < _matrix.cols(); col++)
            {
                ImGui::TableSetupColumn(std::to_string(col).c_str());
            }
            ImGui::TableHeadersRow();
            for (int64_t row = 0; row < _matrix.rows(); row++)
            {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(std::to_string(row).c_str());
                ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                for (int64_t col = 0; col < _matrix.cols(); col++)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.1f", _matrix(row, col));
                }
            }

            ImGui::EndTable();
        }
    }

    for (size_t blockIndex = 0; blockIndex < _blocks.size(); blockIndex++)
    {
        Pin& outputPin = outputPins.at(blockIndex + 1);

        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader((outputPin.name + "## " + std::to_string(size_t(id))).c_str()))
        {
            auto& block = _blocks.at(blockIndex);

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
                    else if (block.startRow >= _matrix.rows())
                    {
                        block.startRow = static_cast<int>(_matrix.rows() - 1);
                    }

                    if (block.blockRows < 1)
                    {
                        block.blockRows = 1;
                    }
                    else if (block.blockRows >= _matrix.rows() - block.startRow)
                    {
                        block.blockRows = static_cast<int>(_matrix.rows() - block.startRow);
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
                    else if (block.startCol >= _matrix.cols())
                    {
                        block.startCol = static_cast<int>(_matrix.cols() - 1);
                    }

                    if (block.blockCols < 1)
                    {
                        block.blockCols = 1;
                    }
                    else if (block.blockCols >= _matrix.cols() - block.startCol)
                    {
                        block.blockCols = static_cast<int>(_matrix.cols() - block.startCol);
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
                    else if (block.blockRows >= _matrix.rows() - block.startRow)
                    {
                        block.blockRows = static_cast<int>(_matrix.rows() - block.startRow);
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
                    else if (block.blockCols >= _matrix.cols() - block.startCol)
                    {
                        block.blockCols = static_cast<int>(_matrix.cols() - block.startCol);
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
                                  static_cast<int>(_matrix.cols() + 1), ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
            {
                ImGui::TableSetupColumn("");
                for (int64_t col = 0; col < _matrix.cols(); col++)
                {
                    ImGui::TableSetupColumn(std::to_string(col).c_str());
                }
                ImGui::TableHeadersRow();
                for (int64_t row = 0; row < _matrix.rows(); row++)
                {
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(std::to_string(row).c_str());
                    ImU32 cell_bg_color = ImGui::GetColorU32(ImGui::GetStyle().Colors[ImGuiCol_TableHeaderBg]);
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, cell_bg_color);
                    for (int64_t col = 0; col < _matrix.cols(); col++)
                    {
                        ImGui::TableNextColumn();
                        if (row < block.startRow || row >= block.startRow + block.blockRows
                            || col < block.startCol || col >= block.startCol + block.blockCols)
                        {
                            ImGui::TextDisabled("%.1f", _matrix(row, col));
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

[[nodiscard]] json NAV::experimental::Matrix::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nRows"] = _nRows;
    j["nCols"] = _nCols;
    j["nBlocks"] = _nBlocks;
    j["blocks"] = _blocks;
    j["matrix"] = _initMatrix;

    return j;
}

void NAV::experimental::Matrix::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("nRows"))
    {
        j.at("nRows").get_to(_nRows);
    }
    if (j.contains("nCols"))
    {
        j.at("nCols").get_to(_nCols);
    }
    if (j.contains("nBlocks"))
    {
        j.at("nBlocks").get_to(_nBlocks);
        updateNumberOfOutputPins();
    }
    if (j.contains("matrix"))
    {
        j.at("matrix").get_to(_initMatrix);
        _matrix = _initMatrix;
        outputPins.at(OUTPUT_PORT_INDEX_FULL_MATRIX).data = Pin::PinData(&_matrix);
    }
    if (j.contains("blocks"))
    {
        j.at("blocks").get_to(_blocks);
        for (size_t blockIndex = 0; blockIndex < _blocks.size(); blockIndex++)
        {
            _blocks.at(blockIndex).matrix = &_matrix;
            outputPins.at(blockIndex + 1).data = Pin::PinData(&_blocks.at(blockIndex));
        }
    }
}

bool NAV::experimental::Matrix::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _matrix = _initMatrix;
    for (auto& block : _blocks)
    {
        block.matrix = &_matrix;
    }

    return true;
}

void NAV::experimental::Matrix::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::experimental::Matrix::updateNumberOfOutputPins()
{
    while (outputPins.size() - 1 < static_cast<size_t>(_nBlocks))
    {
        _blocks.emplace_back(_matrix, std::to_string(_blocks.size() + 1), 0, 0, _initMatrix.rows(), _initMatrix.cols());
        nm::CreateOutputPin(this, std::to_string(_blocks.size()).c_str(), Pin::Type::Matrix, { "BlockMatrix" }, &_blocks.back());
    }
    while (outputPins.size() - 1 > static_cast<size_t>(_nBlocks))
    {
        auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.back().id);
        for (Link* link : connectedLinks)
        {
            nm::DeleteLink(link->id);
        }
        outputPins.pop_back();
        _blocks.pop_back();
    }

    for (size_t blockIndex = 0; blockIndex < _blocks.size(); blockIndex++)
    {
        _blocks.at(blockIndex).matrix = &_matrix;
        outputPins.at(blockIndex + 1).data = Pin::PinData(&_blocks.at(blockIndex));
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

bool NAV::experimental::Matrix::onCreateLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    return true;
}

void NAV::experimental::Matrix::onDeleteLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));
}

void NAV::experimental::Matrix::notifyOnOutputValueChanged(ax::NodeEditor::LinkId linkId)
{
    if (Link* link = nm::FindLink(linkId))
    {
        LOG_DATA("{}: called for {} ==> {}", nameId(), size_t(link->startPinId), size_t(link->endPinId));

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

            auto& block = _blocks.at(pinIndex - 1);

            for (size_t i = 0; i < outputPins.size(); i++) // Loop through all pins and trigger a notify
            {
                if (i == pinIndex) // Don't trigger notify on self, as this is done already by notifyInputValueChanged
                {
                    continue;
                }

                if (i == 0 // Trigger notify on Eigen::MatrixXd pin
                    ||     // Check if modified block is part of any other subblock
                    (((_blocks.at(i - 1).startRow <= block.startRow && _blocks.at(i - 1).startRow + _blocks.at(i - 1).blockRows > block.startRow)
                      || (_blocks.at(i - 1).startRow >= block.startRow && _blocks.at(i - 1).startRow < block.startRow + block.blockRows))
                     && ((_blocks.at(i - 1).startCol <= block.startCol && _blocks.at(i - 1).startCol + _blocks.at(i - 1).blockCols > block.startCol)
                         || (_blocks.at(i - 1).startCol >= block.startCol && _blocks.at(i - 1).startCol < block.startCol + block.blockCols))))
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