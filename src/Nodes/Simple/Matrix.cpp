#include "Matrix.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

namespace NAV
{
void to_json(json& j, const Matrix::Block& data)
{
    j = data.to_json();
}
void from_json(const json& j, Matrix::Block& data)
{
    data.from_json(j);
}

} // namespace NAV

NAV::Matrix::Block::Block(Eigen::MatrixXd& matrix, std::string pinName, int startRow, int startCol, int blockRows, int blockCols)
    : matrix(&matrix), pinName(std::move(pinName)), startRow(startRow), startCol(startCol), blockRows(blockRows), blockCols(blockCols) {}

Eigen::Block<Eigen::MatrixXd> NAV::Matrix::Block::operator()()
{
    return matrix->block(startRow, startCol, blockRows, blockCols);
}

[[nodiscard]] json NAV::Matrix::Block::to_json() const
{
    return json{
        { "pinName", pinName },
        { "startRow", startRow },
        { "startCol", startCol },
        { "blockRows", blockRows },
        { "blockCols", blockCols },
    };
}
void NAV::Matrix::Block::from_json(const json& j)
{
    if (j.contains("pinName"))
    {
        j.at("pinName").get_to(pinName);
    }
    if (j.contains("startRow"))
    {
        j.at("startRow").get_to(startRow);
    }
    if (j.contains("startCol"))
    {
        j.at("startCol").get_to(startCol);
    }
    if (j.contains("blockRows"))
    {
        j.at("blockRows").get_to(blockRows);
    }
    if (j.contains("blockCols"))
    {
        j.at("blockCols").get_to(blockCols);
    }
}

NAV::Matrix::Matrix()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;
    kind = Kind::Simple;

    nm::CreateOutputPin(this, "", Pin::Type::Matrix, "Eigen::MatrixXd", &matrix);

    initMatrix = Eigen::MatrixXd::Zero(nRows, nCols);

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
            initMatrix = mat;
            initializeNode(); // Updates the matrix
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
            initMatrix = mat;
            initializeNode(); // Updates the matrix
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
                for (int64_t col = 0; col < initMatrix.cols(); col++)
                {
                    ImGui::TableNextColumn();
                    ImGui::SetNextItemWidth(30);
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
                flow::ApplyChanges();
                LOG_DEBUG("{}: # Start Col of pin {} changed to {}", nameId(), outputPin.name, block.startCol);
            }
            if (ImGui::InputInt(("Block Rows##" + std::to_string(size_t(id)) + " - " + std::to_string(blockIndex)).c_str(), &block.blockRows))
            {
                if (block.blockRows < 1)
                {
                    block.blockRows = 1;
                }
                else if (block.blockRows >= matrix.rows())
                {
                    block.blockRows = static_cast<int>(matrix.rows());
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
                else if (block.blockCols >= matrix.cols())
                {
                    block.blockCols = static_cast<int>(matrix.cols());
                }
                flow::ApplyChanges();
                LOG_DEBUG("{}: # Block cols of pin {} changed to {}", nameId(), outputPin.name, block.blockCols);
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
    if (j.contains("blocks"))
    {
        j.at("blocks").get_to(blocks);
    }
    if (j.contains("matrix"))
    {
        j.at("matrix").get_to(initMatrix);
    }
}

bool NAV::Matrix::initialize()
{
    LOG_TRACE("{}: called", nameId());

    matrix = initMatrix;

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
        blocks.emplace_back(matrix, std::to_string(blocks.size() + 1), 0, 0, initMatrix.rows(), 1);
        nm::CreateOutputPin(this, std::to_string(blocks.size()).c_str(), Pin::Type::Matrix, "Matrix::Block", &blocks.back());
    }
    while (outputPins.size() - 1 > static_cast<size_t>(nBlocks))
    {
        auto connectedLinks = nm::FindConnectedLinksToPin(inputPins.back().id);
        for (Link* link : connectedLinks)
        {
            nm::DeleteLink(link->id);
        }
        inputPins.pop_back();

        blocks.pop_back();
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