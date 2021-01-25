#include "Matrix.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::Matrix::Matrix()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    color = ImColor(255, 128, 128);
    hasConfig = true;
    kind = Kind::Simple;

    nm::CreateOutputPin(this, "", Pin::Type::Matrix, "Eigen::MatrixXd", &matrix);

    initMatrix = Eigen::MatrixXd(nRows, nCols);
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
            initializeNode();
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
            initializeNode();
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
}

[[nodiscard]] json NAV::Matrix::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["nRows"] = nRows;
    j["nCols"] = nCols;
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

bool NAV::Matrix::onCreateLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    return true;
}

void NAV::Matrix::onDeleteLink([[maybe_unused]] Pin* startPin, [[maybe_unused]] Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));
}