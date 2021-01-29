#include "Eigen.hpp"

NAV::BlockMatrix::BlockMatrix(Eigen::MatrixXd& matrix, std::string pinName, int startRow, int startCol, int blockRows, int blockCols)
    : matrix(&matrix), pinName(std::move(pinName)), startRow(startRow), startCol(startCol), blockRows(blockRows), blockCols(blockCols) {}

Eigen::Block<Eigen::MatrixXd> NAV::BlockMatrix::operator()()
{
    return matrix->block(startRow, startCol, blockRows, blockCols);
}

[[nodiscard]] json NAV::BlockMatrix::to_json() const
{
    return json{
        { "pinName", pinName },
        { "startRow", startRow },
        { "startCol", startCol },
        { "blockRows", blockRows },
        { "blockCols", blockCols },
    };
}

void NAV::BlockMatrix::from_json(const json& j)
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

void NAV::to_json(json& j, const BlockMatrix& data)
{
    j = data.to_json();
}
void NAV::from_json(const json& j, BlockMatrix& data)
{
    data.from_json(j);
}