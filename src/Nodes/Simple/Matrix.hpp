/// @file Matrix.hpp
/// @brief Provides a Matrix Object
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-25

#pragma once

#include "Nodes/Node.hpp"

#include "util/Eigen.hpp"

namespace NAV
{
class Matrix : public Node
{
  public:
    /// @brief Default constructor
    Matrix();
    /// @brief Destructor
    ~Matrix() override;
    /// @brief Copy constructor
    Matrix(const Matrix&) = delete;
    /// @brief Move constructor
    Matrix(Matrix&&) = delete;
    /// @brief Copy assignment operator
    Matrix& operator=(const Matrix&) = delete;
    /// @brief Move assignment operator
    Matrix& operator=(Matrix&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(Pin* startPin, Pin* endPin) override;

    /// @brief Called when a link is to be deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void onDeleteLink(Pin* startPin, Pin* endPin) override;

  private:
    struct BlockInfo
    {
        BlockInfo(int startRow, int startCol, int blockRows, int blockCols)
            : startRow(startRow), startCol(startCol), blockRows(blockRows), blockCols(blockCols) {}

        int startRow = 0;
        int startCol = 0;
        int blockRows = 1;
        int blockCols = 1;
    };

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Adds/Deletes Output Pins depending on the variable nBlocks
    void updateNumberOfOutputPins();

    Eigen::Block<Eigen::MatrixXd> block(size_t blockIndex = 0)
    {
        return matrix.block(blocks.at(blockIndex).startRow, blocks.at(blockIndex).startCol,
                            blocks.at(blockIndex).blockRows, blocks.at(blockIndex).blockCols);
    }

    /// Number of Rows
    int nRows = 3;
    /// Number of Columns
    int nCols = 3;
    /// Number of subblocks of the matrix
    int nBlocks = 0;

    /// List of subblocks
    std::vector<BlockInfo> blocks;

    /// The matrix object
    Eigen::MatrixXd matrix;

    /// The initial matrix set in the gui
    Eigen::MatrixXd initMatrix;
};

} // namespace NAV
