/// @file LooselyCoupledKF.hpp
/// @brief Kalman Filter class for the loosely coupled INS/GNSS integration
/// @author T. Topp (topp@ins.uni-stuttgart.de) and M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-08-04

#pragma once

#include "Nodes/Node.hpp"
#include "NodeData/State/PosVelAtt.hpp"

namespace NAV
{
class LooselyCoupledKF : public Node
{
    public:
        /// @brief Default constructor
        LooselyCoupledKF();
        /// @brief Destructor
        ~LooselyCoupledKF() override;
        /// @brief Copy constructor
        LooselyCoupledKF(const LooselyCoupledKF&) = delete;
        /// @brief Move constructor
        LooselyCoupledKF(LooselyCoupledKF&&) = delete;
        /// @brief Copy assignment operator
        LooselyCoupledKF& operator=(const LooselyCoupledKF&) = delete;
        /// @brief Move assignment operator
        LooselyCoupledKF& operator=(LooselyCoupledKF&&) = delete;
        /// @brief String representation of the class type
        [[nodiscard]] static std::string typeStatic();

        /// @brief String representation of the class type
        [[nodiscard]] std::string type() const override;

        /// @brief String representation of the class category
        [[nodiscard]] static std::string category();
        
        /// @brief ImGui config window which is shown on double click
        /// @attention Don't forget to set hasConfig to true in the constructor of the node
        void guiConfig() override;

        /// @brief Saves the node into a json object
        [[nodiscard]] json save() const override;

        /// @brief Restores the node from a json object
        /// @param[in] j Json object with the node state
        void restore(const json& j) override;

    private:
        constexpr static size_t OutputPortIndex_PosVelAtt__t0 = 1; ///< @brief Flow (PosVelAtt)

        /// @brief Initialize the node
        bool initialize() override;

        /// @brief Deinitialize the node
        void deinitialize() override;

        /// @brief Receive Function for the measured state PosVelAtt at the time t_k
        /// @param[in] nodeData State vector (PosVelAtt)
        /// @param[in] linkId Id of the link over which the data is received
        void recvState__t0(const std::shared_ptr<NodeData>& NodeData, ax::NodeEditor::LinkId linkId);

        /// @brief Filters the observation data from INS and GNSS
        void filterObservation();

        /// Observation at time t_k
        std::shared_ptr<PosVelAtt> posVelAtt__t0 = nullptr;
};
} // namespace NAV