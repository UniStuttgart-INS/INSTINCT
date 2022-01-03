#include "Transformation.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"

NAV::Transformation::Transformation()
{
    name = "to";

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 305, 70 };
    kind = Kind::Simple;

    nm::CreateInputPin(this, "ECEF", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" }, &Transformation::notifyOnInputValueChanged);
    nm::CreateOutputPin(this, "LLA [rad]", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &matrix);
}

NAV::Transformation::~Transformation()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Transformation::typeStatic()
{
    return "Transformation";
}

std::string NAV::Transformation::type() const
{
    return typeStatic();
}

std::string NAV::Transformation::category()
{
    return "Simple";
}

void NAV::Transformation::guiConfig()
{
    ImGui::SetNextItemWidth(280.0F);
    if (ImGui::Combo(("##Transformation" + std::to_string(size_t(id))).c_str(),
                     reinterpret_cast<int*>(&selectedTransformation),
                     "ECEF to LLA [rad]\0"
                     "ECEF to LLA [deg]\0"
                     "LLA [rad] to ECEF [m]\0"
                     "LLA [deg] to ECEF [m]\0"
                     "Quat_nb to Roll, Pitch, Yaw [rad]\0"
                     "Quat_nb to Roll, Pitch, Yaw [deg]\0"
                     "Roll, Pitch, Yaw [rad] to Quat_nb\0"
                     "Roll, Pitch, Yaw [deg] to Quat_nb\0"
                     "ECEF to NED\0"
                     "NED to ECEF\0"
                     //  "Conjugate\0"
                     //  "Transpose\0"
                     "\0"))
    {
        switch (selectedTransformation)
        {
        case Type::ECEF_2_LLArad:
            inputPins.at(InputPortIndex_Matrix).name = "ECEF";
            outputPins.at(OutputPortIndex_Matrix).name = "LLA [rad]";
            break;
        case Type::ECEF_2_LLAdeg:
            inputPins.at(InputPortIndex_Matrix).name = "ECEF";
            outputPins.at(OutputPortIndex_Matrix).name = "LLA [deg]";
            break;
        case Type::LLArad_2_ECEF:
            inputPins.at(InputPortIndex_Matrix).name = "LLA [rad]";
            outputPins.at(OutputPortIndex_Matrix).name = "ECEF";
            break;
        case Type::LLAdeg_2_ECEF:
            inputPins.at(InputPortIndex_Matrix).name = "LLA [deg]";
            outputPins.at(OutputPortIndex_Matrix).name = "ECEF";
            break;
        case Type::Quat_nb_2_RollPitchYawRad:
            inputPins.at(InputPortIndex_Matrix).name = "Quat_nb";
            outputPins.at(OutputPortIndex_Matrix).name = "RollPitchYaw [rad]";
            break;
        case Type::Quat_nb_2_RollPitchYawDeg:
            inputPins.at(InputPortIndex_Matrix).name = "Quat_nb";
            outputPins.at(OutputPortIndex_Matrix).name = "RollPitchYaw [deg]";
            break;
        case Type::RollPitchYawRad_2_Quat_nb:
            inputPins.at(InputPortIndex_Matrix).name = "RollPitchYaw [rad]";
            outputPins.at(OutputPortIndex_Matrix).name = "Quat_nb";
            break;
        case Type::RollPitchYawDeg_2_Quat_nb:
            inputPins.at(InputPortIndex_Matrix).name = "RollPitchYaw [deg]";
            outputPins.at(OutputPortIndex_Matrix).name = "Quat_nb";
            break;
        }
        if (!inputMatrixHasSize(nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Matrix).id)))
        {
            if (Link* connectedLink = nm::FindConnectedLinkToInputPin(inputPins.at(InputPortIndex_Matrix).id))
            {
                nm::DeleteLink(connectedLink->id);
            }
        }

        updateMatrixSize();

        auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Matrix).id);
        for (auto& connectedLink : connectedLinks)
        {
            nm::RefreshLink(connectedLink->id);
        }

        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::Transformation::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["selectedTransformation"] = selectedTransformation;

    return j;
}

void NAV::Transformation::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("selectedTransformation"))
    {
        j.at("selectedTransformation").get_to(selectedTransformation);
    }
}

bool NAV::Transformation::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::Transformation::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::Transformation::onCreateLink(Pin* startPin, Pin* endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

    if (startPin && endPin && endPin->id == inputPins.at(InputPortIndex_Matrix).id)
    {
        if (!inputMatrixHasSize(startPin))
        {
            return false;
        }

        updateMatrixSize();

        auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.at(OutputPortIndex_Matrix).id);
        for (auto& connectedLink : connectedLinks)
        {
            nm::RefreshLink(connectedLink->id);
        }
    }

    return true;
}

void NAV::Transformation::notifyOnOutputValueChanged(ax::NodeEditor::LinkId linkId)
{
    if ([[maybe_unused]] Link* link = nm::FindLink(linkId))
    {
        LOG_DATA("{}: called for {} ==> {}", nameId(), size_t(link->startPinId), size_t(link->endPinId));
    }

    if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Matrix).id))
    {
        if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* inputMatrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Matrix))
            {
                switch (selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    *inputMatrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(matrix.data());
                    lla.x() = trafo::deg2rad(lla.x());
                    lla.y() = trafo::deg2rad(lla.y());
                    *inputMatrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::LLArad_2_ECEF:
                    *inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                    *inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data()));
                    (*inputMatrix)(0, 0) = trafo::rad2deg((*inputMatrix)(0, 0));
                    (*inputMatrix)(1, 0) = trafo::rad2deg((*inputMatrix)(1, 0));
                    break;
                case Type::Quat_nb_2_RollPitchYawRad:
                    *inputMatrix = trafo::quat_nb(matrix(0, 0), matrix(1, 0), matrix(2, 0)).coeffs();
                    break;
                case Type::Quat_nb_2_RollPitchYawDeg:
                    *inputMatrix = trafo::quat_nb(trafo::deg2rad(matrix(0, 0)),
                                                  trafo::deg2rad(matrix(1, 0)),
                                                  trafo::deg2rad(matrix(2, 0)))
                                       .coeffs();
                    break;
                case Type::RollPitchYawRad_2_Quat_nb:
                    *inputMatrix = trafo::quat2eulerZYX(Eigen::Quaterniond(matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0)));
                    break;
                case Type::RollPitchYawDeg_2_Quat_nb:
                    *inputMatrix = trafo::rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond(matrix(0, 0),
                                                                                          matrix(1, 0),
                                                                                          matrix(2, 0),
                                                                                          matrix(3, 0))));
                    break;
                }
                notifyInputValueChanged(InputPortIndex_Matrix);
            }
        }
        else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Matrix))
            {
                auto inputMatrix = (*value)();

                switch (selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    inputMatrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(matrix.data());
                    lla.x() = trafo::deg2rad(lla.x());
                    lla.y() = trafo::deg2rad(lla.y());
                    inputMatrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::LLArad_2_ECEF:
                    inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                    inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(matrix.data()));
                    inputMatrix(0, 0) = trafo::rad2deg(inputMatrix(0, 0));
                    inputMatrix(1, 0) = trafo::rad2deg(inputMatrix(1, 0));
                    break;
                case Type::Quat_nb_2_RollPitchYawRad:
                    inputMatrix = trafo::quat_nb(matrix(0, 0), matrix(1, 0), matrix(2, 0)).coeffs();
                    break;
                case Type::Quat_nb_2_RollPitchYawDeg:
                    inputMatrix = trafo::quat_nb(trafo::deg2rad(matrix(0, 0)),
                                                 trafo::deg2rad(matrix(1, 0)),
                                                 trafo::deg2rad(matrix(2, 0)))
                                      .coeffs();
                    break;
                case Type::RollPitchYawRad_2_Quat_nb:
                    inputMatrix = trafo::quat2eulerZYX(Eigen::Quaterniond(matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0)));
                    break;
                case Type::RollPitchYawDeg_2_Quat_nb:
                    inputMatrix = trafo::rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond(matrix(0, 0),
                                                                                         matrix(1, 0),
                                                                                         matrix(2, 0),
                                                                                         matrix(3, 0))));
                    break;
                }
                notifyInputValueChanged(InputPortIndex_Matrix);
            }
        }
    }
}

void NAV::Transformation::notifyOnInputValueChanged(ax::NodeEditor::LinkId linkId)
{
    if ([[maybe_unused]] Link* link = nm::FindLink(linkId))
    {
        LOG_DATA("{}: called for {} ==> {}", nameId(), size_t(link->startPinId), size_t(link->endPinId));
    }

    if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(InputPortIndex_Matrix).id))
    {
        if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* inputMatrix = getInputValue<Eigen::MatrixXd>(InputPortIndex_Matrix))
            {
                switch (selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix->data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                    matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix->data()));
                    matrix(0, 0) = trafo::rad2deg(matrix(0, 0));
                    matrix(1, 0) = trafo::rad2deg(matrix(1, 0));
                    break;
                case Type::LLArad_2_ECEF:
                    matrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix->data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(inputMatrix->data());
                    lla.x() = trafo::deg2rad(lla.x());
                    lla.y() = trafo::deg2rad(lla.y());
                    matrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::Quat_nb_2_RollPitchYawRad:
                    matrix = trafo::quat2eulerZYX(Eigen::Quaterniond((*inputMatrix)(0, 0), (*inputMatrix)(1, 0), (*inputMatrix)(2, 0), (*inputMatrix)(3, 0)));
                    break;
                case Type::Quat_nb_2_RollPitchYawDeg:
                    matrix = trafo::rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond((*inputMatrix)(0, 0),
                                                                                    (*inputMatrix)(1, 0),
                                                                                    (*inputMatrix)(2, 0),
                                                                                    (*inputMatrix)(3, 0))));
                    break;
                case Type::RollPitchYawRad_2_Quat_nb:
                    matrix = trafo::quat_nb((*inputMatrix)(0, 0), (*inputMatrix)(1, 0), (*inputMatrix)(2, 0)).coeffs();
                    break;
                case Type::RollPitchYawDeg_2_Quat_nb:
                    matrix = trafo::quat_nb(trafo::deg2rad((*inputMatrix)(0, 0)),
                                            trafo::deg2rad((*inputMatrix)(1, 0)),
                                            trafo::deg2rad((*inputMatrix)(2, 0)))
                                 .coeffs();
                    break;
                }
                notifyOutputValueChanged(OutputPortIndex_Matrix);
            }
        }
        else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* value = getInputValue<BlockMatrix>(InputPortIndex_Matrix))
            {
                auto inputMatrix = (*value)();

                switch (selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix.data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                    matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix.data()));
                    matrix(0, 0) = trafo::rad2deg(matrix(0, 0));
                    matrix(1, 0) = trafo::rad2deg(matrix(1, 0));
                    break;
                case Type::LLArad_2_ECEF:
                    matrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix.data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(inputMatrix.data());
                    lla.x() = trafo::deg2rad(lla.x());
                    lla.y() = trafo::deg2rad(lla.y());
                    matrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::Quat_nb_2_RollPitchYawRad:
                    matrix = trafo::quat2eulerZYX(Eigen::Quaterniond(inputMatrix(0, 0), inputMatrix(1, 0), inputMatrix(2, 0), inputMatrix(3, 0)));
                    break;
                case Type::Quat_nb_2_RollPitchYawDeg:
                    matrix = trafo::rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond(inputMatrix(0, 0),
                                                                                    inputMatrix(1, 0),
                                                                                    inputMatrix(2, 0),
                                                                                    inputMatrix(3, 0))));
                    break;
                case Type::RollPitchYawRad_2_Quat_nb:
                    matrix = trafo::quat_nb(inputMatrix(0, 0), inputMatrix(1, 0), inputMatrix(2, 0)).coeffs();
                    break;
                case Type::RollPitchYawDeg_2_Quat_nb:
                    matrix = trafo::quat_nb(trafo::deg2rad(inputMatrix(0, 0)),
                                            trafo::deg2rad(inputMatrix(1, 0)),
                                            trafo::deg2rad(inputMatrix(2, 0)))
                                 .coeffs();
                    break;
                }
                notifyOutputValueChanged(OutputPortIndex_Matrix);
            }
        }
    }
}

bool NAV::Transformation::inputMatrixHasSize(Pin* startPin)
{
    if (!startPin)
    {
        return false;
    }

    int64_t rows = 0;
    int64_t cols = 0;

    switch (selectedTransformation)
    {
    case Type::ECEF_2_LLArad:
    case Type::ECEF_2_LLAdeg:
    case Type::LLArad_2_ECEF:
    case Type::LLAdeg_2_ECEF:
    case Type::RollPitchYawRad_2_Quat_nb:
    case Type::RollPitchYawDeg_2_Quat_nb:
        rows = 3;
        cols = 1;
        break;
    case Type::Quat_nb_2_RollPitchYawRad:
    case Type::Quat_nb_2_RollPitchYawDeg:
        rows = 4;
        cols = 1;
        break;
    }

    if (startPin->dataIdentifier.front() == "Eigen::MatrixXd")
    {
        if (const auto* pval = std::get_if<void*>(&startPin->data))
        {
            if (auto* mat = static_cast<Eigen::MatrixXd*>(*pval))
            {
                if (mat->rows() == rows && mat->cols() == cols)
                {
                    return true;
                }

                LOG_ERROR("{}: The Matrix needs to have the size {}x{}", nameId(), rows, cols);
            }
        }
    }
    else if (startPin->dataIdentifier.front() == "BlockMatrix")
    {
        if (const auto* pval = std::get_if<void*>(&startPin->data))
        {
            if (auto* block = static_cast<BlockMatrix*>(*pval))
            {
                auto mat = (*block)();
                if (mat.rows() == rows && mat.cols() == cols)
                {
                    return true;
                }

                LOG_ERROR("{}: The Matrix needs to have the size {}x{}", nameId(), rows, cols);
            }
        }
    }

    return false;
}

void NAV::Transformation::updateMatrixSize()
{
    switch (selectedTransformation)
    {
    case Type::ECEF_2_LLArad:
    case Type::ECEF_2_LLAdeg:
    case Type::LLArad_2_ECEF:
    case Type::LLAdeg_2_ECEF:
    case Type::Quat_nb_2_RollPitchYawRad:
    case Type::Quat_nb_2_RollPitchYawDeg:
        matrix = Eigen::MatrixXd::Zero(3, 1);
        break;
    case Type::RollPitchYawRad_2_Quat_nb:
    case Type::RollPitchYawDeg_2_Quat_nb:
        matrix = Eigen::MatrixXd::Zero(4, 1);
        break;
    }
}