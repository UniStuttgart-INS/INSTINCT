#include "Transformation.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

NAV::Transformation::Transformation()
{
    name = "to";

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 305, 70 };
    kind = Kind::Simple;

    nm::CreateInputPin(this, "ECEF", Pin::Type::Matrix, { "Eigen::MatrixXd", "BlockMatrix" }, &Transformation::notifyOnInputValueChanged);
    nm::CreateOutputPin(this, "LLA [rad]", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_matrix);
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
                     reinterpret_cast<int*>(&_selectedTransformation),
                     "ECEF to LLA [rad]\0"
                     "ECEF to LLA [deg]\0"
                     "LLA [rad] to ECEF [m]\0"
                     "LLA [deg] to ECEF [m]\0"
                     "n_Quat_b to Roll, Pitch, Yaw [rad]\0"
                     "n_Quat_b to Roll, Pitch, Yaw [deg]\0"
                     "Roll, Pitch, Yaw [rad] to n_Quat_b\0"
                     "Roll, Pitch, Yaw [deg] to n_Quat_b\0"
                     "ECEF to NED\0"
                     "NED to ECEF\0"
                     //  "Conjugate\0"
                     //  "Transpose\0"
                     "\0"))
    {
        switch (_selectedTransformation)
        {
        case Type::ECEF_2_LLArad:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "ECEF";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "LLA [rad]";
            break;
        case Type::ECEF_2_LLAdeg:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "ECEF";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "LLA [deg]";
            break;
        case Type::LLArad_2_ECEF:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "LLA [rad]";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "ECEF";
            break;
        case Type::LLAdeg_2_ECEF:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "LLA [deg]";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "ECEF";
            break;
        case Type::n_Quat_b_2_RollPitchYawRad:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "n_Quat_b";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "RollPitchYaw [rad]";
            break;
        case Type::n_Quat_b_2_RollPitchYawDeg:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "n_Quat_b";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "RollPitchYaw [deg]";
            break;
        case Type::RollPitchYawRad_2_n_Quat_b:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "RollPitchYaw [rad]";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "n_Quat_b";
            break;
        case Type::RollPitchYawDeg_2_n_Quat_b:
            inputPins.at(INPUT_PORT_INDEX_MATRIX).name = "RollPitchYaw [deg]";
            outputPins.at(OUTPUT_PORT_INDEX_MATRIX).name = "n_Quat_b";
            break;
        }
        if (!inputMatrixHasSize(nm::FindConnectedPinToInputPin(inputPins.at(INPUT_PORT_INDEX_MATRIX).id)))
        {
            if (Link* connectedLink = nm::FindConnectedLinkToInputPin(inputPins.at(INPUT_PORT_INDEX_MATRIX).id))
            {
                nm::DeleteLink(connectedLink->id);
            }
        }

        updateMatrixSize();

        auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.at(OUTPUT_PORT_INDEX_MATRIX).id);
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

    j["selectedTransformation"] = _selectedTransformation;

    return j;
}

void NAV::Transformation::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("selectedTransformation"))
    {
        j.at("selectedTransformation").get_to(_selectedTransformation);
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

    if (startPin && endPin && endPin->id == inputPins.at(INPUT_PORT_INDEX_MATRIX).id)
    {
        if (!inputMatrixHasSize(startPin))
        {
            return false;
        }

        updateMatrixSize();

        auto connectedLinks = nm::FindConnectedLinksToOutputPin(outputPins.at(OUTPUT_PORT_INDEX_MATRIX).id);
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

    if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(INPUT_PORT_INDEX_MATRIX).id))
    {
        if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* inputMatrix = getInputValue<Eigen::MatrixXd>(INPUT_PORT_INDEX_MATRIX))
            {
                switch (_selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    *inputMatrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(_matrix.data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(_matrix.data());
                    lla.x() = deg2rad(lla.x());
                    lla.y() = deg2rad(lla.y());
                    *inputMatrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::LLArad_2_ECEF:
                    *inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(_matrix.data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                    *inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(_matrix.data()));
                    (*inputMatrix)(0, 0) = rad2deg((*inputMatrix)(0, 0));
                    (*inputMatrix)(1, 0) = rad2deg((*inputMatrix)(1, 0));
                    break;
                case Type::n_Quat_b_2_RollPitchYawRad:
                    *inputMatrix = trafo::n_Quat_b(_matrix(0, 0), _matrix(1, 0), _matrix(2, 0)).coeffs();
                    break;
                case Type::n_Quat_b_2_RollPitchYawDeg:
                    *inputMatrix = trafo::n_Quat_b(deg2rad(_matrix(0, 0)),
                                                   deg2rad(_matrix(1, 0)),
                                                   deg2rad(_matrix(2, 0)))
                                       .coeffs();
                    break;
                case Type::RollPitchYawRad_2_n_Quat_b:
                    *inputMatrix = trafo::quat2eulerZYX(Eigen::Quaterniond(_matrix(0, 0), _matrix(1, 0), _matrix(2, 0), _matrix(3, 0)));
                    break;
                case Type::RollPitchYawDeg_2_n_Quat_b:
                    *inputMatrix = rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond(_matrix(0, 0),
                                                                                   _matrix(1, 0),
                                                                                   _matrix(2, 0),
                                                                                   _matrix(3, 0))));
                    break;
                }
                notifyInputValueChanged(INPUT_PORT_INDEX_MATRIX);
            }
        }
        else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* value = getInputValue<BlockMatrix>(INPUT_PORT_INDEX_MATRIX))
            {
                auto inputMatrix = (*value)();

                switch (_selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    inputMatrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(_matrix.data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(_matrix.data());
                    lla.x() = deg2rad(lla.x());
                    lla.y() = deg2rad(lla.y());
                    inputMatrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::LLArad_2_ECEF:
                    inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(_matrix.data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                    inputMatrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(_matrix.data()));
                    inputMatrix(0, 0) = rad2deg(inputMatrix(0, 0));
                    inputMatrix(1, 0) = rad2deg(inputMatrix(1, 0));
                    break;
                case Type::n_Quat_b_2_RollPitchYawRad:
                    inputMatrix = trafo::n_Quat_b(_matrix(0, 0), _matrix(1, 0), _matrix(2, 0)).coeffs();
                    break;
                case Type::n_Quat_b_2_RollPitchYawDeg:
                    inputMatrix = trafo::n_Quat_b(deg2rad(_matrix(0, 0)),
                                                  deg2rad(_matrix(1, 0)),
                                                  deg2rad(_matrix(2, 0)))
                                      .coeffs();
                    break;
                case Type::RollPitchYawRad_2_n_Quat_b:
                    inputMatrix = trafo::quat2eulerZYX(Eigen::Quaterniond(_matrix(0, 0), _matrix(1, 0), _matrix(2, 0), _matrix(3, 0)));
                    break;
                case Type::RollPitchYawDeg_2_n_Quat_b:
                    inputMatrix = rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond(_matrix(0, 0),
                                                                                  _matrix(1, 0),
                                                                                  _matrix(2, 0),
                                                                                  _matrix(3, 0))));
                    break;
                }
                notifyInputValueChanged(INPUT_PORT_INDEX_MATRIX);
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

    if (Pin* sourcePin = nm::FindConnectedPinToInputPin(inputPins.at(INPUT_PORT_INDEX_MATRIX).id))
    {
        if (sourcePin->dataIdentifier.front() == "Eigen::MatrixXd")
        {
            if (auto* inputMatrix = getInputValue<Eigen::MatrixXd>(INPUT_PORT_INDEX_MATRIX))
            {
                switch (_selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    _matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix->data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                    _matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix->data()));
                    _matrix(0, 0) = rad2deg(_matrix(0, 0));
                    _matrix(1, 0) = rad2deg(_matrix(1, 0));
                    break;
                case Type::LLArad_2_ECEF:
                    _matrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix->data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(inputMatrix->data());
                    lla.x() = deg2rad(lla.x());
                    lla.y() = deg2rad(lla.y());
                    _matrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::n_Quat_b_2_RollPitchYawRad:
                    _matrix = trafo::quat2eulerZYX(Eigen::Quaterniond((*inputMatrix)(0, 0), (*inputMatrix)(1, 0), (*inputMatrix)(2, 0), (*inputMatrix)(3, 0)));
                    break;
                case Type::n_Quat_b_2_RollPitchYawDeg:
                    _matrix = rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond((*inputMatrix)(0, 0),
                                                                              (*inputMatrix)(1, 0),
                                                                              (*inputMatrix)(2, 0),
                                                                              (*inputMatrix)(3, 0))));
                    break;
                case Type::RollPitchYawRad_2_n_Quat_b:
                    _matrix = trafo::n_Quat_b((*inputMatrix)(0, 0), (*inputMatrix)(1, 0), (*inputMatrix)(2, 0)).coeffs();
                    break;
                case Type::RollPitchYawDeg_2_n_Quat_b:
                    _matrix = trafo::n_Quat_b(deg2rad((*inputMatrix)(0, 0)),
                                              deg2rad((*inputMatrix)(1, 0)),
                                              deg2rad((*inputMatrix)(2, 0)))
                                  .coeffs();
                    break;
                }
                notifyOutputValueChanged(OUTPUT_PORT_INDEX_MATRIX);
            }
        }
        else if (sourcePin->dataIdentifier.front() == "BlockMatrix")
        {
            if (auto* value = getInputValue<BlockMatrix>(INPUT_PORT_INDEX_MATRIX))
            {
                auto inputMatrix = (*value)();

                switch (_selectedTransformation)
                {
                case Type::ECEF_2_LLArad:
                    _matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix.data()));
                    break;
                case Type::ECEF_2_LLAdeg:
                    _matrix = trafo::ecef2lla_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix.data()));
                    _matrix(0, 0) = rad2deg(_matrix(0, 0));
                    _matrix(1, 0) = rad2deg(_matrix(1, 0));
                    break;
                case Type::LLArad_2_ECEF:
                    _matrix = trafo::lla2ecef_WGS84(Eigen::Map<Eigen::Vector3d>(inputMatrix.data()));
                    break;
                case Type::LLAdeg_2_ECEF:
                {
                    Eigen::Vector3d lla = Eigen::Map<Eigen::Vector3d>(inputMatrix.data());
                    lla.x() = deg2rad(lla.x());
                    lla.y() = deg2rad(lla.y());
                    _matrix = trafo::lla2ecef_WGS84(lla);
                    break;
                }
                case Type::n_Quat_b_2_RollPitchYawRad:
                    _matrix = trafo::quat2eulerZYX(Eigen::Quaterniond(inputMatrix(0, 0), inputMatrix(1, 0), inputMatrix(2, 0), inputMatrix(3, 0)));
                    break;
                case Type::n_Quat_b_2_RollPitchYawDeg:
                    _matrix = rad2deg(trafo::quat2eulerZYX(Eigen::Quaterniond(inputMatrix(0, 0),
                                                                              inputMatrix(1, 0),
                                                                              inputMatrix(2, 0),
                                                                              inputMatrix(3, 0))));
                    break;
                case Type::RollPitchYawRad_2_n_Quat_b:
                    _matrix = trafo::n_Quat_b(inputMatrix(0, 0), inputMatrix(1, 0), inputMatrix(2, 0)).coeffs();
                    break;
                case Type::RollPitchYawDeg_2_n_Quat_b:
                    _matrix = trafo::n_Quat_b(deg2rad(inputMatrix(0, 0)),
                                              deg2rad(inputMatrix(1, 0)),
                                              deg2rad(inputMatrix(2, 0)))
                                  .coeffs();
                    break;
                }
                notifyOutputValueChanged(OUTPUT_PORT_INDEX_MATRIX);
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

    switch (_selectedTransformation)
    {
    case Type::ECEF_2_LLArad:
    case Type::ECEF_2_LLAdeg:
    case Type::LLArad_2_ECEF:
    case Type::LLAdeg_2_ECEF:
    case Type::RollPitchYawRad_2_n_Quat_b:
    case Type::RollPitchYawDeg_2_n_Quat_b:
        rows = 3;
        cols = 1;
        break;
    case Type::n_Quat_b_2_RollPitchYawRad:
    case Type::n_Quat_b_2_RollPitchYawDeg:
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
    switch (_selectedTransformation)
    {
    case Type::ECEF_2_LLArad:
    case Type::ECEF_2_LLAdeg:
    case Type::LLArad_2_ECEF:
    case Type::LLAdeg_2_ECEF:
    case Type::n_Quat_b_2_RollPitchYawRad:
    case Type::n_Quat_b_2_RollPitchYawDeg:
        _matrix = Eigen::MatrixXd::Zero(3, 1);
        break;
    case Type::RollPitchYawRad_2_n_Quat_b:
    case Type::RollPitchYawDeg_2_n_Quat_b:
        _matrix = Eigen::MatrixXd::Zero(4, 1);
        break;
    }
}