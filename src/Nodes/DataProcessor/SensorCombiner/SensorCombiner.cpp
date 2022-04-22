#include "SensorCombiner.hpp"

#include "util/Logger.hpp"

#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"

#include <imgui_internal.h>
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::SensorCombiner::SensorCombiner()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 483, 350 }; //TODO: adapt

    updateNumberOfInputPins();

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type() });
}

NAV::SensorCombiner::~SensorCombiner()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SensorCombiner::typeStatic()
{
    return "SensorCombiner";
}

std::string NAV::SensorCombiner::type() const
{
    return typeStatic();
}

std::string NAV::SensorCombiner::category()
{
    return "Data Processor";
}

void NAV::SensorCombiner::guiConfig()
{
    constexpr float configWidth = 380.0F;
    constexpr float unitWidth = 150.0F;

    // TODO: adapt _maxSizeImuObservations to number of sensors?

    if (ImGui::BeginTable(fmt::format("Pin Settings##{}", size_t(id)).c_str(), inputPins.size() > 1 ? 2 : 1,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX, ImVec2(0.0F, 0.0F)))
    {
        ImGui::TableSetupColumn("Pin");
        if (inputPins.size() > 1)
        {
            ImGui::TableSetupColumn("");
        }
        ImGui::TableHeadersRow();

        for (size_t pinIndex = 0; pinIndex < _pinData.size(); ++pinIndex)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); // Pin

            // if (pinIndex == 0 && _dragAndDropPinIndex > 0)
            // {
            //     showDragDropTargetPin(0);
            // }

            bool selectablePinDummy = false;
            ImGui::Selectable(fmt::format("{}##{}", inputPins.at(pinIndex).name, size_t(id)).c_str(), &selectablePinDummy);
            // if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None))
            // {
            //     dragAndDropPinStillInProgress = true;
            //     _dragAndDropPinIndex = static_cast<int>(pinIndex);
            //     // Data is copied into heap inside the drag and drop
            //     ImGui::SetDragDropPayload(fmt::format("DND Pin {}", size_t(id)).c_str(), &pinIndex, sizeof(pinIndex));
            //     ImGui::TextUnformatted(inputPins.at(pinIndex).name.c_str());
            //     ImGui::EndDragDropSource();
            // }
            // if (_dragAndDropPinIndex >= 0
            //     && pinIndex != static_cast<size_t>(_dragAndDropPinIndex - 1)
            //     && pinIndex != static_cast<size_t>(_dragAndDropPinIndex))
            // {
            //     showDragDropTargetPin(pinIndex + 1);
            // }
            // if (ImGui::IsItemHovered())
            // {
            //     ImGui::SetTooltip("This item can be dragged to reorder the pins");
            // }
            if (inputPins.size() > 1)
            {
                ImGui::TableNextColumn(); // Delete
                if (ImGui::Button(fmt::format("x##{} - {}", size_t(id), pinIndex).c_str()))
                {
                    nm::DeleteInputPin(inputPins.at(pinIndex).id);
                    _pinData.erase(_pinData.begin() + static_cast<int64_t>(pinIndex));
                    --_nInputPins;
                    flow::ApplyChanges();
                    // TODO: updateNumberOfInputPins(); ???
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Delete the pin");
                }
            }
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); // Pin
        if (ImGui::Button(fmt::format("Add Pin##{}", size_t(id)).c_str()))
        {
            ++_nInputPins;
            LOG_DEBUG("{}: # Input Pins changed to {}", nameId(), _nInputPins);
            flow::ApplyChanges();
            updateNumberOfInputPins();
        }

        ImGui::EndTable();
    }

    ImGui::Separator();

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);

    constexpr float columnWidth{ 130.0F };

    ImGui::SetNextItemWidth(columnWidth);
    if (ImGui::InputDoubleL(fmt::format("Highest IMU sample rate##{}", size_t(id)).c_str(), &_imuFrequency, 1e-3, 1e4, 0.0, 0.0, "%.0f Hz"))
    {
        LOG_DEBUG("{}: imuFrequency changed to {}", nameId(), _imuFrequency);
        flow::ApplyChanges();
    }

    ImGui::Separator();

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("P Error covariance matrix (init)##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular rate covariance ({})##{}",
                                                           _initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::rad2_s2
                                                                   || _initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAngularRate.data(), reinterpret_cast<int*>(&_initCovarianceAngularRateUnit), "(rad/s)^2\0"
                                                                                                                                                                   "rad/s\0"
                                                                                                                                                                   "(deg/s)^2\0"
                                                                                                                                                                   "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAngularRate changed to {}", nameId(), _initCovarianceAngularRate);
            LOG_DEBUG("{}: InitCovarianceAngularRateUnit changed to {}", nameId(), _initCovarianceAngularRateUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular acceleration covariance ({})##{}",
                                                           _initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::rad2_s4
                                                                   || _initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::deg2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAngularAcc.data(), reinterpret_cast<int*>(&_initCovarianceAngularAccUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                 "rad/s^2\0"
                                                                                                                                                                 "(deg^2)/(s^4)\0"
                                                                                                                                                                 "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAngularAcc changed to {}", nameId(), _initCovarianceAngularAcc);
            LOG_DEBUG("{}: InitCovarianceAngularAccUnit changed to {}", nameId(), _initCovarianceAngularAccUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Acceleration covariance ({})##{}",
                                                           _initCovarianceAccelerationUnit == InitCovarianceAccelerationUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAcceleration.data(), reinterpret_cast<int*>(&_initCovarianceAccelerationUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                     "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAcceleration changed to {}", nameId(), _initCovarianceAcceleration);
            LOG_DEBUG("{}: InitCovarianceAccelerationUnit changed to {}", nameId(), _initCovarianceAccelerationUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk covariance ({})##{}",
                                                           _initCovarianceJerkUnit == InitCovarianceJerkUnit::m2_s6
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceJerk.data(), reinterpret_cast<int*>(&_initCovarianceJerkUnit), "(m^2)/(s^6)\0"
                                                                                                                                                     "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceJerk changed to {}", nameId(), _initCovarianceJerk);
            LOG_DEBUG("{}: InitCovarianceJerkUnit changed to {}", nameId(), _initCovarianceJerkUnit);
            flow::ApplyChanges();
        }

        // TODO: Make for-loop around 'Angular acceleration bias covariance' and 'Jerk bias covariance' to add as many inputs as there are measurements
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Angular acceleration bias covariance ({})##{}",
                                                           _initCovarianceBiasAngRateUnit == InitCovarianceBiasAngRateUnit::rad2_s2
                                                                   || _initCovarianceBiasAngRateUnit == InitCovarianceBiasAngRateUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasAngRate.data(), reinterpret_cast<int*>(&_initCovarianceBiasAngRateUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                   "rad/s^2\0"
                                                                                                                                                                   "(deg^2)/(s^4)\0"
                                                                                                                                                                   "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasAngAcc changed to {}", nameId(), _initCovarianceBiasAngRate);
            LOG_DEBUG("{}: InitCovarianceBiasAngRateUnit changed to {}", nameId(), _initCovarianceBiasAngRateUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk bias covariance ({})##{}",
                                                           _initCovarianceBiasAccUnit == InitCovarianceBiasAccUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasAcc.data(), reinterpret_cast<int*>(&_initCovarianceBiasAccUnit), "(m^2)/(s^6)\0"
                                                                                                                                                           "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasJerk changed to {}", nameId(), _initCovarianceBiasAcc);
            LOG_DEBUG("{}: InitCovarianceBiasAccUnit changed to {}", nameId(), _initCovarianceBiasAccUnit);
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Q - System/Process noise covariance matrix##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);

        // TODO: Make for-loop around 'angular acceleration process noise' and 'Jerk bias covariance' to add as many inputs as there are measurements
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the process noise on the angular acceleration##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _varAngularAccNoise.data(), reinterpret_cast<int*>(&_varAngularAccNoiseUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                     "rad/s^2\0"
                                                                                                                                                     "(deg^2)/(s^4)\0"
                                                                                                                                                     "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varAngularAccNoise changed to {}", nameId(), _varAngularAccNoise.transpose());
            LOG_DEBUG("{}: varAngularAccNoiseUnit changed to {}", nameId(), _varAngularAccNoiseUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the process noise on the jerk##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _varJerkNoise.data(), reinterpret_cast<int*>(&_varJerkNoiseUnit), "(m^2)/(s^6)\0"
                                                                                                                                         "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varJerkNoise changed to {}", nameId(), _varJerkNoise.transpose());
            LOG_DEBUG("{}: varJerkNoiseUnit changed to {}", nameId(), _varJerkNoiseUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the process noise on the bias of the angular rate##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _varBiasAngRateNoise.data(), reinterpret_cast<int*>(&_varBiasAngRateNoiseUnit), "(rad/s)^2\0"
                                                                                                                                                       "rad/s\0"
                                                                                                                                                       "(deg/s)^2\0"
                                                                                                                                                       "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varBiasAngRateNoise changed to {}", nameId(), _varBiasAngRateNoise.transpose());
            LOG_DEBUG("{}: varBiasAngRateNoiseUnit changed to {}", nameId(), _varBiasAngRateNoiseUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the process noise on the bias of the acceleration##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _varBiasAccelerationNoise.data(), reinterpret_cast<int*>(&_varBiasAccelerationNoiseUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                 "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: varBiasAccelerationNoise changed to {}", nameId(), _varBiasAccelerationNoise.transpose());
            LOG_DEBUG("{}: varBiasAccelerationNoiseUnit changed to {}", nameId(), _varBiasAccelerationNoiseUnit);
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);

        // TODO: Make for-loop around 'angular rate measurement uncertainty' and 'acceleration measurement uncertainty' to add as many inputs as there are measurements
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the measurement uncertainty on the angular rate##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _measurementUncertaintyAngularRate.data(), reinterpret_cast<int*>(&_measurementUncertaintyAngularRateUnit), "(rad/s)^2\0"
                                                                                                                                                                                   "rad/s\0"
                                                                                                                                                                                   "(deg/s)^2\0"
                                                                                                                                                                                   "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdevAngularAcc changed to {}", nameId(), _measurementUncertaintyAngularRate.transpose());
            LOG_DEBUG("{}: stdevAngularAccUnit changed to {}", nameId(), _measurementUncertaintyAngularRateUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the measurement uncertainty on the acceleration##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _measurementUncertaintyAcceleration.data(), reinterpret_cast<int*>(&_measurementUncertaintyAccelerationUnit), "(m^2)/(s^4)\0"
                                                                                                                                                                                     "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdevJerk changed to {}", nameId(), _measurementUncertaintyAcceleration.transpose());
            LOG_DEBUG("{}: stdevJerkUnit changed to {}", nameId(), _measurementUncertaintyAccelerationUnit);
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::SensorCombiner::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    // TODO

    return j;
}

void NAV::SensorCombiner::restore([[maybe_unused]] json const& j) //TODO: remove [[maybe_unused]]
{
    LOG_TRACE("{}: called", nameId());

    // TODO
}

bool NAV::SensorCombiner::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _kalmanFilter.setZero();
    _imuObservations.clear();

    updateNumberOfInputPins();

    LOG_DEBUG("SensorCombiner initialized");

    return true;
}

void NAV::SensorCombiner::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::SensorCombiner::updateNumberOfInputPins()
{
    while (inputPins.size() < _nInputPins)
    {
        nm::CreateInputPin(this, fmt::format("Pin {}", inputPins.size() + 1).c_str(), Pin::Type::Flow,
                           { NAV::ImuObs::type() }, &SensorCombiner::recvSignal);
        _pinData.emplace_back();
    }
    while (inputPins.size() > _nInputPins)
    {
        nm::DeleteInputPin(inputPins.back().id);
        _pinData.pop_back();
    }

    // Number of sensors
    auto M = static_cast<uint8_t>(_nInputPins);

    // Number of estimated states (accel and gyro)
    uint8_t numStatesEst = 6;

    // Number of states per pin (biases of accel and gyro)
    uint8_t numStatesPerPin = 6;

    // Number of measurements per pin (acceleration and angular rate)
    uint8_t numMeasPerPin = 6;

    uint8_t numStates = numStatesEst + static_cast<uint8_t>(M * numStatesPerPin);
    uint8_t numMeasurements = numMeasPerPin * M;

    // Initial time step for KF prediction
    double dt = 1.0 / _imuFrequency;

    Eigen::Matrix3d DCM{};

    // ------------------------------------------------------ Error covariance matrix P --------------------------------------------------------

    // Initial Covariance of the angular rate in [rad²/s²]
    Eigen::Vector3d variance_angularRate = Eigen::Vector3d::Zero();
    if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::rad2_s2)
    {
        variance_angularRate = _initCovarianceAngularRate;
    }
    else if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::deg2_s2)
    {
        variance_angularRate = trafo::deg2rad(_initCovarianceAngularRate);
    }
    else if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::rad_s)
    {
        variance_angularRate = _initCovarianceAngularRate.array().pow(2);
    }
    else if (_initCovarianceAngularRateUnit == InitCovarianceAngularRateUnit::deg_s)
    {
        variance_angularRate = trafo::deg2rad(_initCovarianceAngularRate).array().pow(2);
    }

    // Initial Covariance of the angular acceleration in [(rad^2)/(s^4)]
    Eigen::Vector3d variance_angularAcceleration = Eigen::Vector3d::Zero();
    if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::rad2_s4)
    {
        variance_angularAcceleration = _initCovarianceAngularAcc;
    }
    else if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::deg2_s4)
    {
        variance_angularAcceleration = trafo::deg2rad(_initCovarianceAngularAcc);
    }
    else if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::rad_s2)
    {
        variance_angularAcceleration = _initCovarianceAngularAcc.array().pow(2);
    }
    else if (_initCovarianceAngularAccUnit == InitCovarianceAngularAccUnit::deg_s2)
    {
        variance_angularAcceleration = trafo::deg2rad(_initCovarianceAngularAcc).array().pow(2);
    }

    // Initial Covariance of the acceleration in [(m^2)/(s^4)]
    Eigen::Vector3d variance_acceleration = Eigen::Vector3d::Zero();
    if (_initCovarianceAccelerationUnit == InitCovarianceAccelerationUnit::m2_s4)
    {
        variance_acceleration = _initCovarianceAcceleration;
    }
    else if (_initCovarianceAccelerationUnit == InitCovarianceAccelerationUnit::m_s2)
    {
        variance_acceleration = _initCovarianceAcceleration.array().pow(2);
    }

    // Initial Covariance of the jerk in [(m^2)/(s^6)]
    Eigen::Vector3d variance_jerk = Eigen::Vector3d::Zero();
    if (_initCovarianceJerkUnit == InitCovarianceJerkUnit::m2_s6)
    {
        variance_jerk = _initCovarianceJerk;
    }
    else if (_initCovarianceJerkUnit == InitCovarianceJerkUnit::m_s3)
    {
        variance_jerk = _initCovarianceJerk.array().pow(2);
    }

    // TODO: Make for-loop around 'bias of the angular acceleration' and 'bias of the jerk' to add as many inputs as there are measurements
    // Initial Covariance of the bias of the angular acceleration in [(rad^2)/(s^4)]
    Eigen::Vector3d variance_biasAngularAcceleration = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasAngRateUnit == InitCovarianceBiasAngRateUnit::rad2_s2)
    {
        variance_biasAngularAcceleration = _initCovarianceBiasAngRate;
    }
    else if (_initCovarianceBiasAngRateUnit == InitCovarianceBiasAngRateUnit::deg2_s2)
    {
        variance_biasAngularAcceleration = trafo::deg2rad(_initCovarianceBiasAngRate);
    }
    else if (_initCovarianceBiasAngRateUnit == InitCovarianceBiasAngRateUnit::rad_s)
    {
        variance_biasAngularAcceleration = _initCovarianceBiasAngRate.array().pow(2);
    }
    else if (_initCovarianceBiasAngRateUnit == InitCovarianceBiasAngRateUnit::deg_s)
    {
        variance_biasAngularAcceleration = trafo::deg2rad(_initCovarianceBiasAngRate).array().pow(2);
    }

    // Initial Covariance of the bias of the jerk in [(m^2)/(s^6)]
    Eigen::Vector3d variance_biasJerk = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasAccUnit == InitCovarianceBiasAccUnit::m2_s4)
    {
        variance_biasJerk = _initCovarianceBiasAcc;
    }
    else if (_initCovarianceBiasAccUnit == InitCovarianceBiasAccUnit::m_s2)
    {
        variance_biasJerk = _initCovarianceBiasAcc.array().pow(2);
    }

    // ------------------------------------------------------- Process noise matrix Q ----------------------------------------------------------

    // 𝜎_AngAcc Standard deviation of the noise on the angular acceleration state [rad/s²]
    Eigen::Vector3d variance_AngAccNoise = Eigen::Vector3d::Zero();
    switch (_varAngularAccNoiseUnit)
    {
    case VarAngularAccNoiseUnit::rad2_s4:
        variance_AngAccNoise = _varAngularAccNoise;
        break;
    case VarAngularAccNoiseUnit::deg2_s4:
        variance_AngAccNoise = trafo::deg2rad(_varAngularAccNoise);
        break;
    case VarAngularAccNoiseUnit::deg_s2:
        variance_AngAccNoise = trafo::deg2rad(_varAngularAccNoise).array().pow(2);
        break;
    case VarAngularAccNoiseUnit::rad_s2:
        variance_AngAccNoise = _varAngularAccNoise.array().pow(2);
        break;
    }

    // 𝜎_jerk Standard deviation of the noise on the jerk state [m/s³]
    Eigen::Vector3d variance_jerkNoise = Eigen::Vector3d::Zero();
    switch (_varJerkNoiseUnit)
    {
    case VarJerkNoiseUnit::m2_s6:
        variance_jerkNoise = _varJerkNoise;
        break;
    case VarJerkNoiseUnit::m_s3:
        variance_jerkNoise = _varJerkNoise.array().pow(2);
        break;
    }

    // 𝜎_biasAngRate Standard deviation of the bias on the angular rate state [rad/s²]
    Eigen::Vector3d variance_biasAngRate = Eigen::Vector3d::Zero();
    switch (_varBiasAngRateNoiseUnit)
    {
    case VarBiasAngRateNoiseUnit::rad2_s2:
        variance_biasAngRate = _varBiasAngRateNoise;
        break;
    case VarBiasAngRateNoiseUnit::deg2_s2:
        variance_biasAngRate = trafo::deg2rad(_varBiasAngRateNoise);
        break;
    case VarBiasAngRateNoiseUnit::deg_s:
        variance_biasAngRate = trafo::deg2rad(_varBiasAngRateNoise).array().pow(2);
        break;
    case VarBiasAngRateNoiseUnit::rad_s:
        variance_biasAngRate = _varBiasAngRateNoise.array().pow(2);
        break;
    }

    // 𝜎_biasAcceleration Standard deviation of the noise on the acceleration state [m/s³]
    Eigen::Vector3d variance_biasAcceleration = Eigen::Vector3d::Zero();
    switch (_varBiasAccelerationNoiseUnit)
    {
    case VarBiasAccelerationNoiseUnit::m2_s4:
        variance_biasAcceleration = _varBiasAccelerationNoise;
        break;
    case VarBiasAccelerationNoiseUnit::m_s2:
        variance_biasAcceleration = _varBiasAccelerationNoise.array().pow(2);
        break;
    }

    // -------------------------------------------------- Measurement uncertainty matrix R -----------------------------------------------------

    // Measurement uncertainty for the angular rate (Variance σ²) in [(rad/s)^2, (rad/s)^2, (rad/s)^2]
    Eigen::Vector3d sigmaSquaredAngularRateMeas = Eigen::Vector3d::Zero();
    switch (_measurementUncertaintyAngularRateUnit)
    {
    case MeasurementUncertaintyAngularRateUnit::rad_s:
        sigmaSquaredAngularRateMeas = (_measurementUncertaintyAngularRate).array().pow(2);
        break;
    case MeasurementUncertaintyAngularRateUnit::deg_s:
        sigmaSquaredAngularRateMeas = (trafo::deg2rad(_measurementUncertaintyAngularRate)).array().pow(2);
        break;
    case MeasurementUncertaintyAngularRateUnit::rad2_s2:
        sigmaSquaredAngularRateMeas = _measurementUncertaintyAngularRate;
        break;
    case MeasurementUncertaintyAngularRateUnit::deg2_s2:
        sigmaSquaredAngularRateMeas = trafo::deg2rad((_measurementUncertaintyAngularRate).cwiseSqrt()).array().pow(2);
        break;
    }

    // Measurement uncertainty for the acceleration (Variance σ²) in [(m^2)/(s^4), (m^2)/(s^4), (m^2)/(s^4)]
    Eigen::Vector3d sigmaSquaredAccelerationMeas = Eigen::Vector3d::Zero();
    switch (_measurementUncertaintyAccelerationUnit)
    {
    case MeasurementUncertaintyAccelerationUnit::m2_s4:
        sigmaSquaredAccelerationMeas = _measurementUncertaintyAcceleration;
        break;
    case MeasurementUncertaintyAccelerationUnit::m_s2:
        sigmaSquaredAccelerationMeas = (_measurementUncertaintyAcceleration).array().pow(2);
        break;
    }

    // --------------------------------------------------------- KF Initializations ------------------------------------------------------------
    KalmanFilter _kalmanFilter{ numStates, numMeasurements };
    // _kalmanFilter.P = initialErrorCovarianceMatrix_P0(varOmega, varAlpha, varAcc, varJerk, varBiasAlpha, varBiasAcc); // numStates not necessary as an argument
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(numStates, variance_angularRate, variance_angularAcceleration, variance_acceleration, variance_jerk, variance_biasAngularAcceleration, variance_biasJerk);
    _kalmanFilter.Phi = stateTransitionMatrix_Phi(numStates, dt);
    _kalmanFilter.Q = processNoiseMatrix_Q(numStates, dt, variance_AngAccNoise, variance_jerkNoise, variance_biasAngRate, variance_biasAcceleration);
    _kalmanFilter.H = designMatrix_H(numStates, numMeasurements, DCM);
    _kalmanFilter.R = measurementNoiseMatrix_R_init(numMeasurements, sigmaSquaredAngularRateMeas, sigmaSquaredAccelerationMeas);
}

void NAV::SensorCombiner::recvSignal(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    auto imuObs = std::static_pointer_cast<const ImuObs>(nodeData);

    if (!imuObs->insTime.has_value() && !imuObs->timeSinceStartup.has_value())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime/timeSinceStartup)", nameId());
        return;
    }

    // Add imuObs tₖ to the start of the list // TODO: possibly needs info from which sensor the data is coming from
    _imuObservations.push_front(imuObs);

    // Remove observations at the end of the list till the max size is reached
    while (_imuObservations.size() > _maxSizeImuObservations)
    {
        LOG_WARN("{}: Receive new Imu observation, but list is full --> discarding oldest observation", nameId());
        _imuObservations.pop_back();
    }

    // First ImuObs
    if (_imuObservations.size() == 1)
    {
        // Push out a message with the initial state and a matching imu Observation
        auto ImuObsOut = std::make_shared<ImuObs>(_imuPos);

        ImuObsOut->insTime = imuObs->insTime;
        // ImuObsOut->imuObs = imuObs;

        invokeCallbacks(OUTPUT_PORT_INDEX_COMBINED_SIGNAL, ImuObsOut);
        return;
    }

    // If enough imu observations, combine the signals
    if (_imuObservations.size() == _maxSizeImuObservations)
    {
        combineSignals();
    }
}

void NAV::SensorCombiner::combineSignals()
{
    LOG_TRACE("{}: called", nameId());
    // -------------------------------------------------- Construct the message to send out ----------------------------------------------------

    // for (size_t pinIndex = 0; pinIndex < _pinData.size(); ++pinIndex)
    // {
    //     auto& pinData = _pinData.at(pinIndex);    // IMU Observation of sensor #1
    //     const std::shared_ptr<const ImuObs>& imuObs__s0 = _imuObservations.at(0);
    //     // IMU Observation at the time tₖ₋₁
    //     const std::shared_ptr<const ImuObs>& imuObs__t1 = _imuObservations.at(1);
    // }

    // _kalmanFilter.predict();

    // _kalmanFilter.correct();

    auto obs = _imuObservations.front(); // TODO: this should be the combined IMU observation
    invokeCallbacks(OUTPUT_PORT_INDEX_COMBINED_SIGNAL, obs);
}

const Eigen::MatrixXd NAV::SensorCombiner::stateTransitionMatrix_Phi(uint8_t numStates, double dt) //NOLINT(readability-const-return-type,readability-make-member-function-const,readability-convert-member-functions-to-static)
{
    Eigen::MatrixXd Phi(numStates, numStates);

    Phi.diagonal().setOnes(); // constant part of states

    Phi.block<3, 3>(0, 3).diagonal().setConstant(dt); // dependency of angular rate on angular acceleration
    Phi.block<3, 3>(6, 9).diagonal().setConstant(dt); // dependency of acceleration on jerk

    return Phi;
}

Eigen::MatrixXd NAV::SensorCombiner::initialErrorCovarianceMatrix_P0(uint8_t numStates,
                                                                     Eigen::Vector3d& varAngRate,
                                                                     Eigen::Vector3d& varAngAcc,
                                                                     Eigen::Vector3d& varAcc,
                                                                     Eigen::Vector3d& varJerk,
                                                                     Eigen::Vector3d& varBiasAngRate,
                                                                     Eigen::Vector3d& varBiasAcc)
{
    Eigen::MatrixXd P(numStates, numStates);

    P.block<3, 3>(0, 0).diagonal() = varAngRate;
    P.block<3, 3>(3, 3).diagonal() = varAngAcc;
    P.block<3, 3>(6, 6).diagonal() = varAcc;
    P.block<3, 3>(9, 9).diagonal() = varJerk;

    for (uint8_t i = 12; i < numStates; i += 6)
    {
        P.block<3, 3>(i, i).diagonal() = varBiasAngRate;
        P.block<3, 3>(i + 3, i + 3).diagonal() = varBiasAcc;
    }

    return P;
}

Eigen::MatrixXd NAV::SensorCombiner::processNoiseMatrix_Q(uint8_t numStates,
                                                          double dt,
                                                          Eigen::Vector3d& varAngAcc,
                                                          Eigen::Vector3d& varJerk,
                                                          Eigen::Vector3d& varBiasAngAcc,
                                                          Eigen::Vector3d& varBiasJerk)
{
    Eigen::MatrixXd Q(numStates, numStates);

    // Integrated Random Walk of the angular rate
    Q.block<3, 3>(0, 0).diagonal() = varAngAcc / 3. * std::pow(dt, 3);
    Q.block<3, 3>(0, 3).diagonal() = varAngAcc / 2. * std::pow(dt, 2);
    Q.block<3, 3>(3, 0).diagonal() = varAngAcc / 2. * std::pow(dt, 2);
    Q.block<3, 3>(3, 3).diagonal() = varAngAcc * dt;

    // Integrated Random Walk of the acceleration
    Q.block<3, 3>(6, 6).diagonal() = varJerk / 3. * std::pow(dt, 3);
    Q.block<3, 3>(6, 9).diagonal() = varJerk / 2. * std::pow(dt, 2);
    Q.block<3, 3>(9, 6).diagonal() = varJerk / 2. * std::pow(dt, 2);
    Q.block<3, 3>(9, 9).diagonal() = varJerk * dt;

    // Random Walk of the bias states
    for (uint8_t i = 12; i < numStates; i += 6)
    {
        Q.block<3, 3>(i, i).diagonal() = varBiasAngAcc * dt;
        Q.block<3, 3>(i + 3, i + 3).diagonal() = varBiasJerk * dt;
    }

    return Q;
}

const Eigen::MatrixXd NAV::SensorCombiner::designMatrix_H(uint8_t numStates, uint8_t numMeasurements, Eigen::Matrix<double, 3, 3>& DCM) //NOLINT(readability-const-return-type,readability-make-member-function-const,readability-convert-member-functions-to-static)
{
    Eigen::MatrixXd H(numMeasurements, numStates);
    H.setZero();

    // Mapping of state estimates on sensors
    for (uint8_t i = 0; i < numMeasurements; i += 6)
    {
        H.block<3, 3>(i, 0) = DCM;     // Rotation for angular rate
        H.block<3, 3>(i + 3, 6) = DCM; // Rotation for acceleration

        if (i >= 6)
        {
            H.block<6, 6>(i, i + 6) = Eigen::MatrixXd::Identity(6, 6); // constant bias of each sensor (3 angular rates and 3 accelerations)
        }
    }

    return H;
}

Eigen::MatrixXd NAV::SensorCombiner::measurementNoiseMatrix_R(double alpha, Eigen::MatrixXd& R, Eigen::VectorXd& e, Eigen::MatrixXd& H, Eigen::MatrixXd& P)
{
    return alpha * R + (1.0 - alpha) * (e * e.transpose() + H * P * H.transpose());
}

Eigen::MatrixXd NAV::SensorCombiner::measurementNoiseMatrix_R_init(uint8_t numMeasurements, Eigen::Vector3d& varAngRateMeas, Eigen::Vector3d& varAccelerationMeas)
{
    Eigen::MatrixXd R(numMeasurements, numMeasurements);
    R.setZero();

    for (uint8_t i = 0; i < numMeasurements; i += 6)
    {
        R.block<3, 3>(i, i).diagonal() = varAngRateMeas;
        R.block<3, 3>(i + 3, i + 3).diagonal() = varAccelerationMeas;
    }

    return R;
}