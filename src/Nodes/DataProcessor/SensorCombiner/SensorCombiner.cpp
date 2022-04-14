#include "SensorCombiner.hpp"

#include "util/Logger.hpp"

#include "Navigation/Math/Math.hpp"

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
                                                           _initCovarianceBiasAngAccUnit == InitCovarianceBiasAngAccUnit::rad2_s4
                                                                   || _initCovarianceBiasAngAccUnit == InitCovarianceBiasAngAccUnit::deg2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasAngAcc.data(), reinterpret_cast<int*>(&_initCovarianceBiasAngAccUnit), "(rad^2)/(s^4)\0"
                                                                                                                                                                 "rad/s^2\0"
                                                                                                                                                                 "(deg^2)/(s^4)\0"
                                                                                                                                                                 "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasAngAcc changed to {}", nameId(), _initCovarianceBiasAngAcc);
            LOG_DEBUG("{}: InitCovarianceBiasAngAccUnit changed to {}", nameId(), _initCovarianceBiasAngAccUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Jerk bias covariance ({})##{}",
                                                           _initCovarianceBiasJerkUnit == InitCovarianceBiasJerkUnit::m2_s6
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasJerk.data(), reinterpret_cast<int*>(&_initCovarianceBiasJerkUnit), "(m^2)/(s^6)\0"
                                                                                                                                                             "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasJerk changed to {}", nameId(), _initCovarianceBiasJerk);
            LOG_DEBUG("{}: InitCovarianceBiasJerkUnit changed to {}", nameId(), _initCovarianceBiasJerkUnit);
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
                                               configWidth, unitWidth, _stdevAngularAcc.data(), reinterpret_cast<int*>(&_stdevAngularAccUnit), "rad/s^2\0"
                                                                                                                                               "deg/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdevAngularAcc changed to {}", nameId(), _stdevAngularAcc.transpose());
            LOG_DEBUG("{}: stdevAngularAccUnit changed to {}", nameId(), _stdevAngularAccUnit);
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the process noise on the jerk##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _stdevJerk.data(), reinterpret_cast<int*>(&_stdevJerkUnit), "m/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdevJerk changed to {}", nameId(), _stdevJerk.transpose());
            LOG_DEBUG("{}: stdevJerkUnit changed to {}", nameId(), _stdevJerkUnit);
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

    _imuObservations.clear();
    // _kalmanFilter.setZero();
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

    _numStates = numStatesEst + static_cast<uint8_t>(M * numStatesPerPin);
    _numMeasurements = 6 * M;

    double dt{};
    double sigma_w{};
    double sigma_f{};
    double sigma_biasw{};
    double sigma_biasf{};
    Eigen::Matrix3d DCM{};

    // KF initializations
    KalmanFilter _kalmanFilter{ _numStates, _numMeasurements };
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(M);
    _kalmanFilter.Phi = stateTransitionMatrix_Phi();
    _kalmanFilter.Q = processNoiseMatrix_Q(dt, sigma_w, sigma_f, sigma_biasw, sigma_biasf, M);
    _kalmanFilter.H = designMatrix_H(DCM);
    _kalmanFilter.R = measurementNoiseMatrix_R_init(sigma_w, sigma_f, M);
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

const Eigen::MatrixXd NAV::SensorCombiner::stateTransitionMatrix_Phi() //NOLINT(readability-const-return-type,readability-make-member-function-const)
{
    Eigen::MatrixXd Phi(_numStates, _numStates);
    for (uint8_t i = 0; i < _numStates; ++i)
    {
        Phi(i, i) = 1;
    }

    return Phi;
}

Eigen::MatrixXd NAV::SensorCombiner::processNoiseMatrix_Q(double dt,
                                                          double sigma_w,
                                                          double sigma_f,
                                                          double sigma_biasw,
                                                          double sigma_biasf,
                                                          uint8_t M)
{
    auto numStates = static_cast<uint8_t>(6 + 2 * 3 * M); // dim(accelXYZ)=3, dim(gyroXYZ)=3, dim(sensorBiases)=2*3*M --> accel and gyro biases
    Eigen::MatrixXd Q(numStates, numStates);
    for (uint8_t i = 0; i < numStates; ++i)
    {
        Q(i, i) = 0;
    }

    // Process noise of angular rate and specific force - Random Walk
    Q.block<3, 3>(0, 0) = dt * std::pow(sigma_w, 2) * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(3, 3) = dt * std::pow(sigma_f, 2) * Eigen::Matrix3d::Identity();

    // Process noise of sensor biases - Random Walk
    for (uint8_t i = 6; i < numStates; i += 6)
    {
        Q.block<3, 3>(i, i) = dt * std::pow(sigma_biasw, 2) * Eigen::Matrix3d::Identity();
        Q.block<3, 3>(3 + i, 3 + i) = dt * std::pow(sigma_biasf, 2) * Eigen::Matrix3d::Identity();
    }

    return Q;
}

const Eigen::MatrixXd NAV::SensorCombiner::designMatrix_H(Eigen::Matrix<double, 3, 3>& DCM) //NOLINT(readability-const-return-type,readability-make-member-function-const)
{
    Eigen::MatrixXd H(_numStates, _numMeasurements);
    H = Eigen::MatrixXd::Zero(_numStates, _numMeasurements);

    // Mapping of state estimates on sensors
    for (uint8_t i = 0; i < _numStates; i += 6)
    {
        H.block<3, 3>(i, 0) = DCM;
        H.block<3, 3>(i + 3, 3) = DCM;
    }

    return H;
}

Eigen::MatrixXd NAV::SensorCombiner::measurementNoiseMatrix_R(double alpha, Eigen::MatrixXd& R, Eigen::VectorXd& e, Eigen::Matrix<double, Eigen::Dynamic, 9>& H, Eigen::Matrix<double, 9, 9>& P)
{
    return alpha * R + (1.0 - alpha) * (e * e.transpose() + H * P * H.transpose());
}

Eigen::MatrixXd NAV::SensorCombiner::measurementNoiseMatrix_R_init(double sigma_w, double sigma_f, uint8_t M)
{
    auto numMeas = static_cast<uint8_t>(2 * 3 * M); // dim(accelXYZ)=3, dim(gyroXYZ)=3, dim(sensorBiases)=2*3*M --> accel and gyro biases
    Eigen::MatrixXd R(numMeas, numMeas);
    R.setZero();
    for (uint8_t i = 0; i < 3 * M; ++i)
    {
        R(i, i) = std::pow(sigma_w, 2);
        R(3 * M + i, 3 * M + i) = std::pow(sigma_f, 2);
    }

    return R;
}

Eigen::MatrixXd NAV::SensorCombiner::initialErrorCovarianceMatrix_P0(uint8_t M)
{
    auto numStates = static_cast<uint8_t>(6 + 2 * 3 * M); // dim(accelXYZ)=3, dim(gyroXYZ)=3, dim(sensorBiases)=2*3*M --> accel and gyro biases
    Eigen::MatrixXd P(numStates, numStates);

    for (uint8_t i = 0; i < numStates; ++i)
    {
        P(i, i) = 1; // no covariance at the beginning // TODO: Extend by a factor of GUI inputs of 2x3D variances per one sensor
    }

    return P;
}