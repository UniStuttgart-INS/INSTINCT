#include "ErrorModel.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "util/Eigen.hpp"

#include <imgui_internal.h>
#include <limits>

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{ ImuObs::type(), PosVelAtt::type() };

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] rng Object to read info from
void to_json(json& j, const ErrorModel::RandomNumberGenerator& rng)
{
    j = json{
        { "useSeedInsteadOfSystemTime", rng.useSeedInsteadOfSystemTime },
        { "seed", rng.seed },
    };
}
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] rng Output object
void from_json(const json& j, ErrorModel::RandomNumberGenerator& rng)
{
    if (j.contains("useSeedInsteadOfSystemTime"))
    {
        j.at("useSeedInsteadOfSystemTime").get_to(rng.useSeedInsteadOfSystemTime);
    }
    if (j.contains("seed"))
    {
        j.at("seed").get_to(rng.seed);
    }
}

} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::ErrorModel::ErrorModel()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 812, 332 };

    nm::CreateInputPin(this, "True", Pin::Type::Flow, supportedDataIdentifier, &ErrorModel::receiveObs);

    nm::CreateOutputPin(this, "Biased", Pin::Type::Flow, supportedDataIdentifier);
}

NAV::ErrorModel::~ErrorModel()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ErrorModel::typeStatic()
{
    return "ErrorModel";
}

std::string NAV::ErrorModel::type() const
{
    return typeStatic();
}

std::string NAV::ErrorModel::category()
{
    return "DataProcessor";
}

void NAV::ErrorModel::guiConfig()
{
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.size() != 1)
    {
        ImGui::TextUnformatted("Please connect the input pin to show the options");
        return;
    }

    float itemWidth = 470;
    float unitWidth = 180;

    ImGui::TextUnformatted("Offsets:");
    ImGui::Indent();
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer Bias (platform)##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                               _imuAccelerometerBias_p.data(), reinterpret_cast<int*>(&_imuAccelerometerBiasUnit), "m/s^2\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _imuAccelerometerBias_p changed to {}", nameId(), _imuAccelerometerBias_p.transpose());
            LOG_DEBUG("{}: _imuAccelerometerBiasUnit changed to {}", nameId(), _imuAccelerometerBiasUnit);
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Gyroscope Bias (platform)##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                               _imuGyroscopeBias_p.data(), reinterpret_cast<int*>(&_imuGyroscopeBiasUnit), "rad/s\0deg/s\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _imuGyroscopeBias_p changed to {}", nameId(), _imuGyroscopeBias_p.transpose());
            LOG_DEBUG("{}: _imuGyroscopeBiasUnit changed to {}", nameId(), _imuGyroscopeBiasUnit);
            flow::ApplyChanges();
        }
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Position Bias ({})##{}",
                                                           _positionBiasUnit == PositionBiasUnits::meter ? "NED" : "LatLonAlt",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _positionBias.data(), reinterpret_cast<int*>(&_positionBiasUnit), "[m m m]\0[rad, rad, m]\0[deg, deg, m]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _positionBias changed to {}", nameId(), _positionBias.transpose());
            LOG_DEBUG("{}: _positionBiasUnit changed to {}", nameId(), _positionBiasUnit);
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity Bias (NED)##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                               _velocityBias.data(), reinterpret_cast<int*>(&_velocityBiasUnit), "m/s\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _velocityBias changed to {}", nameId(), _velocityBias.transpose());
            LOG_DEBUG("{}: _velocityBiasUnit changed to {}", nameId(), _velocityBiasUnit);
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDouble3WithUnit(fmt::format("RollPitchYaw Bias##{}", size_t(id)).c_str(), itemWidth, unitWidth,
                                               _attitudeBias.data(), reinterpret_cast<int*>(&_attitudeBiasUnit), "rad\0deg\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _attitudeBias changed to {}", nameId(), _attitudeBias.transpose());
            LOG_DEBUG("{}: _attitudeBiasUnit changed to {}", nameId(), _attitudeBiasUnit);
            flow::ApplyChanges();
        }
    }
    ImGui::Unindent();

    ImGui::TextUnformatted("Measurement noise:");
    ImGui::Indent();
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        // #########################################################################################################################################
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer Noise ({})##{}",
                                                           _imuAccelerometerNoiseUnit == ImuAccelerometerNoiseUnits::m_s2 ? "Standard deviation"
                                                                                                                          : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _imuAccelerometerNoise.data(), reinterpret_cast<int*>(&_imuAccelerometerNoiseUnit), "m/s^2\0m^2/s^4\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _imuAccelerometerNoise changed to {}", nameId(), _imuAccelerometerNoise.transpose());
            LOG_DEBUG("{}: _imuAccelerometerNoiseUnit changed to {}", nameId(), _imuAccelerometerNoiseUnit);
            flow::ApplyChanges();
        }
        float currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderULong(fmt::format("Accelerometer Noise Seed##{}", size_t(id)).c_str(), &_imuAccelerometerRandomNumberGenerator.seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _imuAccelerometerRandomNumberGenerator.seed changed to {}", nameId(), _imuAccelerometerRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }

        // #########################################################################################################################################

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Gyroscope Noise ({})##{}",
                                                           _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::rad_s
                                                                   || _imuGyroscopeNoiseUnit == ImuGyroscopeNoiseUnits::deg_s
                                                               ? "Standard deviation"
                                                               : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _imuGyroscopeNoise.data(), reinterpret_cast<int*>(&_imuGyroscopeNoiseUnit), "rad/s\0deg/s\0rad^2/s^2\0deg^2/s^2\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _imuGyroscopeNoise changed to {}", nameId(), _imuGyroscopeNoise.transpose());
            LOG_DEBUG("{}: _imuGyroscopeNoiseUnit changed to {}", nameId(), _imuGyroscopeNoiseUnit);
            flow::ApplyChanges();
        }
        currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderULong(fmt::format("Gyroscope Noise Seed##{}", size_t(id)).c_str(), &(_imuGyroscopeRandomNumberGenerator.seed), 0, std::numeric_limits<uint64_t>::max() / 2UL, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _imuGyroscopeRandomNumberGenerator.seed changed to {}", nameId(), _imuGyroscopeRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        // #########################################################################################################################################
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        // #########################################################################################################################################
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Position Noise ({})##{}",
                                                           _positionNoiseUnit == PositionNoiseUnits::meter
                                                                   || _positionNoiseUnit == PositionNoiseUnits::rad_rad_m
                                                                   || _positionNoiseUnit == PositionNoiseUnits::deg_deg_m
                                                               ? "Standard deviation"
                                                               : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _positionNoise.data(), reinterpret_cast<int*>(&_positionNoiseUnit), "[m m m]\0[rad, rad, m]\0[deg, deg, m]\0[m^2 m^2 m^2]\0[rad^2, rad^2, m^2]\0[deg^2, deg^2, m^2]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _positionNoise changed to {}", nameId(), _positionNoise.transpose());
            LOG_DEBUG("{}: _positionNoiseUnit changed to {}", nameId(), _positionNoiseUnit);
            flow::ApplyChanges();
        }
        float currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_positionRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_positionRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _positionRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _positionRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_positionRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderULong(fmt::format("Position Noise Seed##{}", size_t(id)).c_str(), &_positionRandomNumberGenerator.seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _positionRandomNumberGenerator.seed changed to {}", nameId(), _positionRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_positionRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }

        // #########################################################################################################################################

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity Noise ({})##{}",
                                                           _velocityNoiseUnit == VelocityNoiseUnits::m_s ? "Standard deviation"
                                                                                                         : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _velocityNoise.data(), reinterpret_cast<int*>(&_velocityNoiseUnit), "[m/s]\0[m^2/s^2]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _velocityNoise changed to {}", nameId(), _velocityNoise.transpose());
            LOG_DEBUG("{}: _velocityNoiseUnit changed to {}", nameId(), _velocityNoiseUnit);
            flow::ApplyChanges();
        }
        currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _velocityRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _velocityRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderULong(fmt::format("Velocity Noise Seed##{}", size_t(id)).c_str(), &_velocityRandomNumberGenerator.seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _velocityRandomNumberGenerator.seed changed to {}", nameId(), _velocityRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }

        // #########################################################################################################################################

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Attitude Noise ({})##{}",
                                                           _attitudeNoiseUnit == AttitudeNoiseUnits::rad
                                                                   || _attitudeNoiseUnit == AttitudeNoiseUnits::deg
                                                               ? "Standard deviation"
                                                               : "Variance",
                                                           size_t(id))
                                                   .c_str(),
                                               itemWidth, unitWidth,
                                               _attitudeNoise.data(), reinterpret_cast<int*>(&_attitudeNoiseUnit), "[rad]\0[deg]\0[rad^2]\0[deg^2]\0\0", // NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _attitudeNoise changed to {}", nameId(), _attitudeNoise.transpose());
            LOG_DEBUG("{}: _attitudeNoiseUnit changed to {}", nameId(), _attitudeNoiseUnit);
            flow::ApplyChanges();
        }
        currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime{}", size_t(id)).c_str(), &_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime))
        {
            LOG_DEBUG("{}: _attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime changed to {}", nameId(), _attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime);
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        if (!_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX) + ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::SliderULong(fmt::format("Attitude Noise Seed##{}", size_t(id)).c_str(), &_attitudeRandomNumberGenerator.seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu", ImGuiSliderFlags_Logarithmic))
        {
            LOG_DEBUG("{}: _attitudeRandomNumberGenerator.seed changed to {}", nameId(), _attitudeRandomNumberGenerator.seed);
            flow::ApplyChanges();
        }
        if (!_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime)
        {
            ImGui::PopItemFlag();
            ImGui::PopStyleVar();
        }
        // #########################################################################################################################################
    }
    ImGui::Unindent();
}

json NAV::ErrorModel::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["imuAccelerometerBiasUnit"] = _imuAccelerometerBiasUnit;
    j["imuAccelerometerBias_p"] = _imuAccelerometerBias_p;
    j["imuGyroscopeBiasUnit"] = _imuGyroscopeBiasUnit;
    j["imuGyroscopeBias_p"] = _imuGyroscopeBias_p;

    j["imuAccelerometerNoiseUnit"] = _imuAccelerometerNoiseUnit;
    j["imuAccelerometerNoise"] = _imuAccelerometerNoise;
    j["imuAccelerometerRandomNumberGenerator"] = _imuAccelerometerRandomNumberGenerator;
    j["imuGyroscopeNoiseUnit"] = _imuGyroscopeNoiseUnit;
    j["imuGyroscopeNoise"] = _imuGyroscopeNoise;
    j["imuGyroscopeRandomNumberGenerator"] = _imuGyroscopeRandomNumberGenerator;
    // #########################################################################################################################################
    j["positionBiasUnit"] = _positionBiasUnit;
    j["positionBias"] = _positionBias;
    j["velocityBiasUnit"] = _velocityBiasUnit;
    j["velocityBias"] = _velocityBias;
    j["attitudeBiasUnit"] = _attitudeBiasUnit;
    j["attitudeBias"] = _attitudeBias;

    j["positionNoiseUnit"] = _positionNoiseUnit;
    j["positionNoise"] = _positionNoise;
    j["positionRandomNumberGenerator"] = _positionRandomNumberGenerator;
    j["velocityNoiseUnit"] = _velocityNoiseUnit;
    j["velocityNoise"] = _velocityNoise;
    j["velocityRandomNumberGenerator"] = _velocityRandomNumberGenerator;
    j["attitudeNoiseUnit"] = _attitudeNoiseUnit;
    j["attitudeNoise"] = _attitudeNoise;
    j["attitudeRandomNumberGenerator"] = _attitudeRandomNumberGenerator;

    return j;
}

void NAV::ErrorModel::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuAccelerometerBiasUnit"))
    {
        j.at("imuAccelerometerBiasUnit").get_to(_imuAccelerometerBiasUnit);
    }
    if (j.contains("imuAccelerometerBias_p"))
    {
        j.at("imuAccelerometerBias_p").get_to(_imuAccelerometerBias_p);
    }
    if (j.contains("imuGyroscopeBiasUnit"))
    {
        j.at("imuGyroscopeBiasUnit").get_to(_imuGyroscopeBiasUnit);
    }
    if (j.contains("imuGyroscopeBias_p"))
    {
        j.at("imuGyroscopeBias_p").get_to(_imuGyroscopeBias_p);
    }

    if (j.contains("imuAccelerometerNoiseUnit"))
    {
        j.at("imuAccelerometerNoiseUnit").get_to(_imuAccelerometerNoiseUnit);
    }
    if (j.contains("imuAccelerometerNoise"))
    {
        j.at("imuAccelerometerNoise").get_to(_imuAccelerometerNoise);
    }
    if (j.contains("imuAccelerometerRandomNumberGenerator"))
    {
        j.at("imuAccelerometerRandomNumberGenerator").get_to(_imuAccelerometerRandomNumberGenerator);
    }
    if (j.contains("imuGyroscopeNoiseUnit"))
    {
        j.at("imuGyroscopeNoiseUnit").get_to(_imuGyroscopeNoiseUnit);
    }
    if (j.contains("imuGyroscopeNoise"))
    {
        j.at("imuGyroscopeNoise").get_to(_imuGyroscopeNoise);
    }
    if (j.contains("imuGyroscopeRandomNumberGenerator"))
    {
        j.at("imuGyroscopeRandomNumberGenerator").get_to(_imuGyroscopeRandomNumberGenerator);
    }
    // #########################################################################################################################################
    if (j.contains("positionBiasUnit"))
    {
        j.at("positionBiasUnit").get_to(_positionBiasUnit);
    }
    if (j.contains("positionBias"))
    {
        j.at("positionBias").get_to(_positionBias);
    }
    if (j.contains("velocityBiasUnit"))
    {
        j.at("velocityBiasUnit").get_to(_velocityBiasUnit);
    }
    if (j.contains("velocityBias"))
    {
        j.at("velocityBias").get_to(_velocityBias);
    }
    if (j.contains("attitudeBiasUnit"))
    {
        j.at("attitudeBiasUnit").get_to(_attitudeBiasUnit);
    }
    if (j.contains("attitudeBias"))
    {
        j.at("attitudeBias").get_to(_attitudeBias);
    }

    if (j.contains("positionNoiseUnit"))
    {
        j.at("positionNoiseUnit").get_to(_positionNoiseUnit);
    }
    if (j.contains("positionNoise"))
    {
        j.at("positionNoise").get_to(_positionNoise);
    }
    if (j.contains("positionRandomNumberGenerator"))
    {
        j.at("positionRandomNumberGenerator").get_to(_positionRandomNumberGenerator);
    }
    if (j.contains("velocityNoiseUnit"))
    {
        j.at("velocityNoiseUnit").get_to(_velocityNoiseUnit);
    }
    if (j.contains("velocityNoise"))
    {
        j.at("velocityNoise").get_to(_velocityNoise);
    }
    if (j.contains("velocityRandomNumberGenerator"))
    {
        j.at("velocityRandomNumberGenerator").get_to(_velocityRandomNumberGenerator);
    }
    if (j.contains("attitudeNoiseUnit"))
    {
        j.at("attitudeNoiseUnit").get_to(_attitudeNoiseUnit);
    }
    if (j.contains("attitudeNoise"))
    {
        j.at("attitudeNoise").get_to(_attitudeNoise);
    }
    if (j.contains("attitudeRandomNumberGenerator"))
    {
        j.at("attitudeRandomNumberGenerator").get_to(_attitudeRandomNumberGenerator);
    }
}

bool NAV::ErrorModel::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        _imuAccelerometerRandomNumberGenerator.generator.seed(_imuAccelerometerRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                                  ? _imuAccelerometerRandomNumberGenerator.seed
                                                                  : static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
        _imuGyroscopeRandomNumberGenerator.generator.seed(_imuGyroscopeRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                              ? _imuGyroscopeRandomNumberGenerator.seed
                                                              : static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        _positionRandomNumberGenerator.generator.seed(_positionRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                          ? _positionRandomNumberGenerator.seed
                                                          : static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
        _velocityRandomNumberGenerator.generator.seed(_velocityRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                          ? _velocityRandomNumberGenerator.seed
                                                          : static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
        _attitudeRandomNumberGenerator.generator.seed(_attitudeRandomNumberGenerator.useSeedInsteadOfSystemTime
                                                          ? _attitudeRandomNumberGenerator.seed
                                                          : static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
    }

    return true;
}

void NAV::ErrorModel::afterCreateLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

        if (endPin->parentNode->id != id)
        {
            return; // Link on Output Port
        }

        // Store previous output pin identifier
        auto previousOutputPinDataIdentifier = outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier;
        // Overwrite output pin identifier with input pin identifier
        outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = startPin->dataIdentifier;

        if (previousOutputPinDataIdentifier != outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier) // If the identifier changed
        {
            // Check if connected links on output port are still valid
            for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OUTPUT_PORT_INDEX_FLOW).id))
            {
                auto* startPin = nm::FindPin(link->startPinId);
                auto* endPin = nm::FindPin(link->endPinId);
                if (startPin && endPin)
                {
                    if (startPin->canCreateLink(*endPin))
                    {
                        continue;
                    }
                }
                // If the link is not valid anymore, delete it
                nm::DeleteLink(link->id);
            }

            // Refresh all links connected to the output pin if the type changed
            if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier != previousOutputPinDataIdentifier)
            {
                for (auto* link : nm::FindConnectedLinksToOutputPin(outputPins.at(OUTPUT_PORT_INDEX_FLOW).id))
                {
                    nm::RefreshLink(link->id);
                }
            }
        }
    }
}

void NAV::ErrorModel::afterDeleteLink(Pin* startPin, Pin* endPin)
{
    if (startPin && endPin)
    {
        LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin->id), size_t(endPin->id));

        if ((endPin->parentNode->id != id                                       // Link on Output port is removed
             && !nm::IsPinLinked(inputPins.at(INPUT_PORT_INDEX_FLOW).id))       //     and the Input port is not linked
            || (startPin->parentNode->id != id                                  // Link on Input port is removed
                && !nm::IsPinLinked(outputPins.at(OUTPUT_PORT_INDEX_FLOW).id))) //     and the Output port is not linked
        {
            outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier = supportedDataIdentifier;
        }
    }
}

void NAV::ErrorModel::receiveObs(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /* linkId */)
{
    // Select the correct data type and make a copy of the node data to modify
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        receiveImuObs(std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(nodeData)));
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        receivePosVelAtt(std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(nodeData)));
    }
}

void NAV::ErrorModel::receiveImuObs(const std::shared_ptr<ImuObs>& imuObs)
{
    // Accelerometer Bias in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerBias_p = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerBiasUnit)
    {
    case ImuAccelerometerBiasUnits::m_s2:
        accelerometerBias_p = _imuAccelerometerBias_p;
        break;
    }
    LOG_DATA("{}: accelerometerBias_p = {} [m/s^2]", nameId(), accelerometerBias_p.transpose());

    // Gyroscope Bias in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeBias_p = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeBiasUnit)
    {
    case ImuGyroscopeBiasUnits::deg_s:
        gyroscopeBias_p = trafo::deg2rad(_imuGyroscopeBias_p);
        break;
    case ImuGyroscopeBiasUnits::rad_s:
        gyroscopeBias_p = _imuGyroscopeBias_p;
        break;
    }
    LOG_DATA("{}: gyroscopeBias_p = {} [rad/s]", nameId(), gyroscopeBias_p.transpose());

    // #########################################################################################################################################

    // Accelerometer Noise standard deviation in platform frame coordinates [m/s^2]
    Eigen::Vector3d accelerometerNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuAccelerometerNoiseUnit)
    {
    case ImuAccelerometerNoiseUnits::m_s2:
        accelerometerNoiseStd = _imuAccelerometerNoise;
        break;
    case ImuAccelerometerNoiseUnits::m2_s4:
        accelerometerNoiseStd = _imuAccelerometerNoise.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: accelerometerNoiseStd = {} [m/s^2]", nameId(), accelerometerNoiseStd.transpose());

    // Gyroscope Noise standard deviation in platform frame coordinates [rad/s]
    Eigen::Vector3d gyroscopeNoiseStd = Eigen::Vector3d::Zero();
    switch (_imuGyroscopeNoiseUnit)
    {
    case ImuGyroscopeNoiseUnits::rad_s:
        gyroscopeNoiseStd = _imuGyroscopeNoise;
        break;
    case ImuGyroscopeNoiseUnits::deg_s:
        gyroscopeNoiseStd = trafo::deg2rad(_imuGyroscopeNoise);
        break;
    case ImuGyroscopeNoiseUnits::rad2_s2:
        gyroscopeNoiseStd = _imuGyroscopeNoise.cwiseSqrt();
        break;
    case ImuGyroscopeNoiseUnits::deg2_s2:
        gyroscopeNoiseStd = trafo::deg2rad(_imuGyroscopeNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: gyroscopeNoiseStd = {} [rad/s]", nameId(), gyroscopeNoiseStd.transpose());

    // #########################################################################################################################################

    imuObs->accelUncompXYZ.value() += accelerometerBias_p
                                      + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(0) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                         std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(1) }(_imuAccelerometerRandomNumberGenerator.generator),
                                                         std::normal_distribution<double>{ 0.0, accelerometerNoiseStd(2) }(_imuAccelerometerRandomNumberGenerator.generator) };
    imuObs->gyroUncompXYZ.value() += gyroscopeBias_p
                                     + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(0) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                        std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(1) }(_imuGyroscopeRandomNumberGenerator.generator),
                                                        std::normal_distribution<double>{ 0.0, gyroscopeNoiseStd(2) }(_imuGyroscopeRandomNumberGenerator.generator) };

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, imuObs);
}

void NAV::ErrorModel::receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt)
{
    // Position Bias in latLonAlt in [rad, rad, m]
    Eigen::Vector3d positionBias_lla = Eigen::Vector3d::Zero();
    switch (_positionBiasUnit)
    {
    case PositionBiasUnits::meter:
    {
        Eigen::Vector3d positionBias_e = trafo::quat_en(posVelAtt->latitude(), posVelAtt->longitude()) * _positionBias;
        positionBias_lla = trafo::ecef2lla_WGS84(posVelAtt->position_ecef() + positionBias_e) - posVelAtt->latLonAlt();
        break;
    }
    case PositionBiasUnits::rad_rad_m:
        positionBias_lla = _positionBias;
        break;
    case PositionBiasUnits::deg_deg_m:
        positionBias_lla = Eigen::Vector3d{ trafo::deg2rad(_positionBias(0)), trafo::deg2rad(_positionBias(1)), _positionBias(2) };
        break;
    }
    LOG_DATA("{}: positionBias_lla = {} [rad, rad, m]", nameId(), positionBias_lla.transpose());

    // Velocity bias in local-navigation coordinates in [m/s]
    Eigen::Vector3d velocityBias_n = Eigen::Vector3d::Zero();
    switch (_velocityBiasUnit)
    {
    case VelocityBiasUnits::m_s:
        velocityBias_n = _velocityBias;
        break;
    }
    LOG_DATA("{}: velocityBias_n = {} [m/s]", nameId(), velocityBias_n.transpose());

    // Roll, pitch, yaw bias in [rad]
    Eigen::Vector3d attitudeBias = Eigen::Vector3d::Zero();
    switch (_attitudeBiasUnit)
    {
    case AttitudeBiasUnits::rad:
        attitudeBias = _attitudeBias;
        break;
    case AttitudeBiasUnits::deg:
        attitudeBias = trafo::deg2rad(_attitudeBias);
        break;
    }
    LOG_DATA("{}: attitudeBias = {} [rad]", nameId(), attitudeBias.transpose());

    // #########################################################################################################################################

    // Position Noise standard deviation in latitude, longitude and altitude [rad, rad, m]
    Eigen::Vector3d positionNoiseStd_lla = Eigen::Vector3d::Zero();
    switch (_positionNoiseUnit)
    {
    case PositionNoiseUnits::meter:
    {
        Eigen::Vector3d positionNoiseStd_e = trafo::quat_en(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise;
        positionNoiseStd_lla = trafo::ecef2lla_WGS84(posVelAtt->position_ecef() + positionNoiseStd_e) - posVelAtt->latLonAlt();
        break;
    }
    case PositionNoiseUnits::rad_rad_m:
        positionNoiseStd_lla = _positionNoise;
        break;
    case PositionNoiseUnits::deg_deg_m:
        positionNoiseStd_lla = trafo::deg2rad(_positionNoise);
        break;
    case PositionNoiseUnits::meter2:
    {
        Eigen::Vector3d positionNoiseStd_e = trafo::quat_en(posVelAtt->latitude(), posVelAtt->longitude()) * _positionNoise.cwiseSqrt();
        positionNoiseStd_lla = trafo::ecef2lla_WGS84(posVelAtt->position_ecef() + positionNoiseStd_e) - posVelAtt->latLonAlt();
        break;
    }
    case PositionNoiseUnits::rad2_rad2_m2:
        positionNoiseStd_lla = _positionNoise.cwiseSqrt();
        break;
    case PositionNoiseUnits::deg2_deg2_m2:
        positionNoiseStd_lla = trafo::deg2rad(_positionNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: positionNoiseStd_lla = {} [rad, rad, m]", nameId(), positionNoiseStd_lla.transpose());

    // Velocity Noise standard deviation in local-navigation coordinates in [m/s]
    Eigen::Vector3d velocityNoiseStd_n = Eigen::Vector3d::Zero();
    switch (_velocityNoiseUnit)
    {
    case VelocityNoiseUnits::m_s:
        velocityNoiseStd_n = _velocityNoise;
        break;
    case VelocityNoiseUnits::m2_s2:
        velocityNoiseStd_n = _velocityNoise.cwiseSqrt();
        break;
    }
    LOG_DATA("{}: velocityNoiseStd_n = {} [m/s]", nameId(), velocityNoiseStd_n.transpose());

    // Attitude Noise standard deviation in [rad]
    Eigen::Vector3d attitudeNoiseStd = Eigen::Vector3d::Zero();
    switch (_attitudeNoiseUnit)
    {
    case AttitudeNoiseUnits::rad:
        attitudeNoiseStd = _attitudeNoise;
        break;
    case AttitudeNoiseUnits::deg:
        attitudeNoiseStd = trafo::deg2rad(_attitudeNoise);
        break;
    case AttitudeNoiseUnits::rad2:
        attitudeNoiseStd = _attitudeNoise.cwiseSqrt();
        break;
    case AttitudeNoiseUnits::deg2:
        attitudeNoiseStd = trafo::deg2rad(_attitudeNoise.cwiseSqrt());
        break;
    }
    LOG_DATA("{}: attitudeNoiseStd = {} [rad]", nameId(), attitudeNoiseStd.transpose());

    // #########################################################################################################################################

    posVelAtt->setState_n(posVelAtt->latLonAlt()
                              + positionBias_lla
                              + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, positionNoiseStd_lla(0) }(_positionRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, positionNoiseStd_lla(1) }(_positionRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, positionNoiseStd_lla(2) }(_positionRandomNumberGenerator.generator) },
                          posVelAtt->velocity_n()
                              + velocityBias_n
                              + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, velocityNoiseStd_n(0) }(_velocityRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, velocityNoiseStd_n(1) }(_velocityRandomNumberGenerator.generator),
                                                 std::normal_distribution<double>{ 0.0, velocityNoiseStd_n(2) }(_velocityRandomNumberGenerator.generator) },
                          trafo::quat_nb(posVelAtt->rollPitchYaw()
                                         + attitudeBias
                                         + Eigen::Vector3d{ std::normal_distribution<double>{ 0.0, attitudeNoiseStd(0) }(_attitudeRandomNumberGenerator.generator),
                                                            std::normal_distribution<double>{ 0.0, attitudeNoiseStd(1) }(_attitudeRandomNumberGenerator.generator),
                                                            std::normal_distribution<double>{ 0.0, attitudeNoiseStd(2) }(_attitudeRandomNumberGenerator.generator) }));

    invokeCallbacks(OUTPUT_PORT_INDEX_FLOW, posVelAtt);
}