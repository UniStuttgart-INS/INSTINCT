#include "Ionosphere.hpp"

#include <vector>
#include <array>
#include <imgui.h>
#include "util/Logger.hpp"

#include "Models/Klobuchar.hpp"

namespace NAV
{

const char* to_string(IonosphereModel ionosphereModel)
{
    switch (ionosphereModel)
    {
    case IonosphereModel::None:
        return "None";
    case IonosphereModel::Klobuchar:
        return "Klobuchar / Broadcast";
    case IonosphereModel::COUNT:
        break;
    }
    return "";
}

bool ComboIonosphereModel(const char* label, IonosphereModel& ionosphereModel)
{
    bool clicked = false;
    if (ImGui::BeginCombo(label, NAV::to_string(ionosphereModel)))
    {
        for (size_t i = 0; i < static_cast<size_t>(IonosphereModel::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(ionosphereModel) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<IonosphereModel>(i)), is_selected))
            {
                ionosphereModel = static_cast<IonosphereModel>(i);
                clicked = true;
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }
    return clicked;
}

double calcIonosphericTimeDelay(double tow, Frequency freq,
                                const Eigen::Vector3d& lla_pos,
                                double elevation, double azimuth,
                                const std::vector<double>& alpha, const std::vector<double>& beta,
                                IonosphereModel ionosphereModel)
{
    switch (ionosphereModel)
    {
    case IonosphereModel::Klobuchar:
    {
        if (alpha.size() != 4 || beta.size() != 4)
        {
            LOG_ERROR("Ionosphere model Broadcast needs 4 parameters for alpha and beta. Can't calculate ionosphere model.");
            break;
        }
        std::array<double, 4> a{};
        std::copy(alpha.begin(), alpha.end(), a.begin());
        std::array<double, 4> b{};
        std::copy(beta.begin(), beta.end(), b.begin());
        return calcIonosphericTimeDelay_Klobuchar(tow, freq, lla_pos(0), lla_pos(1), elevation, azimuth, a, b);
    }
    case IonosphereModel::None:
    case IonosphereModel::COUNT:
        break;
    }

    return 0.0;
}

} // namespace NAV
