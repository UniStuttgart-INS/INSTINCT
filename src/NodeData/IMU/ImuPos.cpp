#include "ImuPos.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

namespace NAV
{
/// @brief Converts the provided imu position into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] pos Imu position to convert into json
void to_json(json& j, const ImuPos& pos)
{
    j = json{
        { "posAccel_b", pos.posAccel_b() },
        { "posGyro_b", pos.posGyro_b() },
        { "posMag_b", pos.posMag_b() },
        { "quatAccel_bp", pos.quatAccel_bp().coeffs() },
        { "quatGyro_bp", pos.quatGyro_bp().coeffs() },
        { "quatMag_bp", pos.quatMag_bp().coeffs() },
    };
}
/// @brief Converts the provided json object into a imu position object
/// @param[in] j Json object with the needed values
/// @param[out] pos Object to fill from the json
void from_json(const json& j, ImuPos& pos)
{
    if (j.contains("posAccel_b"))
    {
        j.at("posAccel_b").get_to(pos.positionAccel_b);
    }
    if (j.contains("positionGyro_b"))
    {
        j.at("positionGyro_b").get_to(pos.positionGyro_b);
    }
    if (j.contains("posMag_b"))
    {
        j.at("posMag_b").get_to(pos.positionMag_b);
    }
    if (j.contains("quatAccel_bp"))
    {
        j.at("quatAccel_bp").get_to(pos.quaternionAccel_bp.coeffs());
    }
    if (j.contains("quatGyro_bp"))
    {
        j.at("quatGyro_bp").get_to(pos.quaternionGyro_bp.coeffs());
    }
    if (j.contains("quatMag_bp"))
    {
        j.at("quatMag_bp").get_to(pos.quaternionMag_bp.coeffs());
    }
}

} // namespace NAV
