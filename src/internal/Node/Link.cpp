#include "Link.hpp"

#include "util/Json.hpp"

void NAV::to_json(json& j, const Link& link)
{
    j = json{
        { "id", size_t(link.id) },
        { "startPinId", size_t(link.startPinId) },
        { "endPinId", size_t(link.endPinId) },
        { "color", link.color },
    };
}
void NAV::from_json(const json& j, Link& link)
{
    size_t id = 0;
    j.at("id").get_to(id);
    link.id = id;

    j.at("startPinId").get_to(id);
    link.startPinId = id;

    j.at("endPinId").get_to(id);
    link.endPinId = id;

    if (j.contains("color"))
    {
        j.at("color").get_to(link.color);
    }
}