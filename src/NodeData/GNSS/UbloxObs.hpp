/**
 * @file UbloxObs.hpp
 * @brief ublox Observation Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-19
 */

#pragma once

#include "GnssObs.hpp"

#include "ub/protocol/types.hpp"
#include "ub/protocol/packet.hpp"

namespace NAV
{
/// ublox Observation Class
class UbloxObs : public GnssObs
{
  public:
    UbloxObs() = default;                          ///< Constructor
    ~UbloxObs() override = default;                ///< Destructor
    UbloxObs(const UbloxObs&) = delete;            ///< Copy constructor
    UbloxObs(UbloxObs&&) = delete;                 ///< Move constructor
    UbloxObs& operator=(const UbloxObs&) = delete; ///< Copy assignment operator
    UbloxObs& operator=(UbloxObs&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("UbloxObs");
    }

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "GnssObs" };
        return parents;
    }

    ub::protocol::uart::UbxClass msgClass = ub::protocol::uart::UbxClass::UBX_CLASS_NONE;
    uint8_t msgId = 0;
    uint16_t payloadLength = 0;

    ub::protocol::uart::Packet* p = nullptr;
};

} // namespace NAV
