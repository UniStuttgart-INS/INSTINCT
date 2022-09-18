/// @file GnssObs.hpp
/// @brief GNSS Observation messages
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-26

#pragma once

#include <limits>
#include <vector>
#include <algorithm>

#include "NodeData/NodeData.hpp"

#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "util/Assert.h"

namespace NAV
{
/// GNSS Observation message information
class GnssObs : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "GnssObs";
    }

    /// @brief Stores the satellites observations
    struct ObservationData
    {
        /// @brief Constructor
        /// @param[in] satSigId Satellite signal identifier (frequency and satellite number)
        /// @param[in] code Signal code
        ObservationData(const SatSigId& satSigId, const Code code) : satSigId(satSigId), code(code) {}

        SatSigId satSigId = { Freq_None, 0 };                           ///< Frequency and satellite number
        Code code;                                                      ///< GNSS Code
        double pseudorange = std::numeric_limits<double>::quiet_NaN();  ///< Pseudorange measurement [m]
        double carrierPhase = std::numeric_limits<double>::quiet_NaN(); ///< Carrier phase measurement [cycles]
        double doppler = std::numeric_limits<double>::quiet_NaN();      ///< Doppler measurement [Hz]
        double CN0 = std::numeric_limits<double>::quiet_NaN();          ///< Carrier-to-Noise density [dBHz]
        int8_t LLI = -1;                                                ///< Loss of Lock Indicator [0...6]
    };

    /// @brief Satellite observations
    std::vector<ObservationData> data;

    /// @brief Return the element with the identifier or a newly constructed one if it did not exist
    /// @param[in] freq Signal frequency (also identifies the satellite system)
    /// @param[in] satNum Number of the satellite
    /// @param[in] code Signal code
    /// @return The element found in the observations or a newly constructed one
    ObservationData& operator()(const Frequency& freq, uint16_t satNum, Code code)
    {
        auto iter = std::find_if(data.begin(), data.end(), [freq, satNum, code](const ObservationData& idData) {
            return idData.satSigId.freq == freq && idData.satSigId.satNum == satNum && idData.code == code;
        });
        if (iter != data.end())
        {
            return *iter;
        }

        data.emplace_back(SatSigId{ freq, satNum }, code);
        return data.back();
    }

    /// @brief Return the element with the identifier
    /// @param[in] freq Signal frequency (also identifies the satellite system)
    /// @param[in] satNum Number of the satellite
    /// @param[in] code Signal code
    /// @return The element found in the observations
    const ObservationData& operator()(const Frequency& freq, uint16_t satNum, Code code) const
    {
        auto iter = std::find_if(data.begin(), data.end(), [freq, satNum, code](const ObservationData& idData) {
            return idData.satSigId.freq == freq && idData.satSigId.satNum == satNum && idData.code == code;
        });

        INS_ASSERT_USER_ERROR(iter != data.end(), "You can not insert new elements in a const context.");
        return *iter;
    }
};

} // namespace NAV
