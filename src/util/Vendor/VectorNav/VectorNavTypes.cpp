// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "VectorNavTypes.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "util/Logger.hpp"

NAV::SatelliteSystem NAV::vendor::vectornav::toSatelliteSystem(NAV::vendor::vectornav::SatSys sys)
{
    SatelliteSystem satSys = SatSys_None;
    switch (sys)
    {
    case vendor::vectornav::SatSys::GPS:
        satSys = GPS;
        break;
    case vendor::vectornav::SatSys::SBAS:
        satSys = SBAS;
        break;
    case vendor::vectornav::SatSys::Galileo:
        satSys = GAL;
        break;
    case vendor::vectornav::SatSys::BeiDou:
        satSys = BDS;
        break;
    case vendor::vectornav::SatSys::IMES:
        LOG_TRACE("VectorNav SatInfoElement satellite system '{}' is not supported yet. Skipping measurement.", sys);
        break;
    case vendor::vectornav::SatSys::QZSS:
        satSys = QZSS;
        break;
    case vendor::vectornav::SatSys::GLONASS:
        satSys = GLO;
        break;
    default: // IRNSS not in vectorNav
        LOG_TRACE("VectorNav SatInfoElement satellite system '{}' is not supported yet. Skipping measurement.", sys);
        break;
    }
    return satSys;
}

NAV::vendor::vectornav::SatSys NAV::vendor::vectornav::fromSatelliteSystem(NAV::SatelliteSystem sys)
{
    switch (sys.toEnumeration())
    {
    case SatelliteSystem::Enum_GPS:
        return vendor::vectornav::SatSys::GPS;
    case SatelliteSystem::Enum_GAL:
        return vendor::vectornav::SatSys::Galileo;
    case SatelliteSystem::Enum_GLO:
        return vendor::vectornav::SatSys::GLONASS;
    case SatelliteSystem::Enum_BDS:
        return vendor::vectornav::SatSys::BeiDou;
    case SatelliteSystem::Enum_QZSS:
        return vendor::vectornav::SatSys::QZSS;
    case SatelliteSystem::Enum_SBAS:
        return vendor::vectornav::SatSys::SBAS;
    case SatelliteSystem::Enum_IRNSS:
    case SatelliteSystem::Enum_COUNT:
    case SatelliteSystem::Enum_None:
        LOG_TRACE("'{}' is not supported in the VectorNav satellite systems. Defaulting to GPS.", sys);
        return vendor::vectornav::SatSys::GPS;
    }
    return vendor::vectornav::SatSys::GPS;
}

[[nodiscard]] NAV::Code NAV::vendor::vectornav::RawMeas::SatRawElement::toCode() const
{
    switch (sys)
    {
    case SatSys::GPS:
        switch (freq)
        {
        case Freq::RxChannel:
            return Code::None;
        case Freq::L1:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::G1P;
            case Chan::CA_Code:
                return Code::G1C;
            case Chan::SemiCodeless:
                return Code::G1W;
            case Chan::Y_Code:
                return Code::G1Y;
            case Chan::M_Code:
                return Code::G1M;
            case Chan::Codeless:
                return Code::G1N;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::G1M;
            case Chan::L_Chan:
                return Code::G1L;
            case Chan::BC_Chan:
                return Code::G1X;
            case Chan::Z_Tracking:
                return Code::G1W;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L2:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::G2P;
            case Chan::CA_Code:
                return Code::G2C;
            case Chan::SemiCodeless:
                return Code::G2D;
            case Chan::Y_Code:
                return Code::G2Y;
            case Chan::M_Code:
                return Code::G2M;
            case Chan::Codeless:
                return Code::G2N;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::G2M;
            case Chan::L_Chan:
                return Code::G2L;
            case Chan::BC_Chan:
                return Code::G2X;
            case Chan::Z_Tracking:
                return Code::G2W;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L5:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::G5I;
            case Chan::Q_Chan:
                return Code::G5Q;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::G5X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E6:
        case Freq::E5b:
        case Freq::E5a:
            return Code::None;
        } // GPS-freq
        break;
    case SatSys::SBAS:
        switch (freq)
        {
        case Freq::RxChannel:
            return Code::None;
        case Freq::L1:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::S1C;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::None;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L2:
            return Code::None;
        case Freq::L5:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::S5I;
            case Chan::Q_Chan:
                return Code::S5Q;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::None;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E6:
        case Freq::E5b:
        case Freq::E5a:
            return Code::None;
        } // SBAS-freq
        break;
    case SatSys::Galileo:
        switch (freq)
        {
        case Freq::RxChannel:
            return Code::None;
        case Freq::L1:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::E1C;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::E1A;
            case Chan::B_Chan:
                return Code::E1B;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::E1X;
            case Chan::Z_Tracking:
                return Code::E1Z;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L2:
            return Code::None;
        case Freq::L5:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::E5I;
            case Chan::Q_Chan:
                return Code::E5Q;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::E5X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E6:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::E6C;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::E6A;
            case Chan::B_Chan:
                return Code::E6B;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::E6X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::E6Z;
            }
            break;
        case Freq::E5b:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::E7I;
            case Chan::Q_Chan:
                return Code::E7Q;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::E7X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E5a:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::E8I;
            case Chan::Q_Chan:
                return Code::E8Q;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::E8X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        } // GAL-freq
        break;
    case SatSys::BeiDou:
        switch (freq)
        {
        case Freq::RxChannel:
            return Code::None;
        case Freq::L1:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::B1P;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::None;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L2:
            return Code::None;
        case Freq::L5:
            return Code::None;
        case Freq::E6:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::B6A;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::B6I;
            case Chan::Q_Chan:
                return Code::B6Q;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::B6X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E5b:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::B8P;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::B8X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E5a:
            return Code::None;
        } // BDS-freq
        break;
    case SatSys::IMES:
        return Code::None;
    case SatSys::QZSS:
        switch (freq)
        {
        case Freq::RxChannel:
            return Code::None;
        case Freq::L1:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::J1C;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::J1L;
            case Chan::BC_Chan:
                return Code::J1X;
            case Chan::Z_Tracking:
                return Code::J1Z;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L2:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::J2L;
            case Chan::BC_Chan:
                return Code::J2X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L5:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::J5I;
            case Chan::Q_Chan:
                return Code::J5Q;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::J5X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E6:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::None;
            case Chan::CA_Code:
                return Code::None;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::J6L;
            case Chan::BC_Chan:
                return Code::J6X;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::E5b:
        case Freq::E5a:
            return Code::None;
        } // QZSS-freq
        break;
    case SatSys::GLONASS:
        switch (freq)
        {
        case Freq::RxChannel:
            return Code::None;
        case Freq::L1:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::R1P;
            case Chan::CA_Code:
                return Code::R1C;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::None;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L2:
            switch (chan)
            {
            case Chan::P_Code:
                return Code::R2P;
            case Chan::CA_Code:
                return Code::R2C;
            case Chan::SemiCodeless:
                return Code::None;
            case Chan::Y_Code:
                return Code::None;
            case Chan::M_Code:
                return Code::None;
            case Chan::Codeless:
                return Code::None;
            case Chan::A_Chan:
                return Code::None;
            case Chan::B_Chan:
                return Code::None;
            case Chan::I_Chan:
                return Code::None;
            case Chan::Q_Chan:
                return Code::None;
            case Chan::M_Chan:
                return Code::None;
            case Chan::L_Chan:
                return Code::None;
            case Chan::BC_Chan:
                return Code::None;
            case Chan::Z_Tracking:
                return Code::None;
            case Chan::ABC:
                return Code::None;
            }
            break;
        case Freq::L5:
        case Freq::E6:
        case Freq::E5b:
        case Freq::E5a:
            return Code::None;
        } // GLO-freq
        break;
    }
    return Code::None;
}

std::ostream& NAV::vendor::vectornav::operator<<(std::ostream& os, const NAV::vendor::vectornav::SatSys& satSys)
{
    switch (satSys)
    {
    case SatSys::GPS:
        os << "GPS";
        break;
    case SatSys::SBAS:
        os << "SBAS";
        break;
    case SatSys::Galileo:
        os << "Galileo";
        break;
    case SatSys::BeiDou:
        os << "BeiDou";
        break;
    case SatSys::IMES:
        os << "IMES";
        break;
    case SatSys::QZSS:
        os << "QZSS";
        break;
    case SatSys::GLONASS:
        os << "GLONASS";
        break;
    }
    return os;
}