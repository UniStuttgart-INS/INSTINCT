// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "KmlLogger.hpp"

#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Geoid/EGM96.hpp"
#include "NodeData/NodeData.hpp"

#include "NodeData/State/Pos.hpp"
#include "internal/Node/Pin.hpp"
#include "util/Logger.hpp"

#include <algorithm>
#include <iomanip> // std::setprecision
#include <memory>

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "NodeRegistry.hpp"

NAV::KmlLogger::KmlLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::ASCII;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };
}

NAV::KmlLogger::~KmlLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::KmlLogger::typeStatic()
{
    return "KmlLogger";
}

std::string NAV::KmlLogger::type() const
{
    return typeStatic();
}

std::string NAV::KmlLogger::category()
{
    return "Data Logger";
}

void NAV::KmlLogger::guiConfig()
{
    if (FileWriter::guiConfig(".kml", { ".kml" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
    }
    if (_dynamicInputPins.ShowGuiWidgets(size_t(id), inputPins, this))
    {
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::KmlLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["dynamicInputPins"] = _dynamicInputPins;
    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::KmlLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicInputPins"))
    {
        NAV::gui::widgets::from_json(j.at("dynamicInputPins"), _dynamicInputPins, this);
    }
    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::KmlLogger::flush()
{
    LOG_DEBUG("{}: Received all data. Writing file now...", nameId());
    _filestream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                   "<kml xmlns=\"http://earth.google.com/kml/2.1\">\n"
                   "<Document>\n"
                   "<Style id=\"P0\">\n"
                   "  <IconStyle>\n"
                   "    <color>ffffffff</color>\n"
                   "    <scale>0.3</scale>\n"
                   "    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>\n"
                   "  </IconStyle>\n"
                   "</Style>\n"
                   "<Style id=\"P1\">\n"
                   "  <IconStyle>\n"
                   "    <color>ff008800</color>\n"
                   "    <scale>0.2</scale>\n"
                   "    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>\n"
                   "  </IconStyle>\n"
                   "</Style>\n"
                   "<Style id=\"P2\">\n"
                   "  <IconStyle>\n"
                   "    <color>ff00aaff</color>\n"
                   "    <scale>0.2</scale>\n"
                   "    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>\n"
                   "  </IconStyle>\n"
                   "</Style>\n"
                   "<Style id=\"P3\">\n"
                   "  <IconStyle>\n"
                   "    <color>ff0000ff</color>\n"
                   "    <scale>0.2</scale>\n"
                   "    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>\n"
                   "  </IconStyle>\n"
                   "</Style>\n"
                   "<Style id=\"P4\">\n"
                   "  <IconStyle>\n"
                   "    <color>ff00ffff</color>\n"
                   "    <scale>0.2</scale>\n"
                   "    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>\n"
                   "  </IconStyle>\n"
                   "</Style>\n"
                   "<Style id=\"P5\">\n"
                   "  <IconStyle>\n"
                   "    <color>ffff00ff</color>\n"
                   "    <scale>0.2</scale>\n"
                   "    <Icon><href>http://maps.google.com/mapfiles/kml/pal2/icon18.png</href></Icon>\n"
                   "  </IconStyle>\n"
                   "</Style>\n";

    for (size_t i = 0; i < _positionData.size(); i++)
    {
        const auto& posData = _positionData.at(i);
        if (posData.empty()) { continue; }

        if (posData.size() > 1) // Track
        {
            _filestream << "<Placemark>\n"
                        << "<name>" << inputPins.at(i).name << " Track</name>\n"
                        << "<Style>\n"
                           "<LineStyle>\n"
                           "<color>ff00ffff</color>\n"
                           "</LineStyle>\n"
                           "</Style>";

            _filestream << "<LineString>\n"
                           "<altitudeMode>absolute</altitudeMode>\n"
                           "<coordinates>\n";
            for (const auto& lla : posData)
            {
                fmt::print(_filestream, "{:.9f},{:.9f},{:.3f}\n", lla.y(), lla.x(), lla.z());
            }

            _filestream << "</coordinates>\n"
                           "</LineString>\n"
                           "</Placemark>\n";
        }

        // Position
        {
            if (posData.size() > 1)
            {
                _filestream << "<Folder>\n"
                            << "<name>" << inputPins.at(i).name << " Position</name>\n";
            }
            for (const auto& lla : posData)
            {
                _filestream << "<Placemark>\n";
                if (posData.size() > 1)
                {
                    _filestream << "<styleUrl>#P2</styleUrl>\n";
                }
                else
                {
                    _filestream << "<name>" << inputPins.at(i).name << " Position</name>\n";
                    _filestream << "<styleUrl>#P0</styleUrl>\n";
                }
                _filestream << "<Point>\n"
                               "<extrude>1</extrude>\n"
                               "<altitudeMode>absolute</altitudeMode>\n"
                            << "<coordinates>";
                fmt::print(_filestream, "{:.9f},{:.9f},{:.3f}", lla.y(), lla.x(), lla.z()); //
                _filestream << "</coordinates>\n"
                               "</Point>\n"
                               "</Placemark>\n";
            }
            if (posData.size() > 1)
            {
                _filestream << "</Folder>\n";
            }
        }
    }

    _filestream << "</Document>\n"
                << "</kml>\n";
    _filestream.flush();
}

bool NAV::KmlLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    for (auto& posData : _positionData)
    {
        posData.clear();
    }

    return true;
}

void NAV::KmlLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::KmlLogger::pinAddCallback(Node* node)
{
    auto* kmlNode = static_cast<KmlLogger*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)

    nm::CreateInputPin(node, fmt::format("Pin {}", node->inputPins.size() + 1).c_str(), Pin::Type::Flow,
                       { Pos::type() },
                       &KmlLogger::writeObservation);

    kmlNode->_positionData.emplace_back();
}

void NAV::KmlLogger::pinDeleteCallback(Node* node, size_t pinIdx)
{
    auto* kmlNode = static_cast<KmlLogger*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)

    kmlNode->_positionData.erase(std::next(kmlNode->_positionData.begin(), static_cast<int64_t>(pinIdx)));

    nm::DeleteInputPin(node->inputPins.at(pinIdx));
}

void NAV::KmlLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t pinIdx)
{
    auto obs = std::static_pointer_cast<const Pos>(queue.extract_front());
    LOG_INFO("{}: [{}] Received data {}", nameId(), obs->insTime.toYMDHMS(GPST), egm96_compute_altitude_offset(obs->latitude(), obs->longitude()));

    _positionData.at(pinIdx).emplace_back(rad2deg(obs->latitude()),
                                          rad2deg(obs->longitude()),
                                          obs->altitude() - egm96_compute_altitude_offset(obs->latitude(), obs->longitude()));
}