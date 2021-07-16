#include <catch2/catch.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include "Nodes/FlowTester.hpp"

#include "util/Logger.hpp"

namespace NAV
{
// TEST_CASE("[VectorNavDataLogger] Read file and pass data to logger. Then compare logged data to original", "[VectorNavDataLogger]")
// {
// FIXME: This test is currently disabled till issue 'Flow Testing can assert NodeData' (https://git.nav.uni-stuttgart.de/thomas.topp/instinct/-/issues/76) is resolved
// Logger logger;

// testFlow("test/flow/VectorNavDataLogger.flow");

// std::ifstream filestream_orig{ "test/data/vectornav.csv", std::ios::binary };
// std::vector<std::pair<std::string, double>> origData;
// readCsvHeader(filestream_orig, origData);

// std::ifstream filestream_new{ "test/logs/VectorNavDataLogger.csv", std::ios::binary };
// std::vector<std::pair<std::string, double>> newData;
// readCsvHeader(filestream_new, newData);

// // Read line & Convert into stringstream
// std::string line_orig;
// int lineCnt{ 1 };
// while (std::getline(filestream_orig, line_orig))
// {
//     LOG_DATA("Comparing line '{}'", lineCnt);

//     std::stringstream lineStream_orig(line_orig);
//     // Read line & Convert into stringstream
//     std::string line_new;
//     if (!std::getline(filestream_new, line_new))
//     {
//         LOG_CRITICAL("The written file has less lines than the original");
//     }
//     std::stringstream lineStream_new(line_new);

//     for (auto& data : origData)
//     {
//         std::string cell;
//         if (std::getline(lineStream_orig, cell, ','))
//         {
//             // Remove any trailing non text characters
//             cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
//             if (cell.empty())
//             {
//                 continue;
//             }

//             data.second = std::stod(cell);
//         }
//         else
//         {
//             LOG_CRITICAL("The original data file has less input data than the header specifies");
//         }
//     }
//     for (auto& data : newData)
//     {
//         std::string cell;
//         if (std::getline(lineStream_new, cell, ','))
//         {
//             // Remove any trailing non text characters
//             cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
//             if (cell.empty())
//             {
//                 continue;
//             }

//             data.second = std::stod(cell);
//         }
//         else
//         {
//             LOG_CRITICAL("The new data file has less input data than the header specifies");
//         }
//     }

//     for (auto& newElement : newData)
//     {
//         std::string key = newElement.first;

//         auto origDataIter = std::find_if(origData.begin(), origData.end(),
//                                          [&key](const std::pair<std::string, double>& element) {
//                                              return element.first == key;
//                                          });
//         REQUIRE(origDataIter != origData.end());

//         if (!std::isnan(origDataIter->second) || !std::isnan(newElement.second))
//         {
//             LOG_DATA("  Comparing key '{}'", key);
//             REQUIRE(origDataIter->second == newElement.second);
//         }
//     }

//     lineCnt++;
// }
// }

} // namespace NAV
