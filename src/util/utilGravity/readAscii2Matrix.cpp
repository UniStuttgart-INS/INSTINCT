#include "readAscii2Matrix.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include "util/Logger.hpp"
#include <filesystem>

Eigen::MatrixXd NAV::utilGravity::readAscii2Matrix()
{
    std::string line;
    std::ifstream myfileN("../resources/Data/egm96_to360.ascii");
    std::ifstream myfile("../resources/Data/egm96_to360.ascii");
    std::string delimiter = " ";
    size_t pos = 0;
    std::string token;
    uint32_t nmbrOfLines = static_cast<uint32_t>(std::count(std::istreambuf_iterator<char>(myfileN), std::istreambuf_iterator<char>(), '\n'));
    // loadfile.seekg(0, std::ios::beg); // Resetting the line counter of istreambuf_iterator

    LOG_DEBUG("Number of lines in 'egm96_to360.ascii':_{}", nmbrOfLines);
    //LOG_DEBUG("Current file path:_{}", std::filesystem::current_path()); //During run of INSTINCT: "/home/mm/Documents/Git/INSTINCT/flow"

    int i = 0;
    int j = 0;
    Eigen::MatrixXd coeffs(nmbrOfLines, 6);
    //Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(nmbrOfLines,6);

    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                if (line.substr(0, 1) == " ")
                {
                    line.erase(0, 1);
                }
                else
                {
                    token = line.substr(0, pos);
                    coeffs(i, j) = std::strtod(token.c_str(), nullptr);

                    line.erase(0, pos + delimiter.length());

                    if (j < 5)
                    {
                        j++;
                    }
                }
            }
            coeffs(i, 5) = std::strtod(line.c_str(), nullptr);
            j = 0;
            if (static_cast<uint32_t>(i + 1) < nmbrOfLines)
            {
                i++;
            }
        }
        myfile.close();
    }
    else
    {
        LOG_DEBUG("Unable to open file 'egm96_to360.ascii'");
    }

    return coeffs;
}