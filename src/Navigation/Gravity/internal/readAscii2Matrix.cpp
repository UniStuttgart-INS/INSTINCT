#include "readAscii2Matrix.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <filesystem>

#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

Eigen::MatrixXd NAV::gravity::internal::readAscii2Matrix()
{
    std::string line;
    std::ifstream myfileN("resources/data/egm96_to360.ascii");
    std::ifstream myfile("resources/data/egm96_to360.ascii");

    char delimiter = ' ';
    size_t pos = 0;
    std::string token;
    uint32_t nmbrOfLines = static_cast<uint32_t>(std::count(std::istreambuf_iterator<char>(myfileN), std::istreambuf_iterator<char>(), '\n'));

    LOG_DATA("Number of lines in 'egm96_to360.ascii':_{}", nmbrOfLines);

    int i = 0;
    int j = 0;
    Eigen::MatrixXd coeffs(nmbrOfLines, 6);

    if (myfile.good())
    {
        while (getline(myfile, line))
        {
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                if (line.substr(0, 1) == " ")
                {
                    NAV::str::ltrim(line);
                }
                else
                {
                    token = line.substr(0, pos);
                    coeffs(i, j) = std::strtod(token.c_str(), nullptr);

                    line.erase(0, pos + 1);

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
    }
    else
    {
        LOG_CRITICAL("Unable to open file 'egm96_to360.ascii' --> gravity vector compensation not trustworthy");
        coeffs = Eigen::MatrixXd::Zero(1, 6);
    }
    return coeffs;
}