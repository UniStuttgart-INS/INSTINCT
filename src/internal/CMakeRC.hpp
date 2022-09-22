// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CMakeRC.hpp
/// @brief Handles compiling resources into the binary
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-08-24
/// @see https://caiorss.github.io/C-Cpp-Notes/resources-executable.html#org7635e6a

#pragma once
#include <fstream>

#include <cmrc/cmrc.hpp>

/// @brief Defines the namespace of the filesystem
CMRC_DECLARE(instinct);

namespace cmrc
{
/// Credits: https://stackoverflow.com/a/13059195
/// https://stackoverflow.com/questions/13059091/
struct membuf : std::streambuf
{
    /// @brief Constructor
    /// @param[in] base Pointer to the start of the memory block
    /// @param[in] size Size of the memory block
    membuf(const char* base, size_t size)
    {
        char* p{ const_cast<char*>(base) }; // NOLINT(cppcoreguidelines-pro-type-const-cast)
        this->setg(p, p, p + size);
    }
    /// @brief Destructor
    ~membuf() override = default;
    /// @brief Copy constructor
    membuf(const membuf&) = default;
    /// @brief Move constructor
    membuf(membuf&&) = default;
    /// @brief Copy assignment operator
    membuf& operator=(const membuf&) = default;
    /// @brief Move assignment operator
    membuf& operator=(membuf&&) = default;
};

/// Credits: https://stackoverflow.com/a/13059195
/// https://stackoverflow.com/questions/13059091/
struct memstream : virtual membuf, std::istream
{
    /// @brief Constructor
    /// @param[in] base Pointer to the start of the memory block
    /// @param[in] end Pointer to the end of the memory block
    memstream(char const* base, char* const end)
        : membuf(base, reinterpret_cast<uintptr_t>(end) - reinterpret_cast<uintptr_t>(base)),
          std::istream(static_cast<std::streambuf*>(this)) {}

    /// @brief Constructor
    /// @param[in] base Pointer to the start of the memory block
    /// @param[in] size Size of the memory block
    memstream(char const* base, size_t size)
        : membuf(base, size),
          std::istream(static_cast<std::streambuf*>(this)) {}
};
} // namespace cmrc

/* Example Usage: https://caiorss.github.io/C-Cpp-Notes/resources-executable.html#org7635e6a

#define SHOW_EXPR(expr) \
   std::cout << " =>> EXPR( " << #expr << " ) = " << (expr) << std::endl

int main(int argc, const char** argv)
{
    auto fs = cmrc::app1::get_filesystem();

    std::cout << std::boolalpha;

    std::puts("\n ========= Experiment 1 ==>> Check resources ==========");

    SHOW_EXPR( fs.is_file("resources/protocols.txt") );
    SHOW_EXPR( fs.is_file("resources/hosts.txt") );
    SHOW_EXPR( fs.is_file("/resources/hosts.txt") );
    SHOW_EXPR( fs.is_file("resources/protoc.txt") );

    std::puts("\n ========= Experiment 2 ==>> Read resource ==========");
    {
        auto fd1 = fs.open("resources/hosts.txt");

        // Read whole file content to string
        auto st1 = std::string(fd1.begin(), fd1.end());

        std::cout << "\n Content of resources/hosts = \n" << st1 << std::endl;
    }

    std::puts("\n ========= Experiment 3 ===>> Read resource as stream ===");
    {

        auto fd2 = fs.open("resources/protocols.txt");

        auto is = memstream ( const_cast<char*>(fd2.begin()),
                              const_cast<char*>(fd2.end()) );

        std::string line;
        int  n = 0;
        while(std::getline(is, line) && n < 25)
            std::cout << " line[" << n++ << "] = " << line << std::endl;

    }

    std::puts("\n ========= Experiment 4 ===>> Extract resource to file ===");
    {
        auto fd = fs.open("resources/terminal.jpeg");

        auto is = memstream ( const_cast<char*>(fd.begin()),
                              const_cast<char*>(fd.end()) );


        auto ofs = std::ofstream("/tmp/picture.jpeg");
        // Redirect is to ofs
        ofs << is.rdbuf();
        // Force writing
        ofs.flush();
    }

    return 0;
}
*/