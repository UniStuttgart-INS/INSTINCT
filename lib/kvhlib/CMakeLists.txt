cmake_minimum_required(VERSION 3.9)
project(libkvhcxx CXX)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

set(SOURCE
    src/protocol/packet.cpp
    src/protocol/packetfinder.cpp
    src/sensors/sensors.cpp
    src/util/checksum.cpp
    src/util/utilities.cpp
    src/xplat/criticalsection.cpp
    src/xplat/event.cpp
    src/xplat/port.cpp
    src/xplat/serialport.cpp
    src/xplat/thread.cpp
    src/xplat/ubtime.cpp
    include/kvh/protocol/packet.hpp
    include/kvh/protocol/packetfinder.hpp
    include/kvh/sensors/sensors.hpp
    include/kvh/util/checksum.hpp
    include/kvh/util/nocopy.hpp
    include/kvh/util/utilities.hpp
    include/kvh/xplat/criticalsection.hpp
    include/kvh/xplat/event.hpp
    include/kvh/xplat/port.hpp
    include/kvh/xplat/serialport.hpp
    include/kvh/xplat/thread.hpp
    include/kvh/xplat/ubtime.hpp)

include_directories(include)

add_library(libkvhcxx ${SOURCE})
