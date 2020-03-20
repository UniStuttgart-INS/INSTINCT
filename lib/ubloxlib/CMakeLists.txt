cmake_minimum_required(VERSION 3.9)
project(libubcxx CXX)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

set(SOURCE
        src/protocol/packet.cpp
        src/protocol/packetfinder.cpp
        src/protocol/types.cpp
        src/sensors/sensors.cpp
        src/util/checksum.cpp
        src/util/utilities.cpp
        src/xplat/criticalsection.cpp
        src/xplat/event.cpp
        src/xplat/port.cpp
        src/xplat/serialport.cpp
        src/xplat/thread.cpp
        src/xplat/ubtime.cpp
        include/ub/protocol/packet.hpp
        include/ub/protocol/packetfinder.hpp
        include/ub/protocol/types.hpp
        include/ub/sensors/sensors.hpp
        include/ub/util/checksum.hpp
        include/ub/util/nocopy.hpp
        include/ub/util/utilities.hpp
        include/ub/xplat/criticalsection.hpp
        include/ub/xplat/event.hpp
        include/ub/xplat/port.hpp
        include/ub/xplat/serialport.hpp
        include/ub/xplat/thread.hpp
        include/ub/xplat/ubtime.hpp)

include_directories(
    include)

add_library(libubcxx ${SOURCE})