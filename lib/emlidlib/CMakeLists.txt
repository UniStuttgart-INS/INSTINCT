cmake_minimum_required(VERSION 3.9)
project(libercxx CXX)

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
    src/xplat/ertime.cpp
    include/er/protocol/packet.hpp
    include/er/protocol/packetfinder.hpp
    include/er/sensors/sensors.hpp
    include/er/util/checksum.hpp
    include/er/util/nocopy.hpp
    include/er/util/utilities.hpp
    include/er/xplat/criticalsection.hpp
    include/er/xplat/event.hpp
    include/er/xplat/port.hpp
    include/er/xplat/serialport.hpp
    include/er/xplat/thread.hpp
    include/er/xplat/ertime.hpp)

include_directories(include)

add_library(libercxx ${SOURCE})
