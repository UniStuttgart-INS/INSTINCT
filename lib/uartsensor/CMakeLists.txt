cmake_minimum_required(VERSION 3.16)

project(libUartSensor LANGUAGES CXX)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

set(SOURCE
    src/protocol/packet.cpp
    src/protocol/packetfinder.cpp
    src/sensors/sensors.cpp
    src/util/utilities.cpp
    src/xplat/criticalsection.cpp
    src/xplat/event.cpp
    src/xplat/port.cpp
    src/xplat/serialport.cpp
    src/xplat/thread.cpp
    src/xplat/timestamp.cpp
    include/uart/protocol/packet.hpp
    include/uart/protocol/packetfinder.hpp
    include/uart/sensors/sensors.hpp
    include/uart/util/nocopy.hpp
    include/uart/util/utilities.hpp
    include/uart/xplat/criticalsection.hpp
    include/uart/xplat/event.hpp
    include/uart/xplat/port.hpp
    include/uart/xplat/serialport.hpp
    include/uart/xplat/thread.hpp
    include/uart/xplat/timestamp.hpp)

include_directories(include)

add_library(libUartSensor ${SOURCE})

target_link_libraries(libUartSensor PRIVATE project_options project_warnings)
