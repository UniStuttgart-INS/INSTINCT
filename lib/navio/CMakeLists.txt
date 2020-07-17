cmake_minimum_required(VERSION 3.7)

project(libnavio LANGUAGES CXX)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

# Search all .cpp files file(GLOB_RECURSE SOURCE_FILES "*.cpp")

set(SOURCE_FILES
    src/Common/gpio.cpp
    src/Common/I2Cdev.cpp
    src/Common/MPU9250.cpp
    src/Common/MS5611.cpp
    src/Common/Ublox.cpp
    src/Common/Util.cpp
    src/Navio2/ADC_Navio2.cpp
    src/Navio2/Led_Navio2.cpp
    src/Navio2/LSM9DS1.cpp
    src/Navio2/PWM.cpp
    src/Navio2/RCInput_Navio2.cpp
    src/Navio2/RCOutput_Navio2.cpp
    src/Navio2/RGBled.cpp
    include/navio/Common/ADC.h
    include/navio/Common/gpio.h
    include/navio/Common/I2Cdev.h
    include/navio/Common/InertialSensor.h
    include/navio/Common/Led.h
    include/navio/Common/MPU9250.h
    include/navio/Common/MS5611.h
    include/navio/Common/RCInput.h
    include/navio/Common/RCOutput.h
    include/navio/Common/SPIdev.h
    include/navio/Common/Ublox.h
    include/navio/Common/Util.h
    include/navio/Navio2/ADC_Navio2.h
    include/navio/Navio2/Led_Navio2.h
    include/navio/Navio2/LSM9DS1.h
    include/navio/Navio2/PWM.h
    include/navio/Navio2/RCInput_Navio2.h
    include/navio/Navio2/RCOutput_Navio2.h
    include/navio/Navio2/RGBled.h)

include_directories(include)

add_library(libnavio ${SOURCE_FILES})
