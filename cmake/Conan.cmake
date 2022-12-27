list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

# Download automatically, you can also just copy the conan.cmake file
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake" "${CMAKE_BINARY_DIR}/conan.cmake"
        TLS_VERIFY ON
        STATUS status)

list(GET status 0 error_code)

if(error_code)
    message(STATUS "Could not download Conan Cmake file, using local backup")
    file(COPY "cmake/Conan-0.18.1/conan.cmake" DESTINATION "${CMAKE_BINARY_DIR}")
endif()
endif()

set(CONAN_SYSTEM_INCLUDES ON)

include(${CMAKE_BINARY_DIR}/conan.cmake)

# See https://github.com/conan-io/cmake-conan/blob/release/0.18/README.md for settings

conan_cmake_autodetect(settings)

conan_cmake_install(PATH_OR_REFERENCE ${CMAKE_SOURCE_DIR}
                    BUILD missing
                    REMOTE conancenter
                    SETTINGS ${settings})
