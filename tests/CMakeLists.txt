message(STATUS "Building Tests")

include(GoogleTest)
add_executable(tests main.cpp)

target_link_libraries(tests PRIVATE project_options project_warnings
                                    CONAN_PKG::fmt CONAN_PKG::spdlog CONAN_PKG::boost CONAN_PKG::gtest
                                    Threads::Threads)