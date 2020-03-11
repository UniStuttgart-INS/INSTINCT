message(STATUS "Building Tests")

include(GoogleTest)
add_executable(tests main.cpp)

target_link_libraries(tests PRIVATE project_options project_warnings
                                    gtest gtest_main
                                    Threads::Threads spdlog)