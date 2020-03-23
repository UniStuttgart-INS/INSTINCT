message(STATUS "Building Tests")

file(GLOB_RECURSE SRCFILES    
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# this is the file I want to exclude / remove from the list
get_filename_component(full_path_test_cpp ${CMAKE_SOURCE_DIR}/src/main.cpp ABSOLUTE)
message("${full_path_test_cpp}")

list(REMOVE_ITEM SRCFILES "${full_path_test_cpp}")
message("${SRCFILES}")

include(GoogleTest)
add_executable(tests main.cpp ${SRCFILES})

target_link_libraries(tests PRIVATE project_options project_warnings
                                    CONAN_PKG::fmt CONAN_PKG::spdlog CONAN_PKG::gtest
                                    CONAN_PKG::boost CONAN_PKG::eigen
                                    Threads::Threads
                                    libvncxx libubcxx)