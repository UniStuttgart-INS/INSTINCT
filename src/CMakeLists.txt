message(STATUS "Building ${CMAKE_PROJECT_NAME}")

file(GLOB_RECURSE SRCFILES    
    "*.cpp"
)

add_executable(${PROJECT_NAME_LOWERCASE}
                    ${SRCFILES})

target_link_libraries(${PROJECT_NAME_LOWERCASE}
                PRIVATE project_options project_warnings
                        Threads::Threads spdlog docopt
                        libvncxx libubcxx)