cmake_minimum_required(VERSION 3.18)

set(MDPGUI_FILES src/DefenderAttackerPriority.cpp src/DefenderGoalPriority.cpp src/main.cpp src/MaxExploitabilityPriority.cpp src/MaxImpactPriority.cpp src/MDPGUIConfig.cpp src/NetworkSimulator.cpp src/PathNetwork.cpp src/SimulationUnit.cpp src/TopologyWrapper.cpp)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()
# or -D CMAKE_BUILD_TYPE=Release

project(POMDPGUI)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

set(AS_LIBRARY True)
add_subdirectory("${CMAKE_SOURCE_DIR}/NetworkGUILibrary")

add_executable("${PROJECT_NAME}" ${MDPGUI_FILES})

target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/NetworkGUILibrary/include" "/home/kurt/Desktop/madp/include/madp")
target_link_libraries("${PROJECT_NAME}" NetworkGUILibrary)
target_compile_options(NetworkGUILibrary PRIVATE -DNDEBUG)

set (SFML_DIR "/usr/lib/x86_64-linux-gnu/cmake/SFML")
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)
target_link_libraries("${PROJECT_NAME}" sfml-graphics sfml-window sfml-system)

target_link_libraries("${PROJECT_NAME}" "/home/kurt/Desktop/madp/lib/libMADP.a")

target_compile_options("${PROJECT_NAME}" PRIVATE -Wall)
target_compile_options("${PROJECT_NAME}" PRIVATE -DNDEBUG)

ADD_CUSTOM_TARGET(res ALL COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/res ${CMAKE_CURRENT_BINARY_DIR}/res)

#if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_CURRENT_BINARY_DIR)
#    file(GLOB RESOURCES "*.json")
#    set(RESOURCES ${RESOURCES} "coolvetica rg.ttf")
#    file(COPY ${RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/res")
#endif()
