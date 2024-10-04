cmake_minimum_required(VERSION 3.8)

project(sim)

find_package(Eigen3 REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

set(ABV_SIMULATOR_SOURCES
    main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/SimulatedVehicle.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Controller.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/VehicleVisualizer.cpp
)

add_executable(${PROJECT_NAME} ${ABV_SIMULATOR_SOURCES})

set(ABV_SIMULATOR_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${EIGEN3_INCLUDE_DIR}    
)

target_include_directories(${PROJECT_NAME} PRIVATE ${ABV_SIMULATOR_INCLUDE_DIRS})

set(ABV_SIMULATOR_LIBRARIES
    ${EIGEN3_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${ABV_SIMULATOR_LIBRARIES} sfml-graphics sfml-window sfml-system)

