cmake_minimum_required(VERSION 3.5)

project(PMRMP)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# check if Doxygen is installed
find_package(Doxygen)

if (DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target( docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

# clang-tidy target (linter & static code analysis)
add_custom_target(clang-tidy
  COMMAND CMAKE_EXPORT_COMPILE_COMMANDS=ON run-clang-tidy ${CMAKE_CURRENT_SOURCE_DIR})

# clang-format
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/example/VIP_PB.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/VIP_CBS.cpp
)

set(ALL_LIB_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Collisions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Constraints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/EllipticalSegment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Interval.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/JNI_API.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/LinearSegment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Maths.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/PVT_G.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/PVT_Obstacle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/PVT_ObstaclePt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/PVT_ObstacleSet.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/PVT_Point.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/PVT_S.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/PVT_State.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Scenario.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/ScenarioEvaluator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/ScenarioGenerator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/ScenarioObstacle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/ScenarioReader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/ScenarioUser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/SCIMP_Controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/SocketInterface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/TrajectorySegment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Utilities.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/Vehicle.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/example/XY_Point.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Collisions.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Constants.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Constraints.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Controller.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/EllipticalSegment.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Interval.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/JNI_API.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/LinearSegment.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Maths.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Path.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Planner.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/PVT_G.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/PVT_Obstacle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/PVT_ObstaclePt.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/PVT_ObstacleSet.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/PVT_Point.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/PVT_S.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/PVT_State.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Scenario.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/ScenarioEvaluator.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/ScenarioGenerator.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/ScenarioObstacle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/ScenarioReader.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/ScenarioUser.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/SCIMP_Controller.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/SocketInterface.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/TrajectorySegment.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Utilities.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/Vehicle.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/PVTP/XY_Point.hpp
)
add_library(lib_a SHARED ${ALL_LIB_FILE})

add_custom_target(clang-format
  COMMAND clang-format -i ${ALL_SOURCE_FILES}
)

# tests
add_custom_target(run-test
  COMMAND python3 -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_custom_target(everything
  DEPENDS clang-format clang-tidy docs run-test
)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Creates compile database used by clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
)

# Examples

## a_star
#add_executable(a_star
  #example/a_star.cpp
#)
#target_link_libraries(a_star
  #${Boost_LIBRARIES}
#)

## a_star_epsilon
#add_executable(a_star_epsilon
  #example/a_star_epsilon.cpp
#)
#target_link_libraries(a_star_epsilon
  #${Boost_LIBRARIES}
#)

## VIP_PB
add_executable(VIP_PB
  example/VIP_PB.cpp
)
target_link_libraries(VIP_PB lib_a)

add_executable(VIP_CBS
  example/VIP_CBS.cpp
)
target_link_libraries(VIP_CBS lib_a)




