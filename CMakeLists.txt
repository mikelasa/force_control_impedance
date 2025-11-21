cmake_minimum_required(VERSION 2.8.3)
project(force_control_impedance)

add_definitions(-std=c++17)

find_package(Eigen3 3.4 REQUIRED)
find_package(Franka CONFIG REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  include
)

# Add local libfranka include/lib locations (installed to $HOME/.local)
include_directories($ENV{HOME}/.local/include)
link_directories($ENV{HOME}/.local/lib)

# try to find installed franka library
find_library(FRANKA_LIB franka HINTS $ENV{HOME}/.local/lib /usr/local/lib /usr/lib)

find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

add_library(FORCE_CONTROLLERS SHARED src/admittance_controller.cpp src/config_deserialize.cpp src/impedance_controller.cpp src/config_deserialize_impedance.cpp)
  target_link_libraries(FORCE_CONTROLLERS ${RUT} ${YAMLLib} ${FRANKA_LIB})

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)

install(TARGETS FORCE_CONTROLLERS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

target_include_directories(FORCE_CONTROLLERS
        PRIVATE
            ${CMAKE_SOURCE_DIR}/../cpplibrary/include
            ${CMAKE_SOURCE_DIR}/../yaml-cpp/include
        )