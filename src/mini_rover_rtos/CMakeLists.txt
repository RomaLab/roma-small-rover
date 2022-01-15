################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(mini_rover_rtos)
add_compile_options(-std=c++14)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    dynamixel_workbench_msgs
    dynamixel_workbench_toolbox
    cmake_modules
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")




catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs dynamixel_sdk dynamixel_workbench_msgs dynamixel_workbench_toolbox cmake_modules
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)


aux_source_directory(./src  SRC_MAIN)
add_executable(wheel_motor_node src/wheel_motor_node.cpp src/wheel_motor.cpp src/multi_motor_controller.cpp)
add_dependencies(wheel_motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_motor_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

# add_executable(steer_motor_node src/steer_motor_node.cpp  src/steer_motor.cpp src/multi_motor_controller.cpp)
# add_dependencies(steer_motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(steer_motor_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

# add_executable(lift_motor_node src/lift_motor_node.cpp src/lift_motor.cpp src/multi_motor_controller.cpp)
# add_dependencies(lift_motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(lift_motor_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})


add_executable(rover_control_node  src/rover_control_node.cpp src/rover_state.cpp src/lift_motor.cpp src/steer_motor.cpp src/wheel_motor.cpp src/multi_motor_controller.cpp)
add_dependencies(rover_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rover_control_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})


# install(TARGETS wheel_motor_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
