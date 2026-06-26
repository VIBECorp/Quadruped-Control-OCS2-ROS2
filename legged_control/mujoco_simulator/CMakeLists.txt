cmake_minimum_required(VERSION 3.8)
project(mujoco_simulator)

set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco/mujoco-3.2.2)
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARIES ${MUJOCO_DIR}/lib/libmujoco.so)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${MUJOCO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mujoco_simulator src/SimulatorNode.cpp src/MujocoSimulation.cpp)
ament_target_dependencies(mujoco_simulator rclcpp ament_index_cpp std_msgs legged_msgs ocs2_core Boost) #ros2 link library
target_link_libraries(mujoco_simulator ${MUJOCO_LIBRARIES} glfw) #cmake link library

# libmujoco.so.3.2.2 lives in the source tree (not a colcon package), so bake its
# directory into the installed binary's RPATH; otherwise it is not found at runtime.
set_target_properties(mujoco_simulator PROPERTIES
  INSTALL_RPATH "${MUJOCO_DIR}/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS mujoco_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)


ament_package()
