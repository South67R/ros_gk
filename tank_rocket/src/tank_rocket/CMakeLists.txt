cmake_minimum_required(VERSION 3.0.2)
project(tank_rocket)

# Tìm gói catkin và các thành phần phụ thuộc
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  geometry_msgs
  sensor_msgs
  tf2
)

# Tìm gói Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES tank_rocket
  CATKIN_DEPENDS roscpp gazebo_ros geometry_msgs sensor_msgs tf2 
  INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES ${GAZEBO_LIBRARIES}
)

# Thêm các thư mục include từ catkin và Gazebo
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Liên kết đến thư mục thư viện của Gazebo
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Tạo thư viện plugin
add_library(mecanum_drive_plugin SHARED src/mecanum_drive_plugin.cpp)
target_link_libraries(mecanum_drive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Tạo executable cho tank_rocket_control
add_executable(tank_rocket_control src/tank_rocket_control.cpp)
target_link_libraries(tank_rocket_control ${catkin_LIBRARIES})
set_target_properties(tank_rocket_control PROPERTIES OUTPUT_NAME tank_rocket_control PREFIX "")
add_dependencies(tank_rocket_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
