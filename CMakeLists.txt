cmake_minimum_required(VERSION 3.5)
project(mike_av_stack)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# -g allows for gdb debugging
# turn on -03 for best performance
# add_definitions(-std=c++14 -O3)
# set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wextra")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(Eigen3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(localization scripts/localization/localization.cpp)
add_executable(helper scripts/localization/helper.cpp)
add_executable(scan_matching scripts/localization/scan_matching.cpp)
add_executable(ndt scripts/localization/ndt.cpp)
add_executable(icp scripts/localization/icp.cpp)
add_executable(icps scripts/localization/icps.cpp)
ament_target_dependencies(localization rclcpp std_msgs sensor_msgs pcl_ros tf2 tf2_geometry_msgs)
ament_target_dependencies(helper pcl_ros tf2 tf2_geometry_msgs)
ament_target_dependencies(scan_matching rclcpp sensor_msgs pcl_ros)
ament_target_dependencies(ndt rclcpp pcl_ros pcl_conversions)
ament_target_dependencies(icp rclcpp pcl_ros pcl_conversions sensor_msgs)
ament_target_dependencies(icps rclcpp pcl_ros pcl_conversions)

install(TARGETS
  localization
  # helper
  # scan_matching
  # ndt
  # icp
  # icp
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
