cmake_minimum_required(VERSION 3.8)
project(line_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(line_follower_interfaces REQUIRED)

# camera image processing (black & white -> binary image)
add_executable(camera_opencv_lf src/camera_sub.cpp)
ament_target_dependencies(camera_opencv_lf rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

# take binary image and compute and histogram
add_executable(sub_bw_image src/sub_camera_bw.cpp)
ament_target_dependencies(sub_bw_image rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV line_follower_interfaces)

# take binary image and compute and histogram
add_executable(follow_line src/follow_line.cpp)
ament_target_dependencies(follow_line rclcpp std_msgs sensor_msgs line_follower_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  camera_opencv_lf
  sub_bw_image
  follow_line
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
