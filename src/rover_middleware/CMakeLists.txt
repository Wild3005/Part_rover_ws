cmake_minimum_required(VERSION 3.8)
project(rover_middleware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_dependencies(rclcpp OpenCV sensor_msgs image_transport cv_bridge image_transport)
ament_export_include_directories(include)

add_executable(convert_gray src/convert_gray.cpp)
add_executable(qos_convert src/qos_setting.cpp)
add_executable(qos_setting_advanced src/qos_setting_advanced.cpp)
add_executable(qos_setting_hybrid src/qos_setting_hybrid.cpp)
add_executable(open_cam src/open_cam.cpp)

target_include_directories(convert_gray PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(convert_gray PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(qos_convert PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qos_convert PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(qos_setting_advanced PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qos_setting_advanced PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(qos_setting_hybrid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(qos_setting_hybrid PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(open_cam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(open_cam PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(qos_convert 
  "image_transport"
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "image_transport"
  "cv_bridge"
)

ament_target_dependencies(qos_setting_advanced 
  "image_transport"
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "image_transport"
  "cv_bridge"
)

ament_target_dependencies(qos_setting_hybrid 
  "image_transport"
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "image_transport"
  "cv_bridge"
)

ament_target_dependencies(convert_gray 
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "image_transport"
  "cv_bridge"
)

ament_target_dependencies(open_cam 
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "image_transport"
  "cv_bridge"
)

install(TARGETS 
  convert_gray
  qos_convert
  qos_setting_advanced
  qos_setting_hybrid
  open_cam
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
