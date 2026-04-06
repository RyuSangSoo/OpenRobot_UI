cmake_minimum_required(VERSION 3.10)
project(ads_control_test LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

option(BUILD_CONTROLLER_ONLY "Build controller-only application" ON)
option(USE_OSQP_EIGEN "Enable OsqpEigen dependency" OFF)
option(ENABLE_BECKHOFF_ADS "Enable Beckhoff ADS output backend" ON)
option(ENABLE_ROS2_BRIDGE "Enable ROS 2 state publisher bridge" ON)

set(BECKHOFF_ADS_ROOT "/home/drcl-rss/.ADS/ADS" CACHE PATH "Path to Beckhoff ADS repository")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-g)

find_package(ament_cmake REQUIRED)
if(ENABLE_ROS2_BRIDGE)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
endif()
find_package(RBDL REQUIRED COMPONENTS URDFREADER)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
if(USE_OSQP_EIGEN)
  find_package(OsqpEigen REQUIRED)
endif()

if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
  )
endif()

if(ENABLE_BECKHOFF_ADS)
  if(NOT EXISTS "${BECKHOFF_ADS_ROOT}/AdsLib/CMakeLists.txt")
    message(FATAL_ERROR "Beckhoff AdsLib not found under ${BECKHOFF_ADS_ROOT}")
  endif()
  add_subdirectory("${BECKHOFF_ADS_ROOT}/AdsLib" "${CMAKE_CURRENT_BINARY_DIR}/beckhoff_ads")
  target_compile_definitions(ads PUBLIC CONFIG_DEFAULT_LOGLEVEL=1)
endif()

find_library(LOCOMOTION_LIB NAMES locomotion PATHS
  "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  "/usr/local/lib"
)
if(NOT LOCOMOTION_LIB)
  message(FATAL_ERROR "locomotion library not found. Place liblocomotion.so in ./lib or /usr/local/lib")
endif()

set(CONTROLLER_CORE_SOURCES
  src/control/ControlState.cpp
  src/control/ControllerLoop.cpp
  src/control/MjuJoint.cpp
  src/control/MjuJoy.cpp
  src/control/MjuRobot.cpp
  src/control/RssController.cpp
)

add_library(controller_core STATIC ${CONTROLLER_CORE_SOURCES})
target_include_directories(controller_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/includes>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/includes/control>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/control>
    ${RBDL_INCLUDE_DIR}
    ${RBDL_URDFREADER_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(controller_core
  PUBLIC
    ${LOCOMOTION_LIB}
    ${RBDL_LIBRARY}
    ${RBDL_URDFREADER_LIBRARY}
    Threads::Threads
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    dl
)

if(ENABLE_BECKHOFF_ADS)
  add_library(ads_hwio STATIC
    src/ads/ads_motor_io.cpp
  )
  target_include_directories(ads_hwio
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/includes>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(ads_hwio PUBLIC ads)
endif()

if(USE_OSQP_EIGEN)
  target_link_libraries(controller_core PUBLIC OsqpEigen::OsqpEigen)
  target_compile_definitions(controller_core PUBLIC USE_OSQP_EIGEN=1)
endif()

if(BUILD_CONTROLLER_ONLY)
  set(CAN_RMD_SOURCES
    src/main.cpp
  )
  if(ENABLE_ROS2_BRIDGE)
    list(APPEND CAN_RMD_SOURCES
      src/ros/MjuStatePublisher.cpp
    )
  endif()

  add_executable(CAN_RMD ${CAN_RMD_SOURCES})
  target_include_directories(CAN_RMD
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/includes/control
      ${Boost_INCLUDE_DIRS}
  )
  set_target_properties(CAN_RMD PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/beckhoff_ads"
    INSTALL_RPATH "$ORIGIN/.."
  )
  target_link_libraries(CAN_RMD controller_core)
  if(ENABLE_ROS2_BRIDGE)
    ament_target_dependencies(CAN_RMD
      rclcpp
      sensor_msgs
      std_msgs
    )
    target_compile_definitions(CAN_RMD PRIVATE ADS_CONTROL_TEST_WITH_ROS2=1)
  endif()
  if(ENABLE_BECKHOFF_ADS)
    target_link_libraries(CAN_RMD ads_hwio)
    target_compile_definitions(CAN_RMD PRIVATE CAN_RMD_WITH_ADS=1)
  endif()
endif()

install(DIRECTORY includes/
  DESTINATION include
)

install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS controller_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(ENABLE_BECKHOFF_ADS)
  install(TARGETS ads
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
  )

  install(TARGETS ads_hwio
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_CONTROLLER_ONLY)
  install(TARGETS CAN_RMD
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)
ament_package()
