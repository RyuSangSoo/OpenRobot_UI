cmake_minimum_required(VERSION 3.8)
project(ui_ads_ver)

set(OpenGL_GL_PREFERENCE GLVND)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)

if(DEFINED ENV{MUJOCO_PATH} AND NOT "$ENV{MUJOCO_PATH}" STREQUAL "")
  set(MUJOCO_PATH "$ENV{MUJOCO_PATH}")
else()
  set(MUJOCO_PATH "$ENV{HOME}/.mujoco/mujoco-3.5.0")
endif()

set(HAVE_MUJOCO OFF)
if(EXISTS "${MUJOCO_PATH}/include/mujoco/mujoco.h" AND EXISTS "${MUJOCO_PATH}/lib/libmujoco.so")
  set(HAVE_MUJOCO ON)
endif()

if(DEFINED ENV{IMGUI_PATH} AND NOT "$ENV{IMGUI_PATH}" STREQUAL "")
  set(IMGUI_PATH "$ENV{IMGUI_PATH}")
else()
  set(IMGUI_PATH "$ENV{HOME}/.ImGUI/imgui")
endif()

if(NOT EXISTS "${IMGUI_PATH}/imgui.h")
  message(FATAL_ERROR "ImGui not found. Set IMGUI_PATH or install to ~/.ImGUI/imgui")
endif()

add_executable(ads_state_ui
  src/main.cpp
  src/ads_state_ui.cpp
  ${IMGUI_PATH}/imgui.cpp
  ${IMGUI_PATH}/imgui_draw.cpp
  ${IMGUI_PATH}/imgui_tables.cpp
  ${IMGUI_PATH}/imgui_widgets.cpp
  ${IMGUI_PATH}/backends/imgui_impl_glfw.cpp
  ${IMGUI_PATH}/backends/imgui_impl_opengl2.cpp
)

target_compile_features(ads_state_ui PRIVATE cxx_std_17)

target_include_directories(ads_state_ui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/includes
  ${IMGUI_PATH}
  ${IMGUI_PATH}/backends
)

if(HAVE_MUJOCO)
  target_include_directories(ads_state_ui PRIVATE
    ${MUJOCO_PATH}/include
  )
endif()

target_link_libraries(ads_state_ui
  OpenGL::GL
  PkgConfig::GLFW
)

if(HAVE_MUJOCO)
  target_link_libraries(ads_state_ui
    ${MUJOCO_PATH}/lib/libmujoco.so
  )
  target_compile_definitions(ads_state_ui PRIVATE UI_ADS_VER_HAVE_MUJOCO=1)
  set_target_properties(ads_state_ui PROPERTIES
    BUILD_RPATH "${MUJOCO_PATH}/lib"
    INSTALL_RPATH "${MUJOCO_PATH}/lib"
  )
else()
  message(WARNING "MuJoCo not found at ${MUJOCO_PATH}; Extra tab viewer will be disabled")
  target_compile_definitions(ads_state_ui PRIVATE UI_ADS_VER_HAVE_MUJOCO=0)
endif()

ament_target_dependencies(ads_state_ui
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  ads_state_ui
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
