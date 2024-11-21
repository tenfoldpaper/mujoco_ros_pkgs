cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros VERSION 0.7.0 LANGUAGES CXX)

#catkin_lint: ignore uninstalled_plugin

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not set. Defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

set(OpenGL_GL_PREFERENCE GLVND)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(ProjectOption)

option(NO_GLFW "Disable GLFW3 support and try finding OpenGL EGL or OSMESA for offscreen rendering" OFF)
option(USE_EGL "Build with EGL enabled offscreen rendering. Entails `NO_GLFW`" OFF)
option(USE_OSMESA "Build with OSMESA enabled offscreen rendering. Entails `NO_GLFW`" OFF)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    pluginlib
    mujoco_ros_msgs
    urdf
    tf2
    tf2_ros
    tf2_geometry_msgs
    image_transport
    camera_info_manager
    sensor_msgs
    dynamic_reconfigure
)

# cmake-format: off
configure_project_option(
  WARNINGS
    TARGET project_warning
  AVX
    TARGET project_option
  LINKER
    TARGET project_option
  SANITIZER
    TARGET project_option
  IPO
    DISABLE_FOR_CONFIG Debug
  # EXTRA_ARG -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_NONE
  CATKIN_ROS
    TARGET catkin_pkg
  COMPILER_CACHE
    LAUNCHER ccache
)

# Find MuJoCo
find_package(mujoco 3.2.0 REQUIRED)
find_library(GLFW libglfw.so.3) # Find GLFW3 for GUI


if (NO_GLFW OR USE_EGL OR USE_OSMESA OR ${GLFW} STREQUAL "GLFW-NOTFOUND")
  message(WARNING "GLFW3 not found or disabled. GUI will not be available.")

  find_package(OpenGL COMPONENTS OpenGL EGL) # Find OpenGL (EGL) for offscreen rendering
  if (USE_OSMESA OR ${OpenGL_EGL_FOUND} STREQUAL "FALSE")
    message(WARNING "EGL not found or disabled. Falling back to OSMESA.")

    find_package(OSMesa)

    if (!OSMesa_FOUND)
      message(WARNING "EGL disabled or not found and OSMesa could not be found. Offscreen rendering will not be available!")
    else() # OSMesa found
      set(RENDERING_BACKEND "USE_OSMESA")
      message(STATUS "OSMesa found. Offscreen rendering available.")
    endif()

  else() # EGL found
    set(RENDERING_BACKEND "USE_EGL")
    message(STATUS "EGL found. Offscreen rendering available.")
  endif()

else() # GLFW found
  set(RENDERING_BACKEND "USE_GLFW")
  message(STATUS "GLFW3 found. GUI and offscreen rendering available.")
endif()


# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

## Dynamic Reconfigure
generate_dynamic_reconfigure_options(
  cfg/SimParams.cfg
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    roscpp
    actionlib
    pluginlib
    mujoco_ros_msgs
    urdf
    tf2
    tf2_ros
    tf2_geometry_msgs
    image_transport
    camera_info_manager
    sensor_msgs
    dynamic_reconfigure
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    mujoco
)

add_subdirectory(src)

# Depend on gencfg to ensure build before lib
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# ############
# # Install ##
# ############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} mujoco_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# #############
# ## Testing ##
# #############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
