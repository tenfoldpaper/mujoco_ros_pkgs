find_package(rostest REQUIRED)

add_rostest_gtest(mujoco_ros_laser_test
  launch/mujoco_ros_laser.test
  mujoco_ros_laser_test.cpp
)

add_dependencies(mujoco_ros_laser_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_ros_laser_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# install(FILES
#   empty_world.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
# )
