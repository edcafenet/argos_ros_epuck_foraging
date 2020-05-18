add_library(argos_ros_epuck_foraging MODULE argos_ros_epuck_foraging.h argos_ros_epuck_foraging.cpp RabBuffer.cpp RabBuffer.h)
target_link_libraries(argos_ros_epuck_foraging
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)
