include_directories(/home/eddie/catkin_ws/devel/include/ /tmp/epuck-ros/build/tmp/sysroots/overo/opt/ros/indigo/include)
add_executable(argos_ros_epuck_foraging argos_ros_epuck_foraging.h argos_ros_epuck_foraging.cpp RabBuffer.h RabBuffer.cpp)
target_link_libraries(argos_ros_epuck_foraging
  /tmp/epuck-ros/build/tmp/sysroots/overo/usr/lib/argos3/libargos3core_epuck.so
  /tmp/epuck-ros/build/tmp/sysroots/overo/usr/lib/argos3/libargos3plugin_epuck_epuck.so
  /tmp/epuck-ros/build/tmp/sysroots/overo/usr/lib/argos3/libargos3plugin_epuck_genericrobot.so
  /tmp/epuck-ros/build/tmp/sysroots/overo/opt/ros/indigo/lib/libroscpp.so
  /tmp/epuck-ros/build/tmp/sysroots/overo/opt/ros/indigo/lib/libroscpp_serialization.so
  /tmp/epuck-ros/build/tmp/sysroots/overo/opt/ros/indigo/lib/librostime.so	
  /tmp/epuck-ros/build/tmp/sysroots/overo/opt/ros/indigo/lib/librosconsole.so
  /tmp/epuck-ros/build/tmp/sysroots/overo/usr/lib/libboost_system.so)


