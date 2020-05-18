# Typically you don't care so much for a third party library's tests to be
# run from your own project's code.
set(JSON_BuildTests OFF CACHE INTERNAL "")

set(sim_epuck_loop_functions_SOURCES sim_epuck_loop_functions_foraging.cpp)
# include_directories(/home/eddie/catkin_ws/src/argos_ros_epuck_foraging/plugin/argos3-arena/src/plugins/robots/arena/simulator)
# link_directories("/home/eddie/catkin_ws/src/argos_ros_epuck_foraging/plugin/argos3-arena/build/plugins/robots/arena")
# set(ARENA_SOURCES /home/eddie/catkin_ws/src/argos_ros_epuck_foraging/plugin/argos3-arena/build/plugins/robots/arena/argos3plugin_simulator_arena_automoc.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(sim_epuck_loop_functions_SOURCES
    ${sim_epuck_loop_functions_SOURCES})
    add_subdirectory(id_loop_functions_foraging)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(sim_epuck_loop_functions_foraging MODULE ${sim_epuck_loop_functions_SOURCES})

target_link_libraries(sim_epuck_loop_functions_foraging
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  /home/eddie/argos3-dist/lib/argos3/libargos_ros_epuck.so)

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(sim_epuck_loop_functions_foraging argos3plugin_simulator_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)

