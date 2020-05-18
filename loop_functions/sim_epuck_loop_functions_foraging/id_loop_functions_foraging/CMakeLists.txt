add_library(id_epuck_loop_functions_foraging MODULE 
  id_qtuser_functions.h
  id_qtuser_functions.cpp)

target_link_libraries(id_epuck_loop_functions_foraging
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
