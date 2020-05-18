#!/bin/bash

if [ $# -lt 6 ]; 
   then 
   printf "Not enough arguments - %d\n" 
   printf "Compulsory parameters are:\n"
   printf "n - number or robots in the simulation\n"
   printf "t - number of different tokens in the simulation\n"
   printf "tn - density of tokens from each different color\n"
   printf "s - arena size\n"
   printf "o - output directory\n"
   printf "v - visual output (true/false)\n"
   exit 0 
   fi 

# PARAMETERS 
# The number of robots.
n=$1

# The number of different tokens
t=$2

# Density of each different color token
tn=$3

# Size of Arena side
ARENA_SIDE=$4

# output dir for each simulation
o=$5

# Visualization toggle
v=$6

ARGOS_CONF=/tmp/world.argos
ARGOS_AUX=/tmp/world.aux
ARGOS_AUX2=/tmp/world.aux2
ARGOS_AUX3=/tmp/world.aux3
ARGOS_AUX4=/tmp/world.aux4
ARGOS_AUX5=/tmp/world.aux5
ARENA_DEF=/tmp/arena.def
LOOP_DEF=/tmp/loop.def
VIZ_DEF=/tmp/viz.def
FRAMEWORK_DEF=/tmp/framework.def
ROBOT_DEF=/tmp/robot.def
PUCK_DEF=/tmp/puck.def
ARGOS_TEMPLATE=~/catkin_ws/src/argos_ros_epuck_foraging/world/world-template.argos 
RANDOM_SEED=`shuf -i 1-12500 -n 1`
OUTPUT_DIR=$o
CURRENT_DATE=$(date +%F-%H-%M-%S)
ARENA_BORDER_OFFSET=0.25
ARENA_BORDER=$(echo "scale = 2; $ARENA_SIDE - $ARENA_BORDER_OFFSET" | bc)
ARENA_CENTER=$(echo "scale = 2; $ARENA_SIDE / 2" | bc)
CELL_DIMENSION=0.5
NUMBER_OF_CELLS=$(echo "scale = 2; ($ARENA_SIDE*$ARENA_SIDE)/($CELL_DIMENSION*0.1)" | bc)
EXPERIMENT_LENGTH=21000
TICKS_PER_SECOND=10
BASE_ID_NUM=20

# VIZ DEFINITION
for ((i=0; i<1; i++)); do  
  if [[ $v -eq 1 ]];
  then
    echo -e "<visualization>"
    echo -e "  <qt-opengl>"
    echo -e "  <camera>"
    echo -e "  <placement idx=\"0\" position=\"1.28945,-0.764848,2.10011\" look_at=\"1.28996,-0.124157,1.33231\" up=\"0.000611419,0.767798,0.640691\" lens_focal_length=\"20\" /> "
    echo -e "  </camera>"
    echo -e "  <user_functions library=\"libid_epuck_loop_functions\" label=\"id_qtuser_functions\"/> "
    echo -e "  </qt-opengl>"
    echo -e "</visualization>"
  else
    echo -e "<visualization />"
  fi
  
done > $VIZ_DEF

# Insert Viz definition into the main file
awk 'FNR==NR {a[i++]=$0;next} /<!-- VIZ -->/ {print; for(i=0;i in a;i++) print a[i];next}1' $VIZ_DEF $ARGOS_TEMPLATE > $ARGOS_AUX

# FRAMEWORK DEFINITION
for ((i=0;i<1;i++)); do
  echo -e " <framework> " 
  echo -e "   <system threads=\"0\"/> "
  echo -e "   <experiment length=\"$EXPERIMENT_LENGTH\" ticks_per_second=\"$TICKS_PER_SECOND\" random_seed=\"$RANDOM_SEED\"/> "
  echo -e " </framework> "
done > $FRAMEWORK_DEF

# Insert framework definition into the main file
awk 'FNR==NR {a[i++]=$0;next} /<!-- FRAMEWORK -->/ {print; for(i=0;i in a;i++) print a[i];next}1' $FRAMEWORK_DEF $ARGOS_AUX > $ARGOS_AUX2

# LOOP FUNCTIONS
for ((i=0; i<1; i++)); do
    echo -e "  <loop_functions library=\"libsim_epuck_loop_functions_foraging\" label=\"sim_epuck_loop_functions_foraging\">"
    echo -e "  <floor_color num_of_cells=\"$NUMBER_OF_CELLS\" cell_dimension=\"$CELL_DIMENSION\" num_of_colors=\"$t\" size=\"$ARENA_SIDE,$ARENA_SIDE,1\" />"
    echo -e "  <sim  number_of_robots=\"$n\" output=\"$OUTPUT_DIR/sim_$CURRENT_DATE.dat\" "
    echo -e "  adjacency_matrix=\"$OUTPUT_DIR/adjacency_matrix_$CURRENT_DATE.json\""
    echo -e "  experiment_length=\"$EXPERIMENT_LENGTH\" "
    echo -e "  ticks_per_second=\"$TICKS_PER_SECOND\"/>"
    echo -e "  </loop_functions> "
done > $LOOP_DEF

# Insert Loop definition into the main file
awk 'FNR==NR {a[i++]=$0;next} /<!-- LOOP FUNCTIONS -->/ {print; for(i=0;i in a;i++) print a[i];next}1' $LOOP_DEF $ARGOS_AUX2 > $ARGOS_AUX3

# ROBOT DEFINITIONS 
for ((i=0; i<1; i++)); do
    echo -e "<distribute>"
    echo -e "<position method=\"uniform\" min=\"0.50, 0.50, 0\" max=\"$ARENA_BORDER,$ARENA_BORDER,0\"/>"
    echo -e "<orientation method=\"constant\" values=\"0,0,0\"/>"
    echo -e "  <entity quantity=\"$n\" max_trials=\"100\" base_num=\"$BASE_ID_NUM\">"
    echo -e "     <e-puck id=\"epuck_\" omnidirectional_camera_aperture=\"65\" >"
    echo -e "         <controller config=\"argos_ros_epuck\"/>"
    echo -e "     </e-puck>"
    echo -e "   </entity>"
    echo -e "</distribute>"
done > $ROBOT_DEF

# Insert Robot definition into the main file
awk 'FNR==NR {a[i++]=$0;next} /<!-- ROBOTS -->/ {print; for(i=0;i in a;i++) print a[i];next}1' $ROBOT_DEF $ARGOS_AUX3 > $ARGOS_AUX4

# PUCK COLOR ARRAY
declare -a color_array=("green" "red" "blue" "yellow" "magenta" "cyan" "white" "orange")

# PUCKS DEFINITIONS
for ((i=0; i<$t; i++)); do
    echo -e "<distribute>"
    echo -e "<position method=\"uniform\" min=\"$ARENA_BORDER_OFFSET,$ARENA_BORDER_OFFSET,0\" max=\"$ARENA_BORDER,$ARENA_BORDER,0\"/>"
    echo -e "<orientation method=\"constant\" values=\"0,0,0\"/>"
    echo -e "  <entity quantity=\"$tn\" max_trials=\"100\">"
    echo -e "  <cylinder id=\"${color_array[$i]}_puck\" color=\"${color_array[$i]}\" height=\"0\" radius=\"0\" mass=\"0.1\" movable=\"false\">"
    echo -e "    <body position=\"0.02,0,0\" orientation=\"0,0,0\"/>"
    echo -e "       <leds medium=\"leds\">"
    echo -e "          <led offset=\"0,0,0\" anchor=\"origin\" color=\"${color_array[$i]}\" intensity=\"1.0\"/>"
    echo -e "       </leds>"
    echo -e "   </cylinder>"
    echo -e "   </entity>"
    echo -e "</distribute>"
done > $PUCK_DEF

# Insert Token definition into the main file
awk 'FNR==NR {a[i++]=$0;next} /<!-- PUCKS -->/ {print; for(i=0;i in a;i++) print a[i];next}1' $PUCK_DEF $ARGOS_AUX4 > $ARGOS_AUX5

# ARENA DEFINITIONS
for ((i=0; i<1; i++)); do
  echo -e "<arena size=\"$ARENA_SIDE, $ARENA_SIDE, 1\" center=\"$ARENA_CENTER,$ARENA_CENTER, 0\">"
  echo -e "<floor id=\"floor\" source=\"loop_functions\" pixels_per_meter=\"50\"/>"
  echo -e "<box id=\"wall_north\" size=\"$ARENA_SIDE,0.05,0.50\" movable=\"false\">"
  echo -e "    <body position=\"$ARENA_CENTER,$ARENA_SIDE,0\" orientation=\"0,0,0\"/>"
  echo -e "</box>"
  echo -e "<box id=\"wall_south\" size=\"$ARENA_SIDE,0.05,0.50\" movable=\"false\">"
  echo -e "    <body position=\"$ARENA_CENTER,0,0\" orientation=\"0,0,0\"/> "
  echo -e "</box>"
  echo -e "<box id=\"wall_east\" size=\"0.05,$ARENA_SIDE,0.50\" movable=\"false\">"
  echo -e "    <body position=\"$ARENA_SIDE,$ARENA_CENTER,0\" orientation=\"0,0,0\"/> "
  echo -e "</box>"
  echo -e "<box id=\"wall_west\" size=\"0.05,$ARENA_SIDE,0.50\" movable=\"false\">"
  echo -e "    <body position=\"0,$ARENA_CENTER,0\" orientation=\"0,0,0\"/> "
  echo -e "</box>"
done > $ARENA_DEF

# Insert Token definition into the main file
awk 'FNR==NR {a[i++]=$0;next} /<!-- ARENA -->/ {print; for(i=0;i in a;i++) print a[i];next}1' $ARENA_DEF $ARGOS_AUX5 > $ARGOS_CONF

# ARENA DEFINITIONS
argos3 -c $ARGOS_CONF &
ARGOS_PID=$!

# Get the ROS launch file prepared
LAUNCH_FILE=/tmp/argos_ros.launch

echo "<launch>" > $LAUNCH_FILE

for ((i=$BASE_ID_NUM; i<$BASE_ID_NUM+n; i++)); do
    namespace="epuck_$i"
    echo -e "\t<group ns=\"$namespace\">"
    echo -e "\t\t<node pkg=\"argos_ros_epuck_foraging\" type=\"controller.py\" name=\"robot_controller\" output=\"screen\" />"
    echo -e "\t\t<include file=\"/home/eddie/catkin_ws/src/diff_drive/launch/demo.launch\"/>"
    echo -e "\t\t<remap from=\"/$namespace/diff_drive_go_to_goal/cmd_vel\" to=\"/$namespace/cmd_vel\"/>"
    echo -e "\t</group>"
done >> $LAUNCH_FILE

echo -e "</launch>" >> $LAUNCH_FILE

roslaunch $LAUNCH_FILE &
ROSLAUNCH_PID=$!

wait $ARGOS_PID
kill -HUP $ROSLAUNCH_PID
exit 1
