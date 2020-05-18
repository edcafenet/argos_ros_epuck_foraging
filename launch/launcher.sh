#!/bin/bash
# PARAMETERS 

BASEDIR=/home/eddie/catkin_ws/src/argos_ros_epuck_foraging/data

NUMBEROFREPETITIONS=(100)
NUMBEROFROBOTS=(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16)
NUMBEROFTOKENS=(2 4 5 6 7 8)
DENSITYOFTOKENS=(1)
ARENASIZE=(2.5)

for ((i=0;i<$NUMBEROFREPETITIONS;i++));
do
    for robot_number in "${NUMBEROFROBOTS[@]}"
    do
        for token_number in "${NUMBEROFTOKENS[@]}"
        do
            export MERKLE_LEAFS=$token_number

            for token_density in "${DENSITYOFTOKENS[@]}"
            do
                CURRENT_DIR=$BASEDIR/R$robot_number/T$token_number/D$token_density/SIM$i/

                if [ ! -d "$CURRENT_DIR" ]; then
                    mkdir -p $CURRENT_DIR
                    ./experiment.sh $robot_number $token_number $token_density $ARENASIZE $CURRENT_DIR 0
                    sleep 10
                fi
            done
        done
    done
done

pkill argos3
