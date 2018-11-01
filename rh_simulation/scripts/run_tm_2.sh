#!/bin/bash
DIR=$(dirname $0)
rosrun rh_autonomy load_mission.py $DIR/../scenarios/tm_2.mission
rosrun rh_autonomy start_mission.py

