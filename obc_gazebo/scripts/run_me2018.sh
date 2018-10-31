#!/bin/bash
DIR=$(dirname $0)
rosrun rh_autonomy load_kmz.py $DIR/../scenarios/ME2018_final.kmz
rosrun rh_autonomy start_mission.py

