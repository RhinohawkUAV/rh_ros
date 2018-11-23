#!/bin/sh
cd ~/src/ardupilot/ArduPlane
MAP_SERVICE=MicrosoftSat sim_vehicle.py -v ArduPlane -f quadplane --console --map -L DalbyAU -S 2

