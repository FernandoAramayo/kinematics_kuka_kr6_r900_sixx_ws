#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WS=$(pwd)
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
else
  echo "Advertencia: No se encontró install/setup.bash. ¿Ya compilaste el workspace?"
fi
