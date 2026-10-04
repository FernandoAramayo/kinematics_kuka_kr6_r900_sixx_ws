#!/usr/bin/env bash
set -eo pipefail
WS=$(pwd)
if [ ! -f "$WS/entorno.sh" ] || [ ! -f "$WS/install/setup.bash" ]; then
  echo "ERROR: el workspace no está instalado. Ejecuta primero ./instalar.sh"
  exit 1
fi
source "$WS/entorno.sh"
ros2 launch grupo03_kuka_kr6_bringup display.launch.py
