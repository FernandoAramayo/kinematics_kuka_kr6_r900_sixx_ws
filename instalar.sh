#!/usr/bin/env bash
set -eo pipefail

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: ROS 2 Jazzy no está instalado en /opt/ros/jazzy."
  exit 1
fi

source /opt/ros/jazzy/setup.bash

echo "1. Instalando dependencias del sistema..."
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-urdf \
  ros-jazzy-urdfdom \
  ros-jazzy-urdf-tutorial \
  ros-jazzy-rmw-cyclonedds-cpp \
  liburdfdom-tools

if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update || true

WS=$(pwd)

echo "2. Recuperando dependencias externas (KUKA) usando vcs..."
mkdir -p "$WS/src"
vcs import "$WS/src" < dependencias.repos

echo "3. Instalando dependencias de ROS (rosdep)..."
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true

echo "4. Compilando el workspace..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --symlink-install

echo
echo "INSTALACIÓN COMPLETA - KUKA KR 6 R900 sixx"
echo "Para cargar el entorno en esta terminal, ejecuta:"
echo "source entorno.sh"