#!/bin/bash
set -e

echo "[+] Updating package index..."
sudo apt update

echo "[+] Installing required dependencies..."
sudo apt install -y \
    libgl1-mesa-dev libglu1-mesa-dev libegl1-mesa-dev \
    wayland-protocols libwayland-dev libxrandr-dev \
    libxinerama-dev libxcursor-dev libxi-dev libx11-dev \
    build-essential pkg-config libglew-dev \
    libglfw3-dev libjpeg-dev libpng-dev libtiff-dev \
    libopenexr-dev libeigen3-dev libdrm-dev libgbm-dev \
    libspdlog-dev libjsoncpp-dev libfreetype6-dev \
    libxkbcommon-dev libqhull-dev
