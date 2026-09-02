#!/usr/bin/env bash
# Idempotent Cloud Agent setup for the ScaleAISShape OpenCASCADE C++ project.
set -euo pipefail

sudo apt-get update

# System toolchain + OpenCASCADE (OCCT) modules the sources compile and link against.
# The *-dev graphics packages provide the .so symlinks OCCT's imported CMake
# targets require (TBB, fontconfig, freetype, X11, GL).
sudo apt-get install -y --no-install-recommends \
    cmake \
    g++ \
    make \
    libocct-foundation-dev \
    libocct-modeling-data-dev \
    libocct-modeling-algorithms-dev \
    libocct-visualization-dev \
    libtbb-dev \
    libfontconfig-dev \
    libfreetype-dev \
    libx11-dev \
    libxext-dev \
    libgl-dev \
    libglu1-mesa-dev

# The distro's default `c++` alternative points at Clang, which cannot find
# libstdc++ here, so pin the build to g++.
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build -j"$(nproc)"
