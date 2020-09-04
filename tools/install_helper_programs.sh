#!/bin/bash

set -e # Exit if any command fails

PACKAGES=(
    # Tells you cpu temps (run `sensors`)
    lm-sensors

    # Cool cpu usage monitor
    htop
)

sudo apt update # Make sure package list is up to date
sudo apt install ${PACKAGES[@]} # Install all packages
