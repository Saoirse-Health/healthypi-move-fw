#!/bin/sh

# Simple wrapper to build the application for a given board.
# Usage: ./build.sh [board-name]
# If no board is provided, default to Healthypi Move custom board.

BOARD=${1:-healthypi_move/nrf5340/cpuapp}

west build --build-dir app/build app --board $BOARD -- -DBOARD_ROOT=.
