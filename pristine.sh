#!/bin/sh

# Build the application with a pristine build directory for a given board.
# Usage: ./pristine.sh [board-name]
# If no board is provided, default to Healthypi Move custom board.

BOARD=${1:-healthypi_move/nrf5340/cpuapp}

west build --build-dir app/build app --pristine --board $BOARD -- -DBOARD_ROOT=.
