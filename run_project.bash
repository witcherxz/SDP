#!/bin/bash

# Check if the number of input arguments is correct
if [ $# -ne 2 ]; then
  echo "Usage: $0 <x> <y>"
  exit 1
fi

# Assign the input arguments to variables
x="$1"
y="$2"

# Check if 'localization' node is already running
if rosnode list | grep -q "localization"; then
  echo "'localization' node is already running."
else
  # Run the 'localization' command in the background and suppress output
  echo "Restarting nvargus-daemon (require sudo access)..."
  sudo systemctl restart nvargus-daemon
  echo "Starting localization node..."
  rosrun localization localization_node >/dev/null 2>&1 &
fi

if rosnode list | grep -q "objectDetection"; then
  echo "'objectDetection' node is already running."
else
  echo "Starting objectDetection node..."
  rosrun rosrun OBJ_D OBJ_D_node >/dev/null 2>&1 &
fi

# Check if 'navigation' node is already running
if rosnode list | grep -q "navigator"; then
  echo "'navigation' node is already running."
else
  # Run the 'navigation' command in the background and suppress output
  echo "Starting navigation node with x=$x and y=$y..."
  rosrun navigation navigation_node "$x" "$y" >/dev/null 2>&1 &
fi

# Add a delay to ensure that the processes have enough time to start
sleep 2
  echo "Done."
