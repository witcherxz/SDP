#!/bin/bash

# set -e
# set -o pipefail

# Function to start roscore
start_roscore() {
  roscore >/dev/null 2>&1 &
  sleep 2
}

# Function to start RosAria
start_rosaria() {
  sudo chmod 777 /dev/ttyUSB0
  rosrun rosaria RosAria >/dev/null 2>&1 &
  sleep 2
  rostopic pub /RosAria/motors_state std_msgs/Bool "data: false"  >/dev/null 2>&1 &
  rostopic pub /RosAria/motors_state std_msgs/Bool 'data: true'  >/dev/null 2>&1 &
}

# Check if the number of input arguments is correct
if [ $# -ne 1 ]; then
  echo "Usage: $0 <run|kill>"
  exit 1
fi

# Assign the input argument to a variable
action="$1"

if [ "$action" == "run" ]; then
  

  # Check if roscore is already running
  roscore_pid=$(pgrep -f roscore)
  if [ -z "$roscore_pid" ]; then
    echo "Starting roscore..."
    start_roscore
  else
    echo "roscore is already running."
  fi

  if rosnode list | grep -q "RosAria"; then
    echo "RosAria is already running."
  else
    echo "Starting RosAria..."
      start_rosaria
  fi

elif [ "$action" == "kill" ]; then
  if rosnode list | grep -q "/navigator"; then
      rosnode kill /navigator
      echo "Stopped navigation node."
  fi
  rosnode kill /navigator
  if rosnode list | grep -q "/localization"; then
    rosnode kill /localization
    echo "Stopped localization node."
  fi

  # Stop RosAria
  killall -SIGINT RosAria >/dev/null 2>&1
  echo "Stopped RosAria."

  # Stop roscore
  killall -SIGINT roscore >/dev/null 2>&1
  echo "Stopped roscore."

else
  echo "Invalid action. Please use 'run' or 'kill'."
  exit 1
fi
