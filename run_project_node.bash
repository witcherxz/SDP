#!/bin/bash

# Exit immediately if any command fails
set -e

# Function to restart nvargus-daemon
restart_nvargus_daemon() {
  sudo systemctl restart nvargus-daemon
}

# Function to build the localization
build() {
  catkin build localization
}

# Function to run the localization node
run() {
  rosrun localization localization_node
}

# Function to build and run localization
build_run() {
  build
  run
}

# Check if any command-line arguments were provided
if [[ $# -gt 0 ]]; then
  # Execute the corresponding action based on the provided argument
  case $1 in
    "build")
      restart_nvargus_daemon
      build
      ;;
    "run")
      restart_nvargus_daemon
      run
      ;;
    "build_run")
      restart_nvargus_daemon
      build_run
      ;;
    *)
      echo "Invalid argument. Available options: build, run, build_run"
      exit 1
      ;;
  esac
else
  # Check if the user provided input arguments
  if [[ $# -eq 0 ]]; then
    # Define the menu options
    options=("Build localization" "Run localization node" "Build and run localization" "Quit")

    # Loop until the user chooses to quit
    while true; do
      # Print the menu options
      echo "Select an option:"

      # Use the select command to display the menu options and read the user's choice
      select opt in "${options[@]}"; do
        case $opt in
          "Build localization")
            restart_nvargus_daemon
            build
            break
            ;;
          "Run localization node")
            restart_nvargus_daemon
            run
            break
            ;;
          "Build and run localization")
            restart_nvargus_daemon
            build_run
            break
            ;;
          "Quit")
            # User chose to quit
            exit 0
            ;;
          *)
            # Invalid choice, display an error message
            echo "Invalid option. Try again."
            break
            ;;
        esac
      done
    done
  fi
fi
