#!/bin/bash

set -e

sudo systemctl restart nvargus-daemon

build() {
  catkin build localization
}

run() {
  rosrun localization localization_node
}

build_run(){
  build
  run
}

# Check if any command-line arguments were provided
if [[ $# -gt 0 ]]; then
  # Parse the first argument and execute the corresponding action
  case $1 in
    "build")
      build
      ;;
    "run")
      run
      ;;
    *)
      echo "Invalid argument. Available options: build, run"
      exit 1
      ;;
  esac
else
  # Define the menu options
  options=("Build and run localization" "Run localization node" "Quit")

  # Loop until the user chooses to quit
  while true; do
    # Print the menu options
    echo "Select an option:"

    # Use the select command to display the menu options and read the user's choice
    select opt in "${options[@]}"; do
      case $opt in
        "Build and run localization")
          build_and_run_localization
          break
          ;;
        "Run localization node")
          run_localization_node
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

