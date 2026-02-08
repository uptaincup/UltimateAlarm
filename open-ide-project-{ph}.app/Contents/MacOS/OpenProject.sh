#!/bin/bash

# Ensure the working directory is set to the project directory
PROJECT_PATH="$(cd "$(dirname "$0")" && cd .. && cd .. && cd .. && pwd)"

# Check if the project path exists (if running from the correct directory)
if [ ! -d "$PROJECT_PATH" ]; then
  echo "Error: The project directory does not exist: $PROJECT_PATH"
  exit 1
fi

# Open WebStorm with the project directory
echo "Opening WebStorm with project directory: $PROJECT_PATH"
open -a "CLion" "$PROJECT_PATH"



# "$(dirname "$0")"
