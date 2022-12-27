#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
colcon test --merge-install --packages-select semantic_mapping
colcon test-result
ament_uncrustify src/semantic_mapping --reformat