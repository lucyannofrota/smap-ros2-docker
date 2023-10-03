#!/bin/bash
echo "---<setup.bash>---"
set -e

rm -r -f build install log | sed 's/^/  /'

source /opt/ros/foxy/setup.bash | sed 's/^/  /'

sudo apt-get update | sed 's/^/  /'
rosdep update | sed 's/^/  /'
rosdep install --from-paths src/smap --ignore-src -y | sed 's/^/  /'

source /opt/ros/foxy/setup.bash | sed 's/^/  /'
echo "---</setup.bash>---"