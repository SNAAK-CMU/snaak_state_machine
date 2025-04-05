#!/bin/bash
# Put into Documents

echo "    _____ _   _____    ___    __ __"
echo "   / ___// | / /   |  /   |  / //_/"
echo "   \\__ \/  |/ / /| | / /| | / ,<   "
echo "  ___/ / /|  / ___ |/ ___ |/ /| |  "
echo " /____/_/ |_/_/  |_/_/  |_/_/ |_|  "

if [ "$1" == "--start-frankapy" ]; then
    ./frankapy/bash_scripts/start_control_pc.sh -u snaak -i iam-doc
fi

source manipulation_ws/install/setup.bash

gnome-terminal -- bash -c "ros2 launch snaak_state_machine_dependencies_launch.py; exec bash"
gnome-terminal -- bash -c "ros2 run snaak_state_machine snaak_state_machine_main; exec bash"
