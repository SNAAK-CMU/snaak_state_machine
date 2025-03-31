#!/bin/bash
# Put into Documents

echo "    _____ _   _____    ___    __ __"
echo "   / ___// | / /   |  /   |  / //_/"
echo "   \\__ \/  |/ / /| | / /| | / ,<   "
echo "  ___/ / /|  / ___ |/ ___ |/ /| |  "
echo " /____/_/ |_/_/  |_/_/  |_/_/ |_|  "

if [ "$1" == "--start-frankapy" ]; then
    ./home/snaak/Documents/frankapy/start_control_pc.sh -u snaak -i iam-doc
fi

source home/snaak/Documents/manipulation_ws/install/setup.bash
ros2 launch snaak_state_machine snaak_state_machine_launch.py