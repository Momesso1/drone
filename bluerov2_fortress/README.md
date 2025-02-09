export TURTLEBOT3_MODEL=waffle
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build
export IGN_GAZEBO_RESOURCE_PATH=~/autonomous/src/bluerov2_fortress/models:~/bluerov2/src/bluerov2_fortress/worlds
export PATH="$PATH:$HOME/.local/bin"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}
export IGN_GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}