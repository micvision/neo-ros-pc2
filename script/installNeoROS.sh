#!/bin/bash

# install the neo-ros-pc2 package

CATKIN_WS="$1"

if [ -e "$CATKIN_WS" ]; then
    echo "Install Neo ros package into $CATKIN_WS"
else
    echo "$CATKIN_WS does not exist."
    echo "Please select an existing initialized Catkin Workspace."
    exit 1
fi

cd "$CATKIN_WS"

echo "Installing begin..."
source devel/setup.bash
cd src
git clone https://github.com/micvision/neo-ros-pc2.git
cd neo-ros-pc2
rosdep install -a -y -r
cd ../..
catkin_make
