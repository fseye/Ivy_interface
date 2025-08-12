## installation

'''bash

git clone https://github.com/fseye/Ivy_interface.git

cd Ivy_interface

rosdep update

rosdep install --from-paths src --ignore-src -r -c

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
