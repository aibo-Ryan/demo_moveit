# demo_moveit
demo for the MoveIt2 manipulation
## 1. prerequisite
ubuntu 20.04
ros foxy
## 2. install
```
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone https://github.com/aibo-Ryan/demo_moveit.git
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select demo_moveit
```
## 3. run
+ run moveit2_tutorials launch
if not installed moveit2_tutorials, please refer to the article in here.
+ run demo_moveit launch

```
source install/setup.bash
ros2 launch demo_moveit demo_moveit.launch.py
```
