# ARAIG Test Stack
This stack contains the GUI, calculators, interpreters and runners required to implement any ARAIG test. 

The actual launch files per robot will be stored in [araig_robot_packages](https://github.com/ipa320/araig_robot_packages)

Benchmarking for these components per test will be stored in [araig_benchmarks](https://github.com/ipa320/araig_benchmarks)
## Install steps
* 1. `rosdep install --from-paths src --ignore-src`
* 2. `pip install -r requirement.txt`
* 3. `catkin_build`
## Note
* python dependencies:
    * you can use `rosdep resolve <pkg name>` to check if this pkg is defined in [rosdep/python.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml). If it can be found, it should be able to be installed by rosdep. If not, this pkg will be defined in [requirement.txt](./requirement.txt). That's why you should run `pip install -r requirement.txt`
    * There is also possible **rosdep** can find python pkg but it can't install. (ipa-kut has this issue) 
* docker image:
  * you can use `docker pull ros:melodic-ros-core` or `docker pull ros:melodic-ros-base`. **melodic-ros-core** doesn't include rosdep or other basic tool.
