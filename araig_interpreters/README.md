# araig_interpreters

* ROSparam:
    [config file example](config/interpreter.yaml)
### 1. velocity interpreter
* param:
    
    read as: `/interpreter/"node_name"/max_vel`
    
* `roslaunch araig_interpreters velocity_interpreter.launch`

### 2. goal interpreter
* param:

    read as: `/interpreter/"node_name"/goal/...`
* Request

    * python >= 3.6
    * intall `pip install asyncio` since `rosdep resolve python-asyncio` didn't work

* `roslaunch araig_interpreters goal_interpreter.launch`