# ARAIG Test Descriptions

This repo holds the description YAML file for the ARAIG tests.
These serve as models based on which further tooling can autogenerate the test packages.
The test packages shall including the launch points for the tests and the calculators.

## <a id="structure"> Structure </a>

The test description for each test is split up into a few different files

1. Interfaces _(testX_interfaces.yaml)_ : File to define all the Semantic Definitions and their implemented topic names and types. This serves as a central list that is used by all the other files. See [below](#interfaces) for more info. Tags within this section include:
    * External | Internal - Define if the interfaces are from external devices or for internal nodes
    * Data | Signal- Define if the interface type is data or signal
    * name, topic, type, description - Semantic Names and their bound topics, topic msg type and a description text.
2. Calculators _(testX_calculators.yaml)_: Defines which calculators are used and how their interfaces are mapped. See [below](#calculators) for more info. Tags within this section include:
    * type - The type of the calculator class that is used. Based on the calculator classes created for the test stack
    * name - The name given to this instance of the calculator
    * description - Just a descriptive text as to what each calculator instance does
    * Inputs | Outputs - List the subscribed and published interfaces by semantic definition.
    * Parameters - These parameter names are loaded from a separate parameter yaml file. See the bringup section to see which file is to be loaded. Names and descriptions of the parameters to go within that param yaml file.
    * Data | Signals : Groups the input and output interfaces by their types.
3. Bringup _(testX_bringup.yaml)_ : File that will define what the test launch file should contain. Tags within this section include:
    * Rosparam | Launch | Node : The different types of elements to be created within the launch file. Note that rosparams are global params
    * pkg - The package of the included launch or node
    * launch - the name of the launch file
    * node - the name of the node
    * args - args for the included launch or node
4. Runner description _(readme.md)_ : This markdown file will explain textually how the runner should be written. The actual code for this will be maintained in a different package.


## Some key points about this model

### Noteworthy components of the architecture

1. **Calculators**: Based on HSD's [observers](https://github.com/ipa-hsd/rosgraph_monitor/tree/master/src/rosgraph_monitor/observers). Form the fundamental calculation units of this test stack. Each calculator instance does one and only one atomic function. They are combined in multiple layers to generate the overall test results. These will exist in a Template -> Instance fashion.
    * *Templates*: One package will contain all the calculator templates specific to the test stack. These will be generic, outlining a fundamental operation with general logic that is performed over generic interfaces.
    * *Instances*: Templates will be utilized to create specific instances that have specific logic (partially handwritten) operating over well defined interfaces.
2. **Runner** : Every test will have one central runner whose job is to script the actual test, and generate the signal triggers for interpreters and calculators down the line. This runners are written in another package and included in the bringup of each test package.
3. **Interpreters** : Specially written nodes that will interpret between standard interfaces of the test stack and specific interfaces of robots.
4. **GUI** : RQT based GUI for the operator to orchestrate the test.
5. **Test Packages** : Nadia's tooling will generate specific packages for every test of every robot. These will contain the single launch file that will start up EVERYTHING required for the test, as well as auto-generate the instances of calculators.

### <a id="interfaces"> Interfaces </a>

1. These are divided into three categories:
    * `Data` : ROS topics. Streamed data that is consumed by calculators to calculate and output
    * `Signals` : ROS topics. Signals are binary output levels (True/False) which can enable or disable certain actions in the various nodes and calculators of the test package.
    * `Param` : Global ROS Parameters
    * Interfaces should be modeled as only one of these types, not combinations thereof.
2. Each topic is annotated with a model level name which is called a `Semantic Definition`. This abstracts the implementation layer from the model.
3. If the name of the topic starts with `/data/..` or its Semantic Definition starts with `Data.`, it is `Data`.
4. If the name of the topic starts with `/signals/..` or its Semantic Definition starts with `Signals.`, it is an `Signal`.
5. Params are declared inside a testX_params.yaml, which is a rosparam syntax yaml file. These are all loaded up by the launch file in the global namespace, and can be accessed by any node/calculator.

### Namespaces and Naming Conventions

1. The naming conventions are suggested as follows:
    * For the Params:  `Param.<param_nam>`. The param name should be the same as the once declared in the params file.
    * For the Data or signals:
        * Semantic Definitions: `Data|Signal.<source_lvl_1>.<?source_lvl_2>.<name>` _(Where ? means optional)_.
        * Their corresponding topics: `/<data|signal>/<source_lvl_1>/<?source_lvl_2>/<name_lvl_1>/<?name_lvl_2>`.
        * Examples:
            - `Data.robot.odom`, `/data/robot/odom` => Data from the robot's driver giving odometry.
            - `Data.robot.nav.pose`, `/data/robot/nav/pose` => Data from the robot's nav stack giving it's pose.
            - `Signal.obs.robot_has_stopped`, `/signals/obs/robot_has_stopped/output1` => Signals generated by an calculator, telling us robot has stopped
            - `Signal.runner.robot_do_start`, `/signals/runner/robot_do_start` => Signals generated by the runner, telling the robot to start
2. Some examples of the `<source>` qualifiers are:
    * `.robot.` or `/robot/` : All data that originates from the robots. All signals that directly impact the robot or originate from it.
    * `.sensors.` or `/sensors/` : All data that originates from sensors or their interpreters.
    * `.test.` or `/test/` : All signals that originate from the test itself.
    * `.obs.` or `/calculators/` : All data that is calculated by the calculators. All signals generated by calculators.
    * `ui.` or `/ui/` : Signals generated by the GUI.
    * `runner.` or `/runner/` : Signals generated by the test runners.
4. Qualifiers to Semantic Definitions may be added as they make sense (convenience, personal taste etc...), but topics should be consistent :
    * For example: The first output topic of an calculator named `time_to_stop` shall be `/data/calculators/time_to_stop/duration` but its Semantic Definition could be `Data.obs.time_to_stop`.
    * Reason being: Semantic Definitions are model level only so there is flexibility in representation based on modeler, but topics are system level, so they should be consistent.
5. These conventions need not apply to external interfaces (such as sensors, interpreters, robot etc...) which are device dependent. These topic names can be remapped here directly.

## <a id="calculators"> Description of calculators </a>

### 1. Comparator

This calculator reads a data value of type std_msgs/float64 from a topic and compares against a second value from a param. If these values are equal, it raises an signal.

* Input1: Data Topic - Input data that is compared (handle type in runtime)
* Param1: Rosparam - Value that input data is compared against
* Output1: Signal Topic - Signal when compared values are equal

> NOTE: Suggested to check if it is better to make the second value a topic instead of rosparam. [example](https://answers.ros.org/question/216036/rostopic-node-in-launch-files/)

### 2. Timer

This calculator starts a timer when the first signal topic is true and stops the timer when the second even topic is true. This duration is then published on the first output data topic and also saved to a file if name is provided as a rosparam.

* Input1: Signal topic - Start timer when true
* Input2: Signal topic - Stop timer when true
* Param1: Rosparam - IF this param is provided, the result is saved into a file of this name. Else, no file created.
* Outpu1: Data Topic - Publisher timer duration

> NOTE: Suggested to think deeper on how the timing signals and errors should be handled (what if second input true first? What if first input never goes low? Should there be an output signal topic saying when timer value is ready? etc...)

### 3. Delta

This calculator will read data from first input constantly. When second input is raised, it records the data value. When third input is raised, it records the data value again. The difference between this two values is then published on the first output and also saved to a file if name provided as a rosparam.

* Input1: Data topic - The data source whose delta is to be calculated (handle type in runtime)
* Input2: Signal topic - The rising edge of the delta calculation window. Record value of data as Val1
* Input3: Signal topic - The falling edge of the delta calculation window. Record value of data as Val2
* Param1: Rosparam : IF this param is provided, the result is saved into a file of this name. Else, no file created.
* Output1: Data topic - Publish the calculated delta = Val2 - Val1.

### 4. Logger

This calculator will take in data from a topic and write its contents to a file whose name is provided as a param. Simplest approach is to simply dump entire message packet as one unit into files as raw text.

* Input1: Data topic - Read this data
* Param1: Rosparam - Name of the file
* Output1: File - Dump data to a file

> NOTE: Check feasibility to unpack the incoming message type, extract its field, and create a structured file with rows and columns.

### 5. Plotter

This calculator will take an input data stream and plot it against time in either an RQT plot or a pyplot. It takes in 6 parameters namely the x axis name, y axis name, title, subtitle, units of y axis and resolution.

* Input1: Data topic - The data to be plotted
* Param1: Rosparam - X axis name
* Param1: Rosparam - Y axis name
* Param1: Rosparam - Title name
* Param1: Rosparam - Subtitle name
* Param1: Rosparam - units
* Param1: Rosparam - resolution

> NOTE: Check how to plot multiple inputs in the same plot.

### 6. Rosbagger

This calculator will record rosbags starting when the first input signal is raised till the second input is raised. The parameters will decide the name of the output file, the topics to record, and a recording offset if any. The offset will wait soma additional seconds before stopping recording.

* Input1: Signal topic - When to start recording
* Input2: Signal topic - When to stop recording
* Param1: Rosparam - Output file name
* Param2: Rosparam - List of topics to record
* Param3: Rosparam - Recording end offset duration. Once the stop record signal is raised, these many extra seconds are delayed before recording actually stops.

### 7. PoseComparator

This calculator compares two poses with respect to thresholds, and declares if they are within the threshold or not. It should be flexible in its inputs. It compares two poses where one comes from an input topic, and the second from either another topic or read as a param. If the poses are within the threshold, the output signal is raised, else it is lowered.

* Input1: Data topic - First pose to compare
* Input2: Data topic OR Rosparam - Second pose to compare
* Param1/2: Rosparam - Thresholds of comparison. A list of two elements, first for position and the second for orientation (quaternion)
* Output1 : Signal topic - High if poses within threshold, low if not

### 8. ANDGate

This calculator function as a simple Boolean AND gate for signal topics. Should handle any number of inputs >2.

* Input1 - Signal topic - First operand
* Input2,3,4,.... - Signal topics - All further operands
* Output1 - Signal topic - Result of AND operation of all input operands