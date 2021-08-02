# araig_calculators

Inherit **[comparator](src/comparator/comparator.py)**, only need to change **Compare()** function

* [pose comparator](src/comparator/lib/comparator_two_topics_bool_pose.py):
   * name: comparatorTwoTopicsBoolPose
   * input:  two topics in **geometry_msgs/PoseStamped**
   * output: bool

*  [two topics comparator](src/comparator/lib/comparator_two_topics_bool.py): 
   * name: comparatorTwoTopicsBool
   * input:  two topics with one value for each topic
   * output: bool

*  [topic and param comparator](src/comparator/lib/comparator_param_bool.py): 
   * name: comparatorParamBool
   * input:  topic and param with one value
   * output: bool

* [stopwatch](src/comparator/lib/stop_watch.py) : 
   * name: stopWatch
   * input: any two topics (compare header stamp)
   * output: duration in float64

* [edge detector](src/comparators/edge_detector.py) :
   * name: edgeDetector
   * inputs: 1 BoolStamped - /in_bool
   * outputs: 2 BoolStamped - /out_high, /out_low
   * operation: Detects a rising or falling edge. If rising edge: high=rue/low=false; if falling edge: high=false/low=true
