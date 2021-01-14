# Test 1 : Braking test

This file describes textually how the braking test is to be scripted. All semantic definitions are taken from test1_interfaces.yaml.

```yaml
Test_runner:
  Concurrence_1: # Run main test sequence
    Sequence_1: Wait for Signal.ui.start_test # User presses start button
    Sequence_2: Emit Signal.runner.start_robot # Tell command interpreter to start the robot
    Sequence_3: Wait for Signals.robot.max_velocity # Wait until an calculator tells us robot has reached max vel
    Sequence_4: Emit Signal.runner.stop_robot # Tell command interpreter to stop the robot
  Concurrence_2: # Abort if necessary
    Sequence_1: Wait for Signals.ui.abort_test # User presses abort button
    Sequence_2: Abort entire test # killall -9 rosmaster ??
```

@ipa-rwu : Please fill this part if/when you have time :

```sequence {theme="simple"}
Signals.Test.Start->Signals.robot.do_start: roslaunch turtlebot3_bringup
Signals.robot.do_start->Signals.subtest_1.start: pub /cmd_vel velocity 0.5m/s, calculator confirm
Signals.subtest_1.start->Signals.subtest_1.finish: pub /cmd_vel velocity 0m/s
Signals.subtest_1.finish->Signals.Test.Finish:
```