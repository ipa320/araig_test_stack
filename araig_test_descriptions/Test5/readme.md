# Test5

Test 5 replaces all EVENTS with SIGNALS.   
An Event is a pulse. Stays high for a duration, then goes back low.   
A Signal is a level. Stays high or stays low.   
This should be backported to Test1.    

## Test Runner

```yaml
Concurrence1:
    Sequence1:  Wait for Signal.ui.start_test
    Sequence2:  Set Signal.runner.start_robot TRUE
    Sequence3:  Wait for Signal.ui.fail_test
                    OR Signal.calc.robot_in_collission
                    OR Signal.calc.robot_reached_goal_and_stopped
    Sequence4:  IF Signal.calc.robot_reached_goal_and_stopped
                    Set Signal.runner.test_succeeded TRUE
                ELSE
                    Set Signal.runner.test_failed TRUE
    Sequence5:  Set Signal.runner.test_completed TRUE
Concurrence2:
    Sequence1:  Wait for Signal.ui.abort_test
    Sequence2:  Abort test
```

## GUI

Input Buttons - Start, Stop   
Output LEDs - Test Succeeded, Test Failed, Robot Collision   
(DEBUG ONLY) Signal LEDs - Show all signals - hardcoded   

## ToDos

* [] - Add a reset feature to calculators that need it (Ex: Recorders stop recording and save bag, loggers save file and open new one, plotters clear output etc...)   