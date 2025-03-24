import py_trees

#TO BE IMPLEMENTED
# class BatteryLow(py_trees.behaviour.Behaviour):        WIP
# class GoToChargeStation(py_trees.behaviour.Behaviour): NAV2
# class BatteryCheck(py_trees.behaviour.Behaviour):      BACHELORS
# class BumperActivated(py_trees.behaviour.Behaviour):   WIP
# class EmergencyStopActivated(py_trees.behaviour.Behaviour): WIP
# class DetectPeople(py_trees.behaviour.Behaviour):      WIP
# class SelectTarget(py_trees.behaviour.Behaviour):      READY TO BE IMPLEMENTED
# class GoToTarget(py_trees.behaviour.Behaviour):        NAV2
# class QuizPrompt(py_trees.behaviour.Behaviour):        BACHELORS
# class TargetReached(py_trees.behaviour.Behaviour):     NAV2
# class SelectNewTarget(py_trees.behaviour.Behaviour):   TO BE THOUGHT OUT


def create_battery_tree():
    # Battery check sequence
    battery_low = BatteryLow()
    not_in_critical_task = py_trees.behaviours.CheckBlackboardVariable(
        name="NotInCriticalTask", variable_name="critical_task_active", expected_value=False
    )
    go_to_charge_station = GoToChargeStation()

    battery_check_sequence = py_trees.composites.Sequence(name="BatteryCheck")
    battery_check_sequence.add_children([battery_low, not_in_critical_task, go_to_charge_station])

    # Safety checks
    bumper_activated = py_trees.behaviours.CheckBlackboardVariable(
        name="BumperActivated", variable_name="bumper_activated", expected_value=True
    )
    emergency_stop_activated = py_trees.behaviours.CheckBlackboardVariable(
        name="EmergencyStopActivated", variable_name="emergency_stop_activated", expected_value=True
    )
    safety_parallel = py_trees.composites.Parallel(
        name="SafetyChecks", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    safety_parallel.add_children([bumper_activated, emergency_stop_activated])

    # Main task execution
    main_task = create_main_task_tree()
    main_task_parallel = py_trees.composites.Parallel(
        name="MainTaskExecution", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    main_task_parallel.add_children([main_task, bumper_activated, emergency_stop_activated])

    # Battery tree fallback
    battery_tree = py_trees.composites.Selector(name="BatteryTree")
    battery_tree.add_children([battery_check_sequence, safety_parallel, main_task_parallel])

    return battery_tree


def create_main_task_tree():
    # Detect people and select target
    detect_people = py_trees.behaviours.Success(name="DetectPeople")
    select_target = py_trees.behaviours.Success(name="SelectTarget")
    go_to_target = py_trees.behaviours.Success(name="GoToTarget")

    # Quiz handling
    quiz_prompt = py_trees.behaviours.Success(name="QuizPrompt")
    target_reached = py_trees.behaviours.Success(name="TargetReached")
    quiz_sequence = py_trees.composites.Sequence(name="QuizSequence")
    quiz_sequence.add_children([target_reached, quiz_prompt])

    select_new_target = py_trees.behaviours.Success(name="SelectNewTarget")
    quiz_fallback = py_trees.composites.Selector(name="QuizFallback")
    quiz_fallback.add_children([quiz_sequence, select_new_target])

    # Main task sequence
    main_task = py_trees.composites.Sequence(name="MainTask")
    main_task.add_children([detect_people, select_target, go_to_target, quiz_fallback])

    return main_task

if __name__ == "__main__":
    # Create the behavior tree
    root = create_battery_tree()

    # Create a tree manager
    tree = py_trees.trees.BehaviourTree(root)

    # Tick the tree (simulate execution)
    for _ in range(10):
        tree.tick()