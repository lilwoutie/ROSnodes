import py_trees
import rclpy
from rclpy.node import Node
from nav2_behaviors import GoToChargeStation, GoToSafeLocation

# Create a blackboard for sharing data between behaviors
blackboard = py_trees.blackboard.Blackboard()

class BatteryLow(py_trees.behaviour.Behaviour):
    """Check if the battery level is below a threshold"""
    def __init__(self, name="BatteryLow", threshold=0.2):
        super(BatteryLow, self).__init__(name)
        self.threshold = threshold
        
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # Setup battery monitoring (could be a subscriber to a battery state topic)
        return True
        
    def update(self):
        # Get battery level (replace with actual battery level check)
        battery_level = blackboard.get("battery_level", 0.5)  # Default 0.5 if not set
        
        self.logger.debug(f"Battery level: {battery_level:.2f}")
        
        if battery_level < self.threshold:
            self.logger.info("Battery is low")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class HeartbeatMonitor(py_trees.behaviour.Behaviour):
    """Monitor system heartbeat for failures"""
    def __init__(self, name="HeartbeatMonitor", timeout=1.0):
        super(HeartbeatMonitor, self).__init__(name)
        self.timeout = timeout
        self.last_heartbeat_time = None
        
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # Setup heartbeat monitoring (could be a subscriber to a heartbeat topic)
        # For now we'll use the blackboard
        return True
        
    def initialise(self):
        self.last_heartbeat_time = self.node.get_clock().now() if self.node else None
        
    def update(self):
        # Get current time
        current_time = self.node.get_clock().now() if self.node else None
        
        # Get latest heartbeat time
        latest_heartbeat = blackboard.get("last_heartbeat_time", self.last_heartbeat_time)
        
        if latest_heartbeat is None or current_time is None:
            return py_trees.common.Status.FAILURE
            
        # Calculate time difference
        time_diff = (current_time - latest_heartbeat).nanoseconds / 1e9
        
        if time_diff > self.timeout:
            self.logger.warning(f"Heartbeat timeout: {time_diff:.2f}s > {self.timeout:.2f}s")
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS


def create_safety_subtree():
    """Create the safety monitoring subtree"""
    # Create safety checks
    heartbeat = HeartbeatMonitor()
    
    # Use an inverter to trigger on heartbeat failure
    heartbeat_inverter = py_trees.decorators.Inverter(
        name="HeartbeatInverter",
        child=heartbeat
    )
    
    # Emergency stop check
    emergency_stop = py_trees.behaviours.CheckBlackboardVariable(
        name="EmergencyStopActivated", 
        variable_name="emergency_stop_activated", 
        expected_value=True
    )
    
    # Safety parallel - triggers if ANY safety check fails
    safety_parallel = py_trees.composites.Parallel(
        name="SafetyChecks",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    safety_parallel.add_children([emergency_stop, heartbeat_inverter])
    
    # Add safe location fallback
    go_to_safe_location = GoToSafeLocation()
    
    # Safety sequence - only go to safe location if safety check fails
    safety_sequence = py_trees.composites.Sequence(name="SafetySequence")
    safety_sequence.add_children([safety_parallel, go_to_safe_location])
    
    return safety_sequence


def create_battery_subtree():
    """Create the battery management subtree"""
    # Battery check
    battery_low = BatteryLow()
    
    # Check if not in critical task
    not_in_critical_task = py_trees.behaviours.CheckBlackboardVariable(
        name="NotInCriticalTask", 
        variable_name="critical_task_active", 
        expected_value=False
    )
    
    # Go to charge station
    go_to_charge_station = GoToChargeStation()
    
    # Battery sequence - check battery is low AND not in critical task before charging
    battery_sequence = py_trees.composites.Sequence(name="BatterySequence")
    battery_sequence.add_children([battery_low, not_in_critical_task, go_to_charge_station])
    
    return battery_sequence


def create_main_task_subtree():
    """Create the main robot task subtree"""
    # Placeholder behaviors for main task
    # Replace these with actual implementations
    detect_people = py_trees.behaviours.Success(name="DetectPeople")
    select_target = py_trees.behaviours.Success(name="SelectTarget")
    go_to_target = py_trees.behaviours.Success(name="GoToTarget")
    target_reached = py_trees.behaviours.Success(name="TargetReached")
    quiz_prompt = py_trees.behaviours.Success(name="QuizPrompt")
    select_new_target = py_trees.behaviours.Success(name="SelectNewTarget")
    
    # Quiz sequence
    quiz_sequence = py_trees.composites.Sequence(name="QuizSequence")
    quiz_sequence.add_children([target_reached, quiz_prompt])
    
    # Quiz fallback
    quiz_fallback = py_trees.composites.Selector(name="QuizFallback")
    quiz_fallback.add_children([quiz_sequence, select_new_target])
    
    # Main task sequence
    main_sequence = py_trees.composites.Sequence(name="MainSequence")
    main_sequence.add_children([detect_people, select_target, go_to_target, quiz_fallback])
    
    return main_sequence


def create_root_tree(node=None):
    """Create the complete behavior tree"""
    # Create priority selector (priorities are ordered from highest to lowest)
    root = py_trees.composites.Selector(name="RootSelector")
    
    # Add subtrees in priority order (safety first, then battery, then main task)
    safety_subtree = create_safety_subtree()
    battery_subtree = create_battery_subtree()
    main_task_subtree = create_main_task_subtree()
    
    # Add all subtrees to root
    root.add_children([safety_subtree, battery_subtree, main_task_subtree])
    
    # Create a tree manager
    tree = py_trees.trees.BehaviourTree(root)
    
    # Setup the tree with the ROS node
    tree.setup(timeout=15, node=node)
    
    return tree


if __name__ == "__main__":
    # Initialize ROS
    rclpy.init(args=None)
    node = rclpy.create_node("behavior_tree_node")
    
    # Create the behavior tree
    tree = create_root_tree(node=node)
    
    # Initialize the blackboard with default values
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("battery_level", 0.7)
    blackboard.set("emergency_stop_activated", False)
    blackboard.set("critical_task_active", False)
    blackboard.set("last_heartbeat_time", node.get_clock().now())
    
    # Run the tree
    try:
        # Create a separate threading for spinning ROS
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        # Visualization
        py_trees.display.render_dot_tree(tree.root)
        
        # Simulate behavior tree execution
        for i in range(10):
            print(f"\n--- Tick {i} ---")
            
            # Update blackboard values to simulate different scenarios
            if i == 3:
                blackboard.set("battery_level", 0.1)  # Simulate low battery at tick 3
            if i == 7:
                blackboard.set("emergency_stop_activated", True)  # Simulate emergency stop at tick 7
                
            # Tick the tree
            tree.tick()
            
            # Display the tree ASCII art
            print(py_trees.display.ascii_tree(tree.root))
            
            # Sleep to simulate real-time execution
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()