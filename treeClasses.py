import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import threading
import time

# Base Navigation behavior for Nav2
class NavigationBehavior(py_trees.behaviour.Behaviour):
    """
    Base class for navigation behaviors that use Nav2
    """
    def __init__(self, name, pose_x, pose_y, pose_z=0.0, quat_x=0.0, quat_y=0.0, quat_z=0.0, quat_w=1.0, 
                 frame_id="map", timeout=600.0):
        super(NavigationBehavior, self).__init__(name)
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.pose_z = pose_z
        self.quat_x = quat_x
        self.quat_y = quat_y
        self.quat_z = quat_z
        self.quat_w = quat_w
        self.frame_id = frame_id
        self.timeout = timeout
        self.node = None
        self.nav_action_client = None
        self.goal_handle = None
        self.feedback = None
        self.status = None
        self.result_future = None
        self.goal_id = None

    def setup(self, **kwargs):
        """
        Setup ROS 2 node and action client
        """
        try:
            self.node = kwargs.get('node')
            if self.node is None:
                # Create a ROS node if one wasn't passed in
                rclpy.init(args=None)
                self.node = rclpy.create_node(f'{self.name}_node')
                
            # Create the action client
            self.nav_action_client = ActionClient(
                self.node, 
                NavigateToPose, 
                'navigate_to_pose'
            )
            
            # Wait for action server
            self.logger.info(f"{self.name}: Waiting for Nav2 action server...")
            self.nav_action_client.wait_for_server()
            self.logger.info(f"{self.name}: Nav2 action server available")
            return True
        except Exception as e:
            self.logger.error(f"{self.name}: Error in setup: {e}")
            return False

    def initialise(self):
        """
        Initialize the behavior for a new run
        """
        self.logger.info(f"{self.name}: Initializing navigation request")
        self.feedback = None
        self.status = None
        self.result_future = None
        self.goal_id = None
        
        # Create and send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        
        # Set the goal pose
        goal_msg.pose.pose.position.x = self.pose_x
        goal_msg.pose.pose.position.y = self.pose_y
        goal_msg.pose.pose.position.z = self.pose_z
        goal_msg.pose.pose.orientation.x = self.quat_x
        goal_msg.pose.pose.orientation.y = self.quat_y
        goal_msg.pose.pose.orientation.z = self.quat_z
        goal_msg.pose.pose.orientation.w = self.quat_w
        
        # Send the goal
        self.logger.info(f"{self.name}: Sending goal to ({self.pose_x}, {self.pose_y})")
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def update(self):
        """
        Check the status of the navigation request
        """
        # If we don't have a result future yet, we're waiting for the goal to be accepted
        if self.result_future is None:
            self.logger.debug(f"{self.name}: Waiting for goal to be accepted")
            return py_trees.common.Status.RUNNING
            
        # Check if the result is ready
        if self.result_future.done():
            status = self.result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info(f"{self.name}: Navigation succeeded")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.warning(f"{self.name}: Navigation failed with status: {status}")
                return py_trees.common.Status.FAILURE
        else:
            # Print feedback if available
            if self.feedback:
                remaining_distance = self.feedback.distance_remaining
                self.logger.debug(f"{self.name}: Distance remaining: {remaining_distance:.2f}")
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Clean up when the behavior finishes or is interrupted
        """
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.logger.info(f"{self.name}: Canceling goal")
            cancel_future = self.goal_handle.cancel_goal_async()
            # Wait for cancel response
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=2.0)
        
        self.feedback = None
        self.status = None
        self.goal_handle = None
        self.result_future = None
        
        self.logger.info(f"{self.name}: Terminated with status {new_status}")

    def feedback_callback(self, feedback_msg):
        """
        Store the latest feedback from Nav2
        """
        self.feedback = feedback_msg.feedback

    def goal_response_callback(self, future):
        """
        Handle the goal response (accepted or rejected)
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.logger.error(f"{self.name}: Goal rejected")
            self.status = py_trees.common.Status.FAILURE
            return
            
        self.logger.info(f"{self.name}: Goal accepted")
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle the result of the navigation action
        """
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info(f"{self.name}: Navigation succeeded")
        else:
            self.logger.warning(f"{self.name}: Navigation failed with status {status}")
            
        self.status = status


class GoToChargeStation(NavigationBehavior):
    """
    Navigate to the charging station location
    """
    def __init__(self, name="GoToChargeStation", pose_x=1.0, pose_y=2.0):
        super(GoToChargeStation, self).__init__(name=name, pose_x=pose_x, pose_y=pose_y)
        

class GoToSafeLocation(NavigationBehavior):
    """
    Navigate to a designated safe location
    """
    def __init__(self, name="GoToSafeLocation", pose_x=-1.0, pose_y=-2.0):
        super(GoToSafeLocation, self).__init__(name=name, pose_x=pose_x, pose_y=pose_y)


class GoToTarget(NavigationBehavior):
    """
    Navigate to a target person
    """
    def __init__(self, name="GoToTarget"):
        # Initial values will be updated from blackboard
        super(GoToTarget, self).__init__(name=name, pose_x=0.0, pose_y=0.0)
        
    def initialise(self):
        """
        Get target coordinates from blackboard before navigation
        """
        blackboard = py_trees.blackboard.Blackboard()
        target_pose = blackboard.get("target_pose", None)
        
        if target_pose is not None:
            self.pose_x = target_pose.pose.position.x
            self.pose_y = target_pose.pose.position.y
            self.pose_z = target_pose.pose.position.z
            self.quat_x = target_pose.pose.orientation.x
            self.quat_y = target_pose.pose.orientation.y
            self.quat_z = target_pose.pose.orientation.z
            self.quat_w = target_pose.pose.orientation.w
            self.frame_id = target_pose.header.frame_id
            
        # Call parent initialise to start navigation
        super(GoToTarget, self).initialise()


class HeartbeatMonitor(py_trees.behaviour.Behaviour):
    """
    Monitor heartbeats from critical system nodes.
    Returns SUCCESS if all monitored nodes are alive, FAILURE otherwise.
    """
    def __init__(self, name="HeartbeatMonitor", critical_nodes=None, timeout=3.0):
        super(HeartbeatMonitor, self).__init__(name)
        self.critical_nodes = critical_nodes or ["navigation_node", "perception_node"]  # Default critical nodes
        self.timeout = timeout
        self.node = None
        self.subscription = None
        self.last_heartbeats = {}
        self.lost_nodes = set()
        
    def setup(self, **kwargs):
        """
        Set up the ROS subscriber for heartbeat messages
        """
        try:
            self.node = kwargs.get('node')
            if self.node is None:
                self.logger.error("No ROS node provided to HeartbeatMonitor")
                return False
                
            # Create subscription to heartbeat topic
            self.subscription = self.node.create_subscription(
                String, 
                '/heartbeat', 
                self.heartbeat_callback, 
                10
            )
            self.logger.info("HeartbeatMonitor: Subscribed to /heartbeat")
            return True
        except Exception as e:
            self.logger.error(f"HeartbeatMonitor setup error: {e}")
            return False
            
    def initialise(self):
        """
        Reset the lost nodes set when behavior is initialized
        """
        self.logger.info("HeartbeatMonitor: Initializing")
        self.lost_nodes = set()  # Reset lost nodes tracking
    
    def update(self):
        """
        Check if any critical nodes are lost
        """
        if not self.node:
            self.logger.error("HeartbeatMonitor: No ROS node available")
            return py_trees.common.Status.FAILURE
            
        # Get current time
        current_time = self.node.get_clock().now().seconds_nanoseconds()[0]
        
        # First check if we've received any heartbeats yet
        if not self.last_heartbeats:
            self.logger.debug("HeartbeatMonitor: No heartbeats received yet")
            # If we have critical nodes defined but haven't received any heartbeats,
            # consider this a failure after a grace period
            return py_trees.common.Status.RUNNING
        
        # Check each critical node
        missing_nodes = []
        for node_name in self.critical_nodes:
            if node_name not in self.last_heartbeats:
                missing_nodes.append(f"{node_name} (never seen)")
            else:
                last_time = self.last_heartbeats[node_name]
                if current_time - last_time > self.timeout:
                    missing_nodes.append(f"{node_name} (timeout: {current_time - last_time:.1f}s)")
        
        if missing_nodes:
            self.logger.warning(f"HeartbeatMonitor: Missing critical nodes: {', '.join(missing_nodes)}")
            # Update blackboard with lost nodes information
            blackboard = py_trees.blackboard.Blackboard()
            blackboard.set("lost_nodes", missing_nodes)
            return py_trees.common.Status.FAILURE
        else:
            self.logger.debug("HeartbeatMonitor: All critical nodes are active")
            return py_trees.common.Status.SUCCESS
            
    def heartbeat_callback(self, msg):
        """
        Process incoming heartbeat messages
        """
        try:
            node_name, timestamp = msg.data.split(",")
            timestamp = float(timestamp)
            
            # Store the heartbeat timestamp
            self.last_heartbeats[node_name] = timestamp
            
            # If the node was previously lost, log its recovery
            if node_name in self.lost_nodes:
                self.logger.info(f"HeartbeatMonitor: Node {node_name} recovered")
                self.lost_nodes.remove(node_name)
                
        except Exception as e:
            self.logger.error(f"HeartbeatMonitor: Error processing heartbeat: {e}")
            
    def terminate(self, new_status):
        """
        Clean up when the behavior is terminated
        """
        self.logger.info(f"HeartbeatMonitor: Terminated with status {new_status}")


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
        blackboard = py_trees.blackboard.Blackboard()
        battery_level = blackboard.get("battery_level", 0.5)  # Default 0.5 if not set
        
        self.logger.debug(f"Battery level: {battery_level:.2f}")
        
        if battery_level < self.threshold:
            self.logger.info("Battery is low")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class DetectPeopleSubscriber(py_trees.behaviour.Behaviour):
    def __init__(self, name="DetectPeopleSubscriber"):
        super(DetectPeopleSubscriber, self).__init__(name)
        self.node = None
        self.subscription = None
        self.detected_people = []

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node is None:
            self.logger.error("No ROS node provided to DetectPeopleSubscriber")
            return False
        self.subscription = self.node.create_subscription(
            DetectedPeopleMsg,  # Replace with the actual message type
            '/detected_people',
            self.callback,
            10
        )
        return True

    def callback(self, msg):
        self.detected_people = msg.people  # Replace with actual message field

    def update(self):
        if self.detected_people:
            blackboard = py_trees.blackboard.Blackboard()
            blackboard.set("detected_people", self.detected_people)
            self.logger.info(f"Detected {len(self.detected_people)} people")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("No people detected")
            return py_trees.common.Status.FAILURE


class SelectTargetService(py_trees.behaviour.Behaviour):
    def __init__(self, name="SelectTargetService"):
        super(SelectTargetService, self).__init__(name)
        self.node = None
        self.client = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node is None:
            self.logger.error("No ROS node provided to SelectTargetService")
            return False
        self.client = self.node.create_client(SelectTargetServiceMsg, '/select_target')  # Replace with actual service type
        return self.client.wait_for_service(timeout_sec=5.0)

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        detected_people = blackboard.get("detected_people", [])
        if not detected_people:
            self.logger.warning("No people to select from")
            return py_trees.common.Status.FAILURE

        # Call the service
        request = SelectTargetServiceMsg.Request()
        request.people = detected_people  # Replace with actual request field
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response and response.success:
            blackboard.set("selected_person", response.selected_person)
            self.logger.info(f"Selected person {response.selected_person.id}")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.warning("Target selection failed")
            return py_trees.common.Status.FAILURE


class TargetReached(py_trees.behaviour.Behaviour):
    """Check if the robot has reached the target"""
    def __init__(self, name="TargetReached", distance_threshold=0.5):
        super(TargetReached, self).__init__(name)
        self.distance_threshold = distance_threshold
        
    def update(self):
        # In a real system, this would compare current robot pose with target pose
        # For now, we'll use a flag set by the GoToTarget behavior
        blackboard = py_trees.blackboard.Blackboard()
        target_reached = blackboard.get("target_reached", False)
        
        if target_reached:
            self.logger.info("Target reached")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class QuizPrompt(py_trees.behaviour.Behaviour):
    """Prompt a quiz for the target person"""
    def __init__(self, name="QuizPrompt"):
        super(QuizPrompt, self).__init__(name)

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        quiz_active = blackboard.get("quiz_active", False)

        if quiz_active:
            self.logger.info("Quiz is active")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.warning("Quiz is not active")
            return py_trees.common.Status.FAILURE


class SelectNewTarget(py_trees.behaviour.Behaviour):
    """Select a new target if current target is not valid"""
    def __init__(self, name="SelectNewTarget"):
        super(SelectNewTarget, self).__init__(name)
        
    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        detected_people = blackboard.get("detected_people", [])
        current_person = blackboard.get("selected_person", None)
        
        if not detected_people or current_person is None:
            self.logger.warning("No people detected or no current person selected")
            return py_trees.common.Status.FAILURE
            
        # Select a different person than the current one
        available_people = [p for p in detected_people if p["id"] != current_person["id"]]
        
        if not available_people:
            self.logger.info("No alternative people available")
            return py_trees.common.Status.FAILURE
            
        # Select the first available alternative
        new_person = available_people[0]
        
        # Create a target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = new_person["pose_x"]
        target_pose.pose.position.y = new_person["pose_y"]
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.w = 1.0  # Default orientation
        
        # Update the blackboard
        blackboard.set("selected_person", new_person)
        blackboard.set("target_pose", target_pose)
        blackboard.set("target_reached", False)  # Reset target reached flag
        
        self.logger.info(f"Selected new person {new_person['id']} at ({new_person['pose_x']}, {new_person['pose_y']})")
        return py_trees.common.Status.SUCCESS


def create_safety_subtree(critical_nodes=None):
    """Create the safety monitoring subtree with heartbeat monitoring"""
    # Create heartbeat monitor for critical nodes
    heartbeat = HeartbeatMonitor(critical_nodes=critical_nodes)
    
    # Use an inverter to trigger on heartbeat failure (when any critical node is lost)
    heartbeat_inverter = py_trees.decorators.Inverter(
        name="HeartbeatFailureDetector",
        child=heartbeat
    )
    
    # Emergency stop check
    emergency_stop = py_trees.behaviours.CheckBlackboardVariable(
        name="EmergencyStopActivated", 
        variable_name="emergency_stop_activated", 
        expected_value=True
    )
    
    # Create fallback for safety failures (go to safe location)
    go_to_safe_location = GoToSafeLocation()
    
    # Safety fallback - if any safety check triggers, go to safe location
    safety_fallback = py_trees.composites.Sequence(name="SafetyResponseSequence")
    
    # Parallel for detecting safety issues - succeeds if ANY issue is detected
    safety_detector = py_trees.composites.Parallel(
        name="SafetyIssueDetector",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    safety_detector.add_children([emergency_stop, heartbeat_inverter])
    
    # Only navigate to safe location if a safety issue is detected
    safety_fallback.add_children([safety_detector, go_to_safe_location])
    
    return safety_fallback


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
    # Detect people and select target
    detect_people = DetectPeopleSubscriber()
    select_target = SelectTargetService()
    go_to_target = GoToTarget()

    # Quiz state subscriber
    quiz_state_subscriber = QuizStateSubscriber()

    # Target reached check with timeout
    target_reached = TargetReached()
    target_reached_with_timeout = py_trees.decorators.Timeout(
        name="TargetReachedTimeout",
        child=target_reached,
        duration=30.0
    )

    # Quiz handling
    quiz_prompt = QuizPrompt()
    quiz_sequence = py_trees.composites.Sequence(name="QuizSequence")
    quiz_sequence.add_children([target_reached_with_timeout, quiz_prompt])

    # Select new target if quiz sequence fails
    select_new_target = SelectNewTarget()
    quiz_fallback = py_trees.composites.Selector(name="QuizFallback")
    quiz_fallback.add_children([quiz_sequence, select_new_target])

    # Main task sequence
    main_sequence = py_trees.composites.Sequence(name="MainSequence")
    main_sequence.add_children([quiz_state_subscriber, detect_people, select_target, go_to_target, quiz_fallback])

    return main_sequence


def create_root_tree(node=None, critical_nodes=None):
    """Create the complete behavior tree with proper heartbeat integration"""
    # Create blackboard
    blackboard = py_trees.blackboard.Blackboard()
    
    # Create priority selector (priorities are ordered from highest to lowest)
    root = py_trees.composites.Selector(name="RootSelector")
    
    # Add subtrees in priority order (safety first, then battery, then main task)
    safety_subtree = create_safety_subtree(critical_nodes=critical_nodes)
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
    
    # Define critical nodes that must be alive for safe operation
    critical_nodes = ["navigation_node", "perception_node", "controller_node"]
    
    # Create the behavior tree
    tree = create_root_tree(node=node, critical_nodes=critical_nodes)
    
    # Initialize the blackboard with default values
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("battery_level", 0.7)
    blackboard.set("emergency_stop_activated", False)
    blackboard.set("critical_task_active", False)
    blackboard.set("target_reached", False)
    blackboard.set("quiz_active", False)
    blackboard.set("lost_nodes", [])
    
    # Run the tree
    try:
        # Create a separate thread for spinning ROS
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        # Visualization
        py_trees.display.render_dot_tree(tree.root)
        
        # Simulate a heartbeat publisher for testing
        test_publisher = node.create_publisher(String, '/heartbeat', 10)
        
        def publish_test_heartbeats():
            """Publish test heartbeats for simulation"""
            while rclpy.ok():
                for node_name in critical_nodes:
                    msg = String()
                    msg.data = f"{node_name},{node.get_clock().now().seconds_nanoseconds()[0]}"
                    test_publisher.publish(msg)
                time.sleep(0.5)  # Publish every 0.5 seconds
                
        heartbeat_thread = threading.Thread(target=publish_test_heartbeats)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()
        
        # Simulate behavior tree execution
        for i in range(20):
            print(f"\n--- Tick {i} ---")
            
            # Simulate different scenarios
            if i == 5:
                print("Simulating low battery...")
                blackboard.set("battery_level", 0.1)
            elif i == 8:
                print("Simulating target reached...")
                blackboard.set("target_reached", True)
            elif i == 10:
                print("Simulating heartbeat failure for navigation_node...")
                # Stop sending heartbeats for navigation_node by modifying critical_nodes list
                critical_nodes.remove("navigation_node")
            elif i == 15:
                print("Simulating emergency stop...")
                blackboard.set("emergency_stop_activated", True)
                
            # Tick the tree
            tree.tick()
            
            # Display the tree ASCII art
            print(py_trees.display.ascii_tree(tree.root))
            
            # Report any lost nodes
            lost_nodes = blackboard.get("lost_nodes")
            if lost_nodes:
                print(f"Lost nodes detected: {lost_nodes}")
            
            # Sleep to simulate real-time execution
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()