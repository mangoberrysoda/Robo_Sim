# Implement range-limited direct communication
# Create basic message types (survivor_found, help_needed)
# Add simple message passing between robots

#class SimpleCommNetwork for communication between robots with attributes: range-limited communication, direct communication only if within range, basic message types (survivor_found, help_needed), simple message passing between robots
class SimpleCommNetwork:
    def __init__(self, range_meters=30):
        self.range_meters = range_meters  # Communication range in meters
        self.robots = {}  # Dictionary to store robot positions

    def can_communicate(self, robot1_id, robot2_id):
        """Check if two robots can communicate based on their positions."""
        if robot1_id not in self.robots or robot2_id not in self.robots:
            return False

        pos1 = self.robots[robot1_id]
        pos2 = self.robots[robot2_id]

        distance = ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
        return distance <= self.range_meters

    def send_message(self, sender_id, receiver_id, message):
        """Send a message from sender to receiver if within range."""
        if sender_id not in self.robots or receiver_id not in self.robots:
            return False

        sender_position = self.robots[sender_id]
        receiver_position = self.robots[receiver_id]

        distance = ((sender_position[0] - receiver_position[0]) ** 2 + (sender_position[1] - receiver_position[1]) ** 2) ** 0.5
        if distance <= self.range_meters:
            # In a real scenario, we would handle the message delivery here
            return True  # Message sent successfully
        return False  # Receiver out of range

# Implement basic multi-hop communication
# Create simple routing (shortest path)
# Add message relay system
#Class MessageRouter for routing messages between robots with attributes: multi-hop communication, simple routing (shortest path), message relay system
class MessageRouter:
    def __init__(self):
        self.robots = {}  # Dictionary to store robot positions

   
    def find_shortest_path(self, sender_id, receiver_id):
        """Find the shortest path between sender and receiver using direct communication."""
        if sender_id not in self.robots or receiver_id not in self.robots:
            return None

        sender_position = self.robots[sender_id]
        receiver_position = self.robots[receiver_id]

        # For simplicity, we assume direct communication only
        distance = ((sender_position[0] - receiver_position[0]) ** 2 + (sender_position[1] - receiver_position[1]) ** 2) ** 0.5
        return distance  # In a real scenario, this would return a path or hops

    def relay_message(self, sender_id, receiver_id, message):
        """Relay a message from sender to receiver if within range."""
        if sender_id not in self.robots or receiver_id not in self.robots:
            return False

        sender_position = self.robots[sender_id]
        receiver_position = self.robots[receiver_id]

        # Check if direct communication is possible
        distance = ((sender_position[0] - receiver_position[0]) ** 2 + (sender_position[1] - receiver_position[1]) ** 2) ** 0.5
        if distance <= 30:  # Assuming a fixed communication range of 30 meters
            return True  # Message sent directly

        # If not directly reachable, in a real scenario we would find intermediate robots to relay the message
        return False  # For simplicity, we do not implement multi-hop in detail here

# Implement survivor detection sharing
# Create confirmation request system
# Add basic task coordination

#Class BasicCoordination for coordinating tasks between robots with attributes: survivor detection sharing, confirmation request system
class BasicCoordination:
    def __init__(self):
        self.detections = {}  # Dictionary to store detections by robot_id

    def share_detection(self, robot_id, detection):
        """Share a detection made by a robot."""
        if robot_id not in self.detections:
            self.detections[robot_id] = []
        self.detections[robot_id].append(detection)

    def request_confirmation(self, detection):
        """Request confirmation for a detection from other robots."""
        confirmations = []
        for robot_id, robot_detections in self.detections.items():
            if detection in robot_detections:
                confirmations.append(robot_id)
        return confirmations

    def coordinate_task(self, task, robots):
        """Coordinate a task among available robots."""
        assigned_robots = []
        for robot in robots:
            if robot.current_task == "idle":
                robot.current_task = task
                assigned_robots.append(robot.robot_id)
        return assigned_robots
    

